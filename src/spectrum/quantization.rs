//! Encoder side: global gain estimation, quantization, bit consumption
//! estimation and rate control (section 3.3.10), residual coding (3.3.11)
//! and noise level estimation (3.3.12).

use super::tables::{BITS, CONTEXT_LOOKUP};
use super::{
    GAIN_STEPS_PER_DECADE, MAX_GAIN_INDEX, MAX_NOISE_LEVEL, NOISE_LEVEL_STEPS,
    NOISE_LEVEL_ZERO_INDEX, global_gain_offset, is_noise_line, lsb_truncation_allowed,
    next_context, noise_filling_range, rate_context_offset,
};
use crate::bitstream::BIT_RESOLUTION;
use crate::config::{Bandwidth, Config, MAX_SPECTRAL_LINES};
use crate::math::nint;

/// Ratio of the gain steps per decade to the decibels per decade, which
/// scales the energies of the gain estimation.
const DB_TO_GAIN_STEPS: f32 = GAIN_STEPS_PER_DECADE / 20.0;

/// Offset avoiding the logarithm of zero in the energies of the gain
/// estimation, `2^-31` (equation 111).
const ENERGY_OFFSET: f32 = 1.0 / 2_147_483_648.0;

/// Number of iterations of the bisection of the global gain index.
const GAIN_BISECTION_STEPS: usize = 8;

/// Bounds of the smoothed bit offset of the gain estimation (equation 108).
const MAX_BIT_OFFSET: f32 = 40.0;

/// Weight of the previous offset in the smoothing of the bit offset (equation 108).
const PREVIOUS_OFFSET_WEIGHT: f32 = 0.8;

/// Weight of the bit estimation error in the smoothing of the bit offset (equation 108).
const OFFSET_DRIFT_WEIGHT: f32 = 0.2;

/// Estimated bits of a zero block of 4 lines followed by non-zero blocks, in
/// gain steps (section 3.3.10.2).
const ZERO_BLOCK_BITS: f32 = 2.7 * GAIN_STEPS_PER_DECADE / 20.0;

/// Energy above the gain, in gain steps, of the blocks of 4 lines estimated
/// with the high rate approximation (section 3.3.10.2).
const HIGH_RATE_ENERGY: f32 = 43.0 * GAIN_STEPS_PER_DECADE / 20.0;

/// Bit offset of the high rate approximation, in gain steps (section 3.3.10.2).
const HIGH_RATE_OFFSET: f32 = 36.0 * GAIN_STEPS_PER_DECADE / 20.0;

/// Bit offset of the low rate approximation, in gain steps (section 3.3.10.2).
const LOW_RATE_OFFSET: f32 = 7.0 * GAIN_STEPS_PER_DECADE / 20.0;

/// Ratio of the bit budget to the estimated bits of the bisection, in gain
/// steps (section 3.3.10.2).
const BUDGET_SCALE: f32 = 1.4;

/// Rounding offset of the dead-zone quantizer (equation 115).
const QUANTIZER_ROUNDING: f32 = 0.375;

/// Largest magnitude of a quantized value.
const MAX_QUANTIZED: f32 = 32_768.0;

/// Thresholds `t1`, `t2` and `t3` of the global gain adjustment per
/// sampling rate index (section 3.3.10.6).
const ADJUSTMENT_THRESHOLDS: [[f32; 5]; 3] = [
    [80.0, 230.0, 380.0, 530.0, 680.0],
    [500.0, 1025.0, 1550.0, 2075.0, 2600.0],
    [850.0, 1700.0, 2550.0, 3400.0, 4250.0],
];

/// Number of bits of the arithmetic coder side information `nbits_ari`
/// (equation 107).
pub(crate) fn nbits_ari(config: &Config, nbytes: usize) -> i32 {
    let nbits = 8 * nbytes;
    let extra = match nbits {
        0..=1280 => 3,
        1281..=2560 => 4,
        _ => 5,
    };
    config.lastnz_bits() as i32 + extra
}

/// State of the rate control, kept from frame to frame (section 3.3.10.2).
#[derive(Clone, Copy, Debug)]
pub(crate) struct RateControl {
    /// Bit offset of the previous frame (`nbits_offset`).
    nbits_offset: f32,
    /// Bit budget of the spectrum of the previous frame (`nbits_spec`).
    nbits_spec: i32,
    /// Estimated bits of the first quantization of the previous frame (`nbits_est`).
    nbits_est: i32,
    /// Whether the offset is reset (`reset_offset`).
    reset_offset: bool,
}

impl RateControl {
    /// Rate control before the first frame.
    pub(crate) const fn new() -> Self {
        Self {
            nbits_offset: 0.0,
            nbits_spec: 0,
            nbits_est: 0,
            reset_offset: false,
        }
    }

    /// First global gain estimation of the spectrum `xf` for a bit budget of
    /// `nbits_spec` bits (section 3.3.10.2). Returns the global gain index and
    /// the smallest index keeping the quantized values in range.
    pub(crate) fn estimate_gain(
        &mut self,
        config: &Config,
        xf: &[f32],
        nbytes: usize,
        nbits_spec: i32,
    ) -> (i32, i32) {
        let gain_offset = global_gain_offset(config, nbytes);
        let nbits_offset = if self.reset_offset {
            0.0
        } else {
            let drift = (self.nbits_offset + (self.nbits_spec - self.nbits_est) as f32)
                .clamp(-MAX_BIT_OFFSET, MAX_BIT_OFFSET);
            PREVIOUS_OFFSET_WEIGHT * self.nbits_offset + OFFSET_DRIFT_WEIGHT * drift
        };
        let budget = nint(nbits_spec as f32 + nbits_offset);

        // Energies of blocks of 4 lines, in gain steps (equation 111)
        let blocks = xf.len() / 4;
        let mut e = [0.0f32; MAX_SPECTRAL_LINES / 4];
        for (k, ek) in e.iter_mut().enumerate().take(blocks) {
            let energy: f32 = xf[4 * k..4 * k + 4].iter().map(|v| v * v).sum();
            *ek = 10.0 * libm::log10f(ENERGY_OFFSET + energy) * DB_TO_GAIN_STEPS;
        }

        // Bisection on an approximation of the number of bits
        let target = budget as f32 * BUDGET_SCALE * DB_TO_GAIN_STEPS;
        let mut step = (MAX_GAIN_INDEX + 1) as usize;
        let mut index = MAX_GAIN_INDEX;
        for _ in 0..GAIN_BISECTION_STEPS {
            step >>= 1;
            index -= step as i32;
            let gain = (index + gain_offset) as f32;
            let mut bits = 0.0f32;
            let mut zero = true;
            for &ei in e[..blocks].iter().rev() {
                if ei < gain {
                    if !zero {
                        bits += ZERO_BLOCK_BITS;
                    }
                } else {
                    if gain < ei - HIGH_RATE_ENERGY {
                        bits += 2.0 * ei - 2.0 * gain - HIGH_RATE_OFFSET;
                    } else {
                        bits += ei - gain + LOW_RATE_OFFSET;
                    }
                    zero = false;
                }
            }
            if bits > target && !zero {
                index += step as i32;
            }
        }

        // Limitation to the range of the quantized values (equations 112 and 113)
        let xf_max = xf.iter().fold(0.0f32, |m, v| m.max(libm::fabsf(*v)));
        let min_index = if xf_max > 0.0 {
            let ratio = xf_max / (MAX_QUANTIZED - QUANTIZER_ROUNDING);
            libm::ceilf(GAIN_STEPS_PER_DECADE * libm::log10f(1e-31 + ratio)) as i32 - gain_offset
        } else {
            0
        };
        self.reset_offset = index < min_index || xf_max == 0.0;
        if self.reset_offset {
            index = min_index;
        }
        self.nbits_offset = nbits_offset;
        self.nbits_spec = nbits_spec;
        (index, min_index)
    }

    /// Records the estimated number of bits of the first quantization of the frame.
    pub(crate) fn set_bit_estimate(&mut self, nbits_est: i32) {
        self.nbits_est = nbits_est;
    }
}

/// Dead-zone quantization of the spectrum `xf` with the global gain `gain`
/// (equation 115).
pub(crate) fn quantize(xf: &[f32], gain: f32, xq: &mut [i32]) {
    let inverse = 1.0 / gain;
    for (q, &x) in xq.iter_mut().zip(xf) {
        let v = x * inverse;
        let v = if x >= 0.0 {
            libm::floorf(v + QUANTIZER_ROUNDING)
        } else {
            libm::ceilf(v - QUANTIZER_ROUNDING)
        };
        *q = v.clamp(-MAX_QUANTIZED, MAX_QUANTIZED - 1.0) as i32;
    }
}

/// Estimated number of bits of a quantized spectrum (section 3.3.10.4).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct BitEstimate {
    /// Estimated number of bits of the whole spectrum (`nbits_est`).
    pub(crate) nbits: i32,
    /// Estimated number of bits of the spectrum truncated to the bit budget
    /// (`nbits_trunc`).
    pub(crate) nbits_truncated: i32,
    /// Number of spectral lines up to the last non-zero 2-tuple (`lastnz`).
    pub(crate) lastnz: usize,
    /// Number of spectral lines up to the last non-zero 2-tuple fitting the
    /// bit budget (`lastnz_trunc`).
    pub(crate) lastnz_truncated: usize,
}

/// Number of spectral lines up to the last non-zero 2-tuple.
fn last_nonzero(xq: &[i32]) -> usize {
    let mut lastnz = xq.len();
    while lastnz > 2 && xq[lastnz - 1] == 0 && xq[lastnz - 2] == 0 {
        lastnz -= 2;
    }
    lastnz
}

/// Estimation of the number of bits of the quantized spectrum `xq` for a
/// budget of `nbits_spec` bits (section 3.3.10.4).
pub(crate) fn estimate_bits(
    config: &Config,
    nbytes: usize,
    nbits_spec: i32,
    xq: &[i32],
) -> BitEstimate {
    let rate_offset = rate_context_offset(config, nbytes);
    let lsb_allowed = lsb_truncation_allowed(config, nbytes);
    let lastnz = last_nonzero(xq);
    let budget = nbits_spec * BIT_RESOLUTION as i32;

    let mut nbits = 0i32;
    let mut nbits_truncated = 0i32;
    let mut nbits_lsb = 0i32;
    let mut lastnz_truncated = 2;
    let mut c = 0;
    for n in (0..lastnz).step_by(2) {
        let mut t = c + rate_offset;
        if n > xq.len() / 2 {
            t += 256;
        }
        let mut a = xq[n].unsigned_abs();
        let mut b = xq[n + 1].unsigned_abs();
        let mut level = 0;
        while a.max(b) >= 4 {
            let model = usize::from(CONTEXT_LOOKUP[t + level * 1024]);
            nbits += i32::from(BITS[model][16]);
            if level == 0 && lsb_allowed {
                nbits_lsb += 2;
            } else {
                nbits += 2 * BIT_RESOLUTION as i32;
            }
            a >>= 1;
            b >>= 1;
            level = (level + 1).min(3);
        }
        let model = usize::from(CONTEXT_LOOKUP[t + level * 1024]);
        nbits += i32::from(BITS[model][(a + 4 * b) as usize]);

        // Signs, and least significant bits coded in the residual data
        let mut a_lsb = xq[n].unsigned_abs();
        let mut b_lsb = xq[n + 1].unsigned_abs();
        nbits += (a_lsb.min(1) + b_lsb.min(1)) as i32 * BIT_RESOLUTION as i32;
        if level > 0 && lsb_allowed {
            a_lsb >>= 1;
            b_lsb >>= 1;
            if a_lsb == 0 && xq[n] != 0 {
                nbits_lsb += 1;
            }
            if b_lsb == 0 && xq[n + 1] != 0 {
                nbits_lsb += 1;
            }
        }
        if (xq[n] != 0 || xq[n + 1] != 0) && nbits <= budget {
            lastnz_truncated = n + 2;
            nbits_truncated = nbits;
        }
        c = next_context(c, a, b, level);
    }
    let resolution = BIT_RESOLUTION as i32;
    BitEstimate {
        nbits: (nbits + resolution - 1) / resolution + nbits_lsb,
        nbits_truncated: (nbits_truncated + resolution - 1) / resolution,
        lastnz,
        lastnz_truncated,
    }
}

/// Global gain adjustment (section 3.3.10.6): the new global gain index when
/// the spectrum needs to be requantized.
pub(crate) fn adjust_global_gain(
    config: &Config,
    index: i32,
    min_index: i32,
    nbits_est: i32,
    nbits_spec: i32,
) -> Option<i32> {
    let rate = config.sample_rate.index();
    let [t1, t2, t3] = ADJUSTMENT_THRESHOLDS.map(|t| t[rate]);
    let estimate = nbits_est as f32;
    let delta = if estimate < t1 {
        (estimate + 48.0) / 16.0
    } else if estimate < t2 {
        let tmp1 = t1 / 16.0 + 3.0;
        let tmp2 = t2 / 48.0;
        (estimate - t1) * (tmp2 - tmp1) / (t2 - t1) + tmp1
    } else if estimate < t3 {
        estimate / 48.0
    } else {
        t3 / 48.0
    };
    let delta = nint(delta);
    let delta2 = delta + 2;

    let too_many_bits = index < MAX_GAIN_INDEX && nbits_est > nbits_spec;
    let too_few_bits = index > 0 && nbits_est < nbits_spec - delta2;
    if !too_many_bits && !too_few_bits {
        return None;
    }
    let adjusted = if nbits_est < nbits_spec - delta2 {
        index - 1
    } else if index == MAX_GAIN_INDEX - 1 || nbits_est < nbits_spec + delta {
        index + 1
    } else {
        index + 2
    };
    Some(adjusted.max(min_index))
}

/// Residual bits refining the non-zero quantized values (section 3.3.11),
/// returns the number of bits written to `res_bits`.
pub(crate) fn residual_bits(
    xf: &[f32],
    xq: &[i32],
    gain: f32,
    max_bits: i32,
    res_bits: &mut [u8],
) -> usize {
    let mut count = 0;
    for (&x, &q) in xf.iter().zip(xq) {
        if count as i32 >= max_bits {
            break;
        }
        if q != 0 {
            res_bits[count] = u8::from(x >= q as f32 * gain);
            count += 1;
        }
    }
    count
}

/// Noise level `F_NF` of the spectral lines quantized to zero (section 3.3.12).
pub(crate) fn noise_level(
    config: &Config,
    bw: Bandwidth,
    xf: &[f32],
    xq: &[i32],
    gain: f32,
) -> u32 {
    let (start, width) = noise_filling_range(config.frame_duration);
    let stop = bw.stop_line(config.frame_duration);
    let mut sum = 0.0f32;
    let mut count = 0u32;
    for (k, &x) in xf.iter().enumerate().take(stop).skip(start) {
        if is_noise_line(xq, k, width, stop) {
            sum += libm::fabsf(x) / gain;
            count += 1;
        }
    }
    let level = if count > 0 { sum / count as f32 } else { 0.0 };
    nint(NOISE_LEVEL_ZERO_INDEX - NOISE_LEVEL_STEPS * level).clamp(0, MAX_NOISE_LEVEL as i32) as u32
}

#[cfg(test)]
mod tests {
    use super::{RateControl, adjust_global_gain, estimate_bits, quantize};
    use crate::config::{Config, FrameDuration, SampleRate};
    use crate::spectrum::{global_gain, global_gain_offset};
    use crate::test_vectors::{FRAME_1_XF, FRAME_1_XQ};

    /// First frame of appendix C.3.2, 16 kHz at 10 ms and 40 bytes.
    #[test]
    fn rate_control_of_appendix_c_frame() {
        let config = Config::new(SampleRate::Hz16000, FrameDuration::Ms10);
        let nbytes = 40;
        let nbits_spec = 225;
        let xf = FRAME_1_XF.map(|v| v as f32);

        let (index, min_index) = RateControl::new().estimate_gain(&config, &xf, nbytes, nbits_spec);
        assert_eq!((index, min_index), (191, 121));

        let mut xq = [0; 160];
        quantize(
            &xf,
            global_gain(index, global_gain_offset(&config, nbytes)),
            &mut xq,
        );
        assert_eq!(xq, FRAME_1_XQ);

        let estimate = estimate_bits(&config, nbytes, nbits_spec, &xq);
        assert_eq!(estimate.nbits, 231);
        assert_eq!(estimate.lastnz, 108);
        assert_eq!(
            adjust_global_gain(&config, index, min_index, estimate.nbits, nbits_spec),
            Some(192)
        );
    }
}
