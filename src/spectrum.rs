//! Quantization and coding of the spectrum: global gain, quantization and
//! rate control (section 3.3.10), residual coding (3.3.11), noise level
//! (3.3.12), arithmetic coding (3.3.13.4 and 3.4.2.5), residual decoding
//! (3.4.3) and noise filling (3.4.4).

mod coding;
mod quantization;
mod tables;

pub(crate) use coding::{
    LsbBits, apply_residual, decode, decode_lsb_residual, encode, noise_fill, noise_seed,
    read_residual,
};
pub(crate) use quantization::{
    RateControl, adjust_global_gain, estimate_bits, nbits_ari, noise_level, quantize, residual_bits,
};

use crate::config::{Config, FrameDuration};

/// Number of global gain steps per decade (equations 114 and 120).
const GAIN_STEPS_PER_DECADE: f32 = 28.0;

/// Largest global gain index.
pub(crate) const MAX_GAIN_INDEX: i32 = 255;

/// Number of bits of the global gain index.
pub(crate) const GAIN_INDEX_BITS: u32 = 8;

/// Number of bits of the noise level `F_NF`.
pub(crate) const NOISE_LEVEL_BITS: u32 = 3;

/// Largest noise level `F_NF`, also the level of silent frames.
const MAX_NOISE_LEVEL: u32 = 7;

/// Noise level index `F_NF` of a zero noise level (equations 118 and 119).
const NOISE_LEVEL_ZERO_INDEX: f32 = 8.0;

/// Number of noise level indices per unit of noise level (equations 118 and
/// 119).
const NOISE_LEVEL_STEPS: f32 = 16.0;

/// Coding of the least significant bits of the spectrum (`lsbMode`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum LsbMode {
    /// The remaining bits refine the non-zero quantized values (`lsbMode = 0`).
    Refinement,
    /// The least significant bits of the first bit plane of the large values
    /// are coded with the remaining bits (`lsbMode = 1`).
    Truncation,
}

impl LsbMode {
    /// Mode of the bit of the side information.
    pub(crate) const fn from_bit(bit: u32) -> Self {
        if bit == 0 {
            Self::Refinement
        } else {
            Self::Truncation
        }
    }

    /// Bit of the side information of the mode.
    pub(crate) const fn bit(self) -> u32 {
        match self {
            Self::Refinement => 0,
            Self::Truncation => 1,
        }
    }
}

/// Offset of the global gain index `gg_off` (equations 110 and 121).
pub(crate) fn global_gain_offset(config: &Config, nbytes: usize) -> i32 {
    let nbits = 8 * nbytes as i32;
    let rate = config.sample_rate.index() as i32 + 1;
    -(nbits / (10 * rate)).min(115) - 105 - 5 * rate
}

/// Global gain `gg` of a global gain index and its offset (equations 114 and 120).
pub(crate) fn global_gain(index: i32, offset: i32) -> f32 {
    libm::powf(10.0, (index + offset) as f32 / GAIN_STEPS_PER_DECADE)
}

/// Offset of the arithmetic coder contexts at high bitrates (`rateFlag`).
fn rate_context_offset(config: &Config, nbytes: usize) -> usize {
    if 8 * nbytes > 160 + config.sample_rate.index() * 160 {
        512
    } else {
        0
    }
}

/// Whether the LSB truncation mode is allowed at a bitrate (`modeFlag`).
pub(crate) fn lsb_truncation_allowed(config: &Config, nbytes: usize) -> bool {
    8 * nbytes >= 480 + config.sample_rate.index() * 160
}

/// Whether a frame is a zero frame, coding silence without noise filling
/// (section 3.4.4): a single zero 2-tuple with the smallest global gain and
/// the largest noise level.
pub(crate) fn is_zero_frame(lastnz: usize, xq: &[i32], gain_index: i32, noise_level: u32) -> bool {
    lastnz == 2 && xq[0] == 0 && xq[1] == 0 && gain_index == 0 && noise_level == MAX_NOISE_LEVEL
}

/// Context of the arithmetic coder of the next 2-tuple, from the context `c`
/// and the most significant bits `a` and `b` of the current 2-tuple coded at
/// the level `level`.
fn next_context(c: usize, a: u32, b: u32, level: usize) -> usize {
    let t = if level <= 1 {
        1 + (a + b) as usize * (level + 1)
    } else {
        12 + level
    };
    (c & 15) * 16 + t
}

/// First spectral line and half width of the neighborhood of the noise
/// filling, `NF_start` and `NF_width` (tables 3.17 and 3.19).
const fn noise_filling_range(frame_duration: FrameDuration) -> (usize, usize) {
    match frame_duration {
        FrameDuration::Ms10 => (24, 3),
        FrameDuration::Ms7_5 => (18, 2),
    }
}

/// Whether the spectral line `k` is filled with noise: all the quantized
/// values around it are zero (equations 116 and 119).
fn is_noise_line(xq: &[i32], k: usize, width: usize, stop: usize) -> bool {
    xq[k - width..=(stop - 1).min(k + width)]
        .iter()
        .all(|&v| v == 0)
}

#[cfg(test)]
mod tests {
    use super::{global_gain, global_gain_offset};
    use crate::config::{Config, FrameDuration, SampleRate};

    #[test]
    fn global_gain_of_appendix_c() {
        let config = Config::new(SampleRate::Hz16000, FrameDuration::Ms10);
        let offset = global_gain_offset(&config, 40);
        assert_eq!(offset, -131);
        assert!((global_gain(191, offset) - 138.949_55).abs() < 1e-3);
    }
}
