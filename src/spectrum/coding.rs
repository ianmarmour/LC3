//! Arithmetic coding of the quantized spectrum (sections 3.3.13.4 and
//! 3.4.2.5), residual data (3.3.13.5 and 3.4.2.6), residual decoding (3.4.3)
//! and noise filling (3.4.4).

use super::tables::{CONTEXT_LOOKUP, CUMFREQ, FREQ};
use super::{
    LsbMode, NOISE_LEVEL_STEPS, NOISE_LEVEL_ZERO_INDEX, is_noise_line, next_context,
    noise_filling_range, rate_context_offset,
};
use crate::bitstream::{Reader, Writer};
use crate::config::{Bandwidth, Config, MAX_SPECTRAL_LINES};

/// Escape symbol, coding an additional bit plane.
const ESCAPE_SYMBOL: usize = 16;

/// Largest number of escaped bit planes of a 2-tuple.
const MAX_LEVELS: usize = 14;

/// Number of contexts of each level of the context lookup table.
const CONTEXTS_PER_LEVEL: usize = 1024;

/// Context offset of the 2-tuples of the upper half of the spectrum.
const UPPER_HALF_CONTEXT_OFFSET: usize = 256;

/// Adjustment of a refined value when the residual bit is 0, depending on
/// the sign of the value (section 3.4.3).
const RESIDUAL_DOWN: [f32; 2] = [-0.1875, -0.3125];

/// Adjustment of a refined value when the residual bit is 1, depending on
/// the sign of the value (section 3.4.3).
const RESIDUAL_UP: [f32; 2] = [0.3125, 0.1875];

/// Multiplier of the noise filling random generator.
const NOISE_SEED_MULTIPLIER: u32 = 31_821;

/// Increment of the noise filling random generator.
const NOISE_SEED_INCREMENT: u32 = 13_849;

/// Mask of the 16 bits seeds of the random generators.
const SEED_MASK: u32 = 0xffff;

/// Seeds below this value give a positive noise value.
const NOISE_SIGN_THRESHOLD: u32 = 0x8000;

/// Context of the 2-tuple of the spectral line `k`.
fn context(c: usize, rate_offset: usize, k: usize, lines: usize) -> usize {
    let mut t = c + rate_offset;
    if k > lines / 2 {
        t += UPPER_HALF_CONTEXT_OFFSET;
    }
    t
}

/// Probability model of a context at a level.
fn model(t: usize, level: usize) -> usize {
    usize::from(CONTEXT_LOOKUP[t + level.min(3) * CONTEXTS_PER_LEVEL])
}

/// Least significant bits written to the residual data in the
/// [`LsbMode::Truncation`] mode.
#[derive(Clone)]
pub(crate) struct LsbBits {
    /// Bits, one per byte.
    bits: [u8; 2 * MAX_SPECTRAL_LINES],
    /// Number of bits.
    len: usize,
}

impl LsbBits {
    /// Empty set of bits.
    pub(crate) const fn new() -> Self {
        Self {
            bits: [0; 2 * MAX_SPECTRAL_LINES],
            len: 0,
        }
    }

    /// Appends a bit.
    fn push(&mut self, bit: u32) {
        if let Some(b) = self.bits.get_mut(self.len) {
            *b = bit as u8;
            self.len += 1;
        }
    }

    /// The bits.
    pub(crate) fn as_slice(&self) -> &[u8] {
        &self.bits[..self.len]
    }
}

/// Arithmetic encoding of the quantized spectrum `xq` up to the spectral
/// line `lastnz` (section 3.3.13.4.2). The least significant bits are
/// collected in `lsbs` in the [`LsbMode::Truncation`] mode.
pub(crate) fn encode(
    config: &Config,
    nbytes: usize,
    w: &mut Writer,
    xq: &[i32],
    lastnz: usize,
    lsb_mode: LsbMode,
    lsbs: &mut LsbBits,
) {
    let rate_offset = rate_context_offset(config, nbytes);
    let truncation = lsb_mode == LsbMode::Truncation;
    lsbs.len = 0;
    let mut c = 0;
    for k in (0..lastnz).step_by(2) {
        let t = context(c, rate_offset, k, xq.len());
        let mut a = xq[k].unsigned_abs();
        let mut b = xq[k + 1].unsigned_abs();
        let mut level = 0;
        let (mut lsb0, mut lsb1) = (0, 0);
        while a.max(b) >= 4 {
            let m = model(t, level);
            w.ac_encode(CUMFREQ[m][ESCAPE_SYMBOL], FREQ[m][ESCAPE_SYMBOL]);
            if truncation && level == 0 {
                lsb0 = a & 1;
                lsb1 = b & 1;
            } else {
                w.write_bit(a & 1);
                w.write_bit(b & 1);
            }
            a >>= 1;
            b >>= 1;
            level += 1;
        }
        let m = model(t, level);
        let symbol = (a + 4 * b) as usize;
        w.ac_encode(CUMFREQ[m][symbol], FREQ[m][symbol]);

        // Signs, coded after the least significant bits in the truncation mode
        let mut a_lsb = xq[k].unsigned_abs();
        let mut b_lsb = xq[k + 1].unsigned_abs();
        if truncation && level > 0 {
            a_lsb >>= 1;
            b_lsb >>= 1;
            lsbs.push(lsb0);
            if a_lsb == 0 && xq[k] != 0 {
                lsbs.push(u32::from(xq[k] < 0));
            }
            lsbs.push(lsb1);
            if b_lsb == 0 && xq[k + 1] != 0 {
                lsbs.push(u32::from(xq[k + 1] < 0));
            }
        }
        if a_lsb > 0 {
            w.write_bit(u32::from(xq[k] < 0));
        }
        if b_lsb > 0 {
            w.write_bit(u32::from(xq[k + 1] < 0));
        }
        c = next_context(c, a, b, level.min(3));
    }
}

/// Arithmetic decoding of the quantized spectrum up to the spectral line
/// `lastnz` (section 3.4.2.5), `None` on a bit error. In the
/// [`LsbMode::Truncation`] mode, the levels of the 2-tuples are saved in
/// `levels` for the decoding of the residual data.
pub(crate) fn decode(
    config: &Config,
    nbytes: usize,
    r: &mut Reader,
    lastnz: usize,
    lsb_mode: LsbMode,
    xq: &mut [i32],
    levels: &mut [u8],
) -> Option<()> {
    let rate_offset = rate_context_offset(config, nbytes);
    let truncation = lsb_mode == LsbMode::Truncation;
    let mut c = 0;
    for k in (0..lastnz).step_by(2) {
        let t = context(c, rate_offset, k, xq.len());
        let (mut a, mut b) = (0i32, 0i32);
        let mut level = 0;
        let symbol = loop {
            if level == MAX_LEVELS {
                return None;
            }
            let m = model(t, level);
            let symbol = r.ac_decode(&CUMFREQ[m], &FREQ[m])?;
            if symbol < ESCAPE_SYMBOL {
                break symbol;
            }
            if !truncation || level > 0 {
                a += (r.read_bit() as i32) << level;
                b += (r.read_bit() as i32) << level;
            }
            level += 1;
        };
        if truncation {
            levels[k] = level as u8;
        }
        let msb_a = (symbol & 3) as u32;
        let msb_b = (symbol >> 2) as u32;
        a += (msb_a as i32) << level;
        b += (msb_b as i32) << level;
        if a > 0 && r.read_bit() == 1 {
            a = -a;
        }
        if b > 0 && r.read_bit() == 1 {
            b = -b;
        }
        xq[k] = a;
        xq[k + 1] = b;
        c = next_context(c, msb_a, msb_b, level.min(3));
        if r.overrun() {
            return None;
        }
    }
    xq[lastnz..].fill(0);
    Some(())
}

/// Refines a spectral value with a residual bit of the
/// [`LsbMode::Truncation`] mode, `false` when no residual bit is left.
fn refine_lsb(r: &mut Reader, nbits: &mut i32, x: &mut i32) -> bool {
    if *nbits == 0 {
        return false;
    }
    let bit = r.read_bit();
    *nbits -= 1;
    if bit == 1 {
        if *x > 0 {
            *x += 1;
        } else if *x < 0 {
            *x -= 1;
        } else {
            if *nbits == 0 {
                return false;
            }
            let sign = r.read_bit();
            *nbits -= 1;
            *x = if sign == 0 { 1 } else { -1 };
        }
    }
    true
}

/// Decoding of the residual data of the [`LsbMode::Truncation`] mode
/// (section 3.4.2.6).
pub(crate) fn decode_lsb_residual(
    r: &mut Reader,
    mut nbits: i32,
    lastnz: usize,
    levels: &[u8],
    xq: &mut [i32],
) {
    for k in (0..lastnz).step_by(2) {
        if levels[k] == 0 {
            continue;
        }
        if !refine_lsb(r, &mut nbits, &mut xq[k]) || !refine_lsb(r, &mut nbits, &mut xq[k + 1]) {
            break;
        }
    }
}

/// Reading of the residual bits of the [`LsbMode::Refinement`] mode
/// (section 3.4.2.6), returns the number of bits read to `res_bits`.
pub(crate) fn read_residual(r: &mut Reader, nbits: i32, xq: &[i32], res_bits: &mut [u8]) -> usize {
    let mut count = 0;
    for _ in xq.iter().filter(|&&q| q != 0) {
        if count as i32 == nbits {
            break;
        }
        res_bits[count] = r.read_bit() as u8;
        count += 1;
    }
    count
}

/// Refinement of the non-zero values of the spectrum `x` with the residual
/// bits (section 3.4.3).
pub(crate) fn apply_residual(x: &mut [f32], res_bits: &[u8]) {
    let mut bits = res_bits.iter();
    for v in x.iter_mut().filter(|v| **v != 0.0) {
        let Some(&bit) = bits.next() else {
            break;
        };
        let negative = usize::from(*v <= 0.0);
        *v += if bit == 0 {
            RESIDUAL_DOWN[negative]
        } else {
            RESIDUAL_UP[negative]
        };
    }
}

/// Seed of the noise filling (section 3.4.2.6).
pub(crate) fn noise_seed(xq: &[i32]) -> u32 {
    let sum = xq.iter().enumerate().fold(0u32, |sum, (k, &v)| {
        sum.wrapping_add(v.unsigned_abs().wrapping_mul(k as u32))
    });
    sum & SEED_MASK
}

/// Noise filling of the spectrum `x` (section 3.4.4), from the decoded
/// quantized spectrum `xq`, the noise level `F_NF` and the seed `seed`.
pub(crate) fn noise_fill(
    config: &Config,
    bw: Bandwidth,
    noise_level: u32,
    mut seed: u32,
    xq: &[i32],
    x: &mut [f32],
) {
    let (start, width) = noise_filling_range(config.frame_duration);
    let stop = bw.stop_line(config.frame_duration);
    let level = (NOISE_LEVEL_ZERO_INDEX - noise_level as f32) / NOISE_LEVEL_STEPS;
    for (k, v) in x.iter_mut().enumerate().take(stop).skip(start) {
        if is_noise_line(xq, k, width, stop) {
            seed = (NOISE_SEED_INCREMENT + seed * NOISE_SEED_MULTIPLIER) & SEED_MASK;
            *v = if seed < NOISE_SIGN_THRESHOLD {
                level
            } else {
                -level
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{LsbBits, apply_residual, decode, encode};
    use crate::bitstream::{Reader, Writer};
    use crate::config::{Config, FrameDuration, SampleRate};
    use crate::spectrum::LsbMode;

    #[test]
    fn spectrum_round_trip() {
        let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
        let mut xq = [0i32; 400];
        for (k, v) in xq.iter_mut().enumerate().take(300) {
            *v = ((k * 7919) % 41) as i32 - 20;
        }
        xq[3] = 1000;
        for lsb_mode in [LsbMode::Refinement, LsbMode::Truncation] {
            let mut frame = [0u8; 400];
            let mut lsbs = LsbBits::new();
            let mut w = Writer::new(&mut frame);
            encode(&config, 400, &mut w, &xq, 300, lsb_mode, &mut lsbs);
            w.finish();

            let mut decoded = [0i32; 400];
            let mut levels = [0u8; 400];
            let mut r = Reader::new(&frame);
            r.ac_start();
            decode(
                &config, 400, &mut r, 300, lsb_mode, &mut decoded, &mut levels,
            )
            .unwrap();
            if lsb_mode == LsbMode::Refinement {
                assert_eq!(decoded, xq);
            } else {
                // The least significant bit of the first bit plane of the
                // escaped values is coded in the residual data.
                for (k, (&d, &q)) in decoded.iter().zip(&xq).enumerate() {
                    if levels[k & !1] > 0 {
                        assert!((d - q).abs() <= 1, "line {k}: {d} {q}");
                    } else {
                        assert_eq!(d, q);
                    }
                }
            }
        }
    }

    #[test]
    fn residual_refines_toward_the_original_value() {
        let mut x = [3.0, 0.0, -2.0, 1.0];
        apply_residual(&mut x, &[1, 0, 1]);
        assert_eq!(x, [3.3125, 0.0, -2.3125, 1.3125]);
    }
}
