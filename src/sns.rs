//! Spectral Noise Shaping (sections 3.3.7 and 3.4.7).
//!
//! The encoder derives 16 scale factors from the band energies, quantizes
//! them with a vector quantizer, and shapes the spectrum with their
//! interpolation over the bands. The decoder applies the inverse shaping.

mod tables;
mod vq;

pub(crate) use vq::{dequantize, quantize, read, write};

use crate::config::{Config, FrameDuration, MAX_BANDS};

/// Number of scale factors.
pub(crate) const SCALE_FACTORS: usize = 16;

/// Number of bits of the quantized scale factors in the side information.
pub(crate) const SIDE_INFO_BITS: u32 = 38;

/// Number of band energies used by the analysis, after padding.
const ENERGIES: usize = 64;

/// Pre-emphasis tilt `g_tilt` per sampling rate index (table 3.7).
const TILT: [f32; 5] = [14.0, 18.0, 22.0, 26.0, 30.0];

/// Denominator of the exponent of the pre-emphasis (equation 21).
const TILT_DENOMINATOR: f32 = 630.0;

/// Noise floor relative to the average band energy, -40 dB (equation 23).
const NOISE_FLOOR_RELATIVE: f32 = 1e-4;

/// Minimum noise floor, `2^-32` (equation 23).
const NOISE_FLOOR_MIN: f32 = 1.0 / 4_294_967_296.0;

/// Offset avoiding the logarithm of zero (equation 24).
const LOG_OFFSET: f32 = 1e-31;

/// Weights of the band energy grouping (equation 26).
const GROUPING_WEIGHTS: [f32; 6] = [
    1.0 / 12.0,
    2.0 / 12.0,
    3.0 / 12.0,
    3.0 / 12.0,
    2.0 / 12.0,
    1.0 / 12.0,
];

/// Scaling of the scale factors after mean removal (equation 27).
const SCALE: f32 = 0.85;

/// Scaling of the smoothed scale factors of attack frames (equation 34).
const fn attack_scale(frame_duration: FrameDuration) -> f32 {
    match frame_duration {
        FrameDuration::Ms10 => 0.5,
        FrameDuration::Ms7_5 => 0.3,
    }
}

/// Direction of the spectral shaping.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Shaping {
    /// Encoder shaping, dividing the spectrum by the scale factors.
    Analysis,
    /// Decoder shaping, multiplying the spectrum by the scale factors.
    Synthesis,
}

/// Band energies `E_B` padded to 64 bands (section 3.3.7.2.1).
fn pad(eb: &[f32]) -> [f32; ENERGIES] {
    let mut padded = [0.0; ENERGIES];
    let nb = eb.len();
    if nb < ENERGIES {
        let n2 = ENERGIES - nb;
        for i in 0..n2 {
            padded[2 * i] = eb[i];
            padded[2 * i + 1] = eb[i];
        }
        padded[2 * n2..].copy_from_slice(&eb[n2..]);
    } else {
        padded.copy_from_slice(&eb[..ENERGIES]);
    }
    padded
}

/// Smoothing of the scale factors of attack frames (equations 29 to 33).
fn smooth(scf0: &[f32; SCALE_FACTORS]) -> [f32; SCALE_FACTORS] {
    let mut scf1 = [0.0; SCALE_FACTORS];
    scf1[0] = (scf0[0] + scf0[1] + scf0[2]) / 3.0;
    scf1[1] = (scf0[0] + scf0[1] + scf0[2] + scf0[3]) / 4.0;
    for n in 2..14 {
        scf1[n] = (scf0[n - 2] + scf0[n - 1] + scf0[n] + scf0[n + 1] + scf0[n + 2]) / 5.0;
    }
    scf1[14] = (scf0[12] + scf0[13] + scf0[14] + scf0[15]) / 4.0;
    scf1[15] = (scf0[13] + scf0[14] + scf0[15]) / 3.0;
    scf1
}

/// SNS analysis: the 16 scale factors of the band energies `eb`
/// (section 3.3.7.2), `attack` being the attack flag `F_att`.
pub(crate) fn analyze(config: &Config, eb: &[f32], attack: bool) -> [f32; SCALE_FACTORS] {
    let eb = pad(eb);

    // Smoothing (equation 20)
    let mut es = [0.0; ENERGIES];
    es[0] = 0.75 * eb[0] + 0.25 * eb[1];
    for b in 1..ENERGIES - 1 {
        es[b] = 0.25 * eb[b - 1] + 0.5 * eb[b] + 0.25 * eb[b + 1];
    }
    es[ENERGIES - 1] = 0.25 * eb[ENERGIES - 2] + 0.75 * eb[ENERGIES - 1];

    // Pre-emphasis (equation 21)
    let tilt = TILT[config.sample_rate.index()];
    let mut ep = [0.0; ENERGIES];
    for (b, e) in ep.iter_mut().enumerate() {
        *e = es[b] * libm::powf(10.0, b as f32 * tilt / TILT_DENOMINATOR);
    }

    // Noise floor (equations 22 and 23) and logarithm (equation 24)
    let sum: f32 = ep.iter().sum();
    let noise_floor = f32::max(
        sum / ENERGIES as f32 * NOISE_FLOOR_RELATIVE,
        NOISE_FLOOR_MIN,
    );
    let el = ep.map(|e| libm::log2f(LOG_OFFSET + f32::max(e, noise_floor)) / 2.0);

    // Band energy grouping (equation 25), the band indices being clamped to 0 .. 63
    let mut e4 = [0.0; SCALE_FACTORS];
    for (b2, e) in e4.iter_mut().enumerate() {
        let mut sum = 0.0;
        for (k, w) in GROUPING_WEIGHTS.iter().enumerate() {
            sum += w * el[(4 * b2 + k).saturating_sub(1).min(ENERGIES - 1)];
        }
        *e = sum;
    }

    // Mean removal and scaling (equation 27), attack handling (equation 34)
    let mean = e4.iter().sum::<f32>() / SCALE_FACTORS as f32;
    let scf0 = e4.map(|e| SCALE * (e - mean));
    if !attack {
        return scf0;
    }
    let scf1 = smooth(&scf0);
    let mean = scf1.iter().sum::<f32>() / SCALE_FACTORS as f32;
    let scale = attack_scale(config.frame_duration);
    scf1.map(|s| scale * (s - mean))
}

/// Scale factors interpolated to the bands and converted to linear gains
/// (sections 3.3.7.4 and 3.4.7.3).
fn gains(config: &Config, scfq: &[f32; SCALE_FACTORS], shaping: Shaping) -> [f32; MAX_BANDS] {
    // Interpolation (equation 63)
    let mut s = [0.0; ENERGIES];
    s[0] = scfq[0];
    s[1] = scfq[0];
    for n in 0..SCALE_FACTORS - 1 {
        let d = scfq[n + 1] - scfq[n];
        s[4 * n + 2] = scfq[n] + d / 8.0;
        s[4 * n + 3] = scfq[n] + 3.0 * d / 8.0;
        s[4 * n + 4] = scfq[n] + 5.0 * d / 8.0;
        s[4 * n + 5] = scfq[n] + 7.0 * d / 8.0;
    }
    let d = scfq[15] - scfq[14];
    s[62] = scfq[15] + d / 8.0;
    s[63] = scfq[15] + 3.0 * d / 8.0;

    // Reduction to the number of bands
    let nb = config.band_count();
    if nb < ENERGIES {
        let n2 = ENERGIES - nb;
        let mut reduced = [0.0; ENERGIES];
        for i in 0..n2 {
            reduced[i] = 0.5 * s[2 * i] + 0.5 * s[2 * i + 1];
        }
        reduced[n2..nb].copy_from_slice(&s[2 * n2..nb + n2]);
        s[..nb].copy_from_slice(&reduced[..nb]);
    }

    // Linear domain (equations 64 and 124)
    let mut g = [0.0; MAX_BANDS];
    for (gb, &sb) in g.iter_mut().zip(&s[..nb]) {
        *gb = libm::exp2f(match shaping {
            Shaping::Analysis => -sb,
            Shaping::Synthesis => sb,
        });
    }
    g
}

/// Spectral shaping of `x` with the quantized scale factors `scfq`
/// (sections 3.3.7.5 and 3.4.7.4).
pub(crate) fn shape(config: &Config, scfq: &[f32; SCALE_FACTORS], shaping: Shaping, x: &mut [f32]) {
    let g = gains(config, scfq, shaping);
    for (b, edges) in config.bands().windows(2).enumerate() {
        for v in &mut x[usize::from(edges[0])..usize::from(edges[1])] {
            *v *= g[b];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Shaping, analyze, gains};
    use crate::config::{Config, FrameDuration, SampleRate};
    use crate::test_vectors::{
        FRAME_1_EB, FRAME_1_SCF, SHAPE_3_EB, SHAPE_3_G_SNS, SHAPE_3_SCF, SHAPE_3_SCFQ,
    };

    /// Configuration of the vectors of appendix C, 16 kHz at 10 ms.
    const CONFIG: Config = Config::new(SampleRate::Hz16000, FrameDuration::Ms10);

    /// Asserts that two vectors are equal within `tolerance`.
    fn assert_close(got: &[f32], want: &[f64], tolerance: f64) {
        for (i, (&g, &w)) in got.iter().zip(want).enumerate() {
            assert!(
                (f64::from(g) - w).abs() <= tolerance,
                "[{i}]: got {g}, want {w}"
            );
        }
    }

    #[test]
    fn analysis_of_appendix_c_energies() {
        let scf = analyze(&CONFIG, &FRAME_1_EB.map(|v| v as f32), false);
        assert_close(&scf, &FRAME_1_SCF, 1e-4);
        let scf = analyze(&CONFIG, &SHAPE_3_EB.map(|v| v as f32), false);
        assert_close(&scf, &SHAPE_3_SCF, 1e-4);
    }

    #[test]
    fn interpolated_gains_of_appendix_c() {
        let g = gains(&CONFIG, &SHAPE_3_SCFQ.map(|v| v as f32), Shaping::Analysis);
        assert_close(&g, &SHAPE_3_G_SNS, 1e-5);
    }
}
