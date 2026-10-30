//! Temporal Noise Shaping (sections 3.3.8 and 3.4.6).
//!
//! Up to two lattice filters are applied over frequency to the spectrum, to
//! shape the quantization noise in time.

mod tables;

use crate::bitstream::{BIT_RESOLUTION, Reader, Writer};
use crate::config::{Bandwidth, Config, FrameDuration};
use crate::math::nint;
use tables::{COEF_BITS, COEF_CUMFREQ, COEF_FREQ, ORDER_BITS, ORDER_CUMFREQ, ORDER_FREQ};

/// Maximum number of filters.
const MAX_FILTERS: usize = 2;

/// Maximum order of a filter.
const MAX_ORDER: usize = 8;

/// Number of subdivisions of the frequency range of a filter.
const SUBDIVISIONS: usize = 3;

/// Index of the quantized reflection coefficient 0.
const ZERO_INDEX: i32 = 8;

/// Largest index of the quantized reflection coefficients.
const MAX_INDEX: i32 = 16;

/// Quantization step of the reflection coefficients in the arcsine domain,
/// `pi / 17` (equation 73).
const QUANTIZATION_STEP: f32 = core::f32::consts::PI / 17.0;

/// Autocorrelation used when a subdivision has no energy (equation 66).
const SILENT_AUTOCORRELATION: f32 = 3.0;

/// Factor of the Gaussian lag window (equation 68).
const LAG_WINDOW: f32 = 0.02 * core::f32::consts::PI;

/// Prediction gain above which a filter is enabled (equation 69).
const MIN_PREDICTION_GAIN: f32 = 1.5;

/// Prediction gain below which the LPC weighting applies (equation 70).
const WEIGHTING_PREDICTION_GAIN: f32 = 2.0;

/// Smallest LPC weighting factor (equation 70).
const MIN_WEIGHTING: f32 = 0.85;

/// Frequency layout of the TNS filters of a configuration and bandwidth
/// (tables 3.15 and 3.20).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Layout {
    /// Number of filters.
    filters: usize,
    /// Subdivisions of each filter: the subdivision `s` of the filter `f`
    /// spans the spectral lines `edges[f][s] .. edges[f][s + 1]`, the filter
    /// spanning `edges[f][0] .. edges[f][3]`.
    edges: [[usize; SUBDIVISIONS + 1]; MAX_FILTERS],
}

impl Layout {
    /// Layout of the filters of a frame.
    const fn new(frame_duration: FrameDuration, bw: Bandwidth) -> Self {
        let (filters, edges) = match (frame_duration, bw) {
            (FrameDuration::Ms10, Bandwidth::Narrow) => (1, [[12, 34, 57, 80], [0; 4]]),
            (FrameDuration::Ms10, Bandwidth::Wide) => (1, [[12, 61, 110, 160], [0; 4]]),
            (FrameDuration::Ms10, Bandwidth::SemiSuperWide) => (1, [[12, 88, 164, 240], [0; 4]]),
            (FrameDuration::Ms10, Bandwidth::SuperWide) => {
                (2, [[12, 61, 110, 160], [160, 213, 266, 320]])
            }
            (FrameDuration::Ms10, Bandwidth::Full) => {
                (2, [[12, 74, 137, 200], [200, 266, 333, 400]])
            }
            (FrameDuration::Ms7_5, Bandwidth::Narrow) => (1, [[9, 26, 43, 60], [0; 4]]),
            (FrameDuration::Ms7_5, Bandwidth::Wide) => (1, [[9, 46, 83, 120], [0; 4]]),
            (FrameDuration::Ms7_5, Bandwidth::SemiSuperWide) => (1, [[9, 66, 123, 180], [0; 4]]),
            (FrameDuration::Ms7_5, Bandwidth::SuperWide) => {
                (2, [[9, 46, 82, 120], [120, 159, 200, 240]])
            }
            (FrameDuration::Ms7_5, Bandwidth::Full) => {
                (2, [[9, 56, 103, 150], [150, 200, 250, 300]])
            }
        };
        Self { filters, edges }
    }

    /// Spectral lines filtered by the filter `f`.
    fn range(&self, f: usize) -> core::ops::Range<usize> {
        self.edges[f][0]..self.edges[f][SUBDIVISIONS]
    }
}

/// Whether the LPC weighting applies to frames of `nbytes` bytes (equation 71).
pub(crate) const fn lpc_weighting(config: &Config, nbytes: usize) -> bool {
    let nbits = 8 * nbytes;
    match config.frame_duration {
        FrameDuration::Ms10 => nbits < 480,
        FrameDuration::Ms7_5 => nbits < 360,
    }
}

/// TNS parameters of a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Tns {
    /// Frequency layout of the filters.
    layout: Layout,
    /// Whether the LPC weighting applies, selecting the order tables.
    lpc_weighting: bool,
    /// Order of each filter (`rc_order`), 0 when disabled.
    order: [usize; MAX_FILTERS],
    /// Quantization indices of the reflection coefficients (`rc_i`).
    index: [[u8; MAX_ORDER]; MAX_FILTERS],
}

impl Tns {
    /// TNS parameters with all the filters disabled.
    fn disabled(config: &Config, bw: Bandwidth, nbytes: usize) -> Self {
        Self {
            layout: Layout::new(config.frame_duration, bw),
            lpc_weighting: lpc_weighting(config, nbytes),
            order: [0; MAX_FILTERS],
            index: [[ZERO_INDEX as u8; MAX_ORDER]; MAX_FILTERS],
        }
    }

    /// Quantized reflection coefficients of the filter `f` (equations 74 and 122).
    fn coefficients(&self, f: usize) -> [f32; MAX_ORDER] {
        let mut rc = [0.0; MAX_ORDER];
        for (k, c) in rc.iter_mut().enumerate().take(self.order[f]) {
            *c = libm::sinf(QUANTIZATION_STEP * (i32::from(self.index[f][k]) - ZERO_INDEX) as f32);
        }
        rc
    }

    /// Number of bits of the TNS data (equations 75 to 77).
    pub(crate) fn nbits(&self) -> u32 {
        let weighting = usize::from(self.lpc_weighting);
        (0..self.layout.filters)
            .map(|f| {
                let order = self.order[f];
                let mut bits = BIT_RESOLUTION;
                if order > 0 {
                    bits += u32::from(ORDER_BITS[weighting][order - 1]);
                    for k in 0..order {
                        bits += u32::from(COEF_BITS[k][usize::from(self.index[f][k])]);
                    }
                }
                bits.div_ceil(BIT_RESOLUTION)
            })
            .sum()
    }

    /// Writes the activation flags of the filters to the side information.
    pub(crate) fn write_flags(&self, w: &mut Writer) {
        for &order in &self.order[..self.layout.filters] {
            w.write_bit(u32::from(order > 0));
        }
    }

    /// Arithmetic encoding of the orders and coefficients of the filters
    /// (section 3.3.13.4.2).
    pub(crate) fn write_coefficients(&self, w: &mut Writer) {
        let weighting = usize::from(self.lpc_weighting);
        for f in 0..self.layout.filters {
            let order = self.order[f];
            if order == 0 {
                continue;
            }
            w.ac_encode(
                ORDER_CUMFREQ[weighting][order - 1],
                ORDER_FREQ[weighting][order - 1],
            );
            for k in 0..order {
                let i = usize::from(self.index[f][k]);
                w.ac_encode(COEF_CUMFREQ[k][i], COEF_FREQ[k][i]);
            }
        }
    }

    /// Reads the activation flags of the filters from the side information.
    pub(crate) fn read_flags(
        config: &Config,
        bw: Bandwidth,
        nbytes: usize,
        r: &mut Reader,
    ) -> Self {
        let mut tns = Self::disabled(config, bw, nbytes);
        for f in 0..tns.layout.filters {
            tns.order[f] = r.read_bit() as usize;
        }
        tns
    }

    /// Arithmetic decoding of the orders and coefficients of the enabled
    /// filters (section 3.4.2.5), `None` on a bit error.
    pub(crate) fn read_coefficients(&mut self, r: &mut Reader) -> Option<()> {
        let weighting = usize::from(self.lpc_weighting);
        for f in 0..self.layout.filters {
            if self.order[f] == 0 {
                continue;
            }
            self.order[f] = r.ac_decode(&ORDER_CUMFREQ[weighting], &ORDER_FREQ[weighting])? + 1;
            for k in 0..self.order[f] {
                self.index[f][k] = r.ac_decode(&COEF_CUMFREQ[k], &COEF_FREQ[k])? as u8;
            }
        }
        Some(())
    }

    /// Analysis filtering of the spectrum `x`, in place (section 3.3.8.4).
    pub(crate) fn filter_analysis(&self, x: &mut [f32]) {
        let mut st = [0.0f32; MAX_ORDER];
        for f in 0..self.layout.filters {
            let order = self.order[f];
            if order == 0 {
                continue;
            }
            let rc = self.coefficients(f);
            for v in &mut x[self.layout.range(f)] {
                let mut t = *v;
                let mut st_save = t;
                for k in 0..order - 1 {
                    let st_tmp = rc[k] * t + st[k];
                    t += rc[k] * st[k];
                    st[k] = st_save;
                    st_save = st_tmp;
                }
                t += rc[order - 1] * st[order - 1];
                st[order - 1] = st_save;
                *v = t;
            }
        }
    }

    /// Synthesis filtering of the spectrum `x`, in place (section 3.4.6).
    pub(crate) fn filter_synthesis(&self, x: &mut [f32]) {
        let mut s = [0.0f32; MAX_ORDER];
        for f in 0..self.layout.filters {
            let order = self.order[f];
            if order == 0 {
                continue;
            }
            let rc = self.coefficients(f);
            for v in &mut x[self.layout.range(f)] {
                let mut t = *v - rc[order - 1] * s[order - 1];
                for k in (0..order - 1).rev() {
                    t -= rc[k] * s[k];
                    s[k + 1] = rc[k] * t + s[k];
                }
                *v = t;
                s[0] = t;
            }
        }
    }
}

/// Normalized autocorrelation of the spectrum over the subdivisions of a
/// filter (equations 65 to 67).
fn autocorrelation(x: &[f32], edges: &[usize; SUBDIVISIONS + 1]) -> [f32; MAX_ORDER + 1] {
    let mut r = [0.0f32; MAX_ORDER + 1];
    let mut energy = [0.0f32; SUBDIVISIONS];
    for (s, e) in energy.iter_mut().enumerate() {
        *e = x[edges[s]..edges[s + 1]].iter().map(|v| v * v).sum();
    }
    if energy.contains(&0.0) {
        r[0] = SILENT_AUTOCORRELATION;
        return r;
    }
    for (k, rk) in r.iter_mut().enumerate() {
        let mut sum = 0.0f32;
        for s in 0..SUBDIVISIONS {
            let mut c = 0.0f32;
            for n in edges[s]..edges[s + 1] - k {
                c += x[n] * x[n + k];
            }
            sum += c / energy[s];
        }
        *rk = sum;
    }
    r
}

/// Levinson-Durbin recursion: the LPC coefficients and the prediction error
/// of an autocorrelation (section 3.3.8.2).
fn levinson_durbin(r: &[f32; MAX_ORDER + 1]) -> ([f32; MAX_ORDER + 1], f32) {
    let mut a = [0.0f32; MAX_ORDER + 1];
    let mut err = r[0];
    a[0] = 1.0;
    for k in 1..=MAX_ORDER {
        let mut acc = 0.0f32;
        for n in 0..k {
            acc += a[n] * r[k - n];
        }
        let rc = -acc / err;
        let mut tmp = [0.0f32; MAX_ORDER + 1];
        tmp[0] = 1.0;
        for n in 1..k {
            tmp[n] = a[n] + rc * a[k - n];
        }
        tmp[k] = rc;
        a[..=k].copy_from_slice(&tmp[..=k]);
        err *= 1.0 - rc * rc;
    }
    (a, err)
}

/// Reflection coefficients of LPC coefficients (section 3.3.8.2).
fn reflection_coefficients(a: &[f32; MAX_ORDER + 1]) -> [f32; MAX_ORDER] {
    let mut rc = [0.0f32; MAX_ORDER];
    let mut tmp1 = *a;
    for k in (1..=MAX_ORDER).rev() {
        rc[k - 1] = tmp1[k];
        let e = 1.0 - rc[k - 1] * rc[k - 1];
        let mut tmp2 = [0.0f32; MAX_ORDER + 1];
        for n in 1..k {
            tmp2[n] = (tmp1[n] - rc[k - 1] * tmp1[k - n]) / e;
        }
        tmp1[1..k].copy_from_slice(&tmp2[1..k]);
    }
    rc
}

/// TNS analysis and quantization of the shaped spectrum `x` (sections
/// 3.3.8.2 and 3.3.8.3).
pub(crate) fn analyze(
    config: &Config,
    bw: Bandwidth,
    near_nyquist: bool,
    nbytes: usize,
    x: &[f32],
) -> Tns {
    let mut tns = Tns::disabled(config, bw, nbytes);
    for f in 0..tns.layout.filters {
        let mut r = autocorrelation(x, &tns.layout.edges[f]);
        for (k, rk) in r.iter_mut().enumerate() {
            let lag = LAG_WINDOW * k as f32;
            *rk *= libm::expf(-0.5 * lag * lag);
        }
        let (a, err) = levinson_durbin(&r);

        let prediction_gain = r[0] / err;
        let enabled = prediction_gain > MIN_PREDICTION_GAIN && !near_nyquist;
        if !enabled {
            continue;
        }

        // LPC weighting (equations 70 and 72)
        let gamma = if tns.lpc_weighting && prediction_gain < WEIGHTING_PREDICTION_GAIN {
            1.0 - (1.0 - MIN_WEIGHTING) * (WEIGHTING_PREDICTION_GAIN - prediction_gain)
                / (WEIGHTING_PREDICTION_GAIN - MIN_PREDICTION_GAIN)
        } else {
            1.0
        };
        let mut aw = [0.0f32; MAX_ORDER + 1];
        let mut g = 1.0f32;
        for (w, &ak) in aw.iter_mut().zip(&a) {
            *w = g * ak;
            g *= gamma;
        }

        // Quantization (equation 73) and order of the filter
        let rc = reflection_coefficients(&aw);
        for (k, &coefficient) in rc.iter().enumerate() {
            let index =
                nint(libm::asinf(coefficient.clamp(-1.0, 1.0)) / QUANTIZATION_STEP) + ZERO_INDEX;
            let index = index.clamp(0, MAX_INDEX);
            tns.index[f][k] = index as u8;
            if index != ZERO_INDEX {
                tns.order[f] = k + 1;
            }
        }
    }
    tns
}

#[cfg(test)]
mod tests {
    use super::{Tns, analyze};
    use crate::config::{Bandwidth, Config, FrameDuration, SampleRate};
    use crate::test_vectors::{FRAME_1_XF, FRAME_1_XS, TNS_DECODER_INPUT, TNS_DECODER_OUTPUT};

    /// Asserts that two vectors are equal within `tolerance` of their largest magnitude.
    fn assert_close(got: &[f32], want: &[f64], tolerance: f64) {
        let scale = want.iter().fold(1.0f64, |m, w| m.max(w.abs()));
        for (i, (&g, &w)) in got.iter().zip(want).enumerate() {
            assert!(
                (f64::from(g) - w).abs() <= tolerance * scale,
                "[{i}]: got {g}, want {w}"
            );
        }
    }

    /// First frame of appendix C.3.2, 16 kHz at 10 ms and 40 bytes.
    #[test]
    fn analysis_of_appendix_c_frame() {
        let config = Config::new(SampleRate::Hz16000, FrameDuration::Ms10);
        let mut x = FRAME_1_XS.map(|v| v as f32);
        let tns = analyze(&config, Bandwidth::Wide, false, 40, &x);
        assert_eq!(tns.order, [6, 0]);
        assert_eq!(tns.index[0], [13, 9, 4, 9, 8, 9, 8, 8]);
        assert_eq!(tns.nbits(), 24);

        tns.filter_analysis(&mut x);
        assert_close(&x, &FRAME_1_XF, 1e-5);
    }

    /// Synthesis with two filters of appendix C.4.4, 48 kHz at 10 ms.
    #[test]
    fn synthesis_of_appendix_c_spectrum() {
        let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
        let mut tns = Tns::disabled(&config, Bandwidth::Full, 100);
        tns.order = [4, 5];
        tns.index = [[3, 10, 10, 9, 8, 8, 8, 8], [4, 9, 11, 9, 9, 8, 8, 8]];

        let mut x = TNS_DECODER_INPUT.map(|v| v as f32);
        tns.filter_synthesis(&mut x);
        assert_close(&x, &TNS_DECODER_OUTPUT, 1e-5);
    }
}
