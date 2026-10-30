//! Low delay MDCT analysis (section 3.3.4) and synthesis (section 3.4.8).
//!
//! Both transforms are evaluated with a DCT-IV, computed through a complex
//! FFT of half its size.

mod fft;
mod windows;

use crate::config::{Config, MAX_FRAME_SAMPLES};
use fft::{Complex, fft};
pub(crate) use windows::window;

include!(concat!(env!("OUT_DIR"), "/mdct_twiddles.rs"));

/// Scratch buffers of the transforms.
#[derive(Clone)]
pub(crate) struct Scratch {
    /// Input and output of the FFT.
    fft: [Complex; MAX_FRAME_SAMPLES / 2],
    /// Scratch buffer of the FFT.
    fft_scratch: [Complex; MAX_FRAME_SAMPLES / 2],
    /// Time aliased signal, input or output of the DCT-IV.
    aliased: [f32; MAX_FRAME_SAMPLES],
}

impl Scratch {
    /// New scratch buffers.
    pub(crate) const fn new() -> Self {
        Self {
            fft: [Complex::ZERO; MAX_FRAME_SAMPLES / 2],
            fft_scratch: [Complex::ZERO; MAX_FRAME_SAMPLES / 2],
            aliased: [0.0; MAX_FRAME_SAMPLES],
        }
    }
}

/// Pre-twiddles and post-twiddles of the DCT-IV of size `n`.
fn twiddles(n: usize) -> (&'static [Complex], &'static [Complex]) {
    match n {
        60 => (&PRE_60, &POST_60),
        80 => (&PRE_80, &POST_80),
        120 => (&PRE_120, &POST_120),
        160 => (&PRE_160, &POST_160),
        180 => (&PRE_180, &POST_180),
        240 => (&PRE_240, &POST_240),
        320 => (&PRE_320, &POST_320),
        360 => (&PRE_360, &POST_360),
        480 => (&PRE_480, &POST_480),
        _ => unreachable!("no MDCT of size {n}"),
    }
}

/// Scaled DCT-IV of `x` into `y`:
/// `y(k) = sqrt(2/N) sum_n x(n) cos(pi/N (n + 1/2) (k + 1/2))`.
fn dct4(x: &[f32], y: &mut [f32], fft_buf: &mut [Complex], fft_scratch: &mut [Complex]) {
    let n = x.len();
    let half = n / 2;
    let (pre, post) = twiddles(n);
    let z = &mut fft_buf[..half];
    for k in 0..half {
        z[k] = Complex::new(x[2 * k], x[n - 1 - 2 * k]) * pre[k];
    }
    fft(z, &mut fft_scratch[..half]);
    for k in 0..half {
        let v = z[k] * post[k];
        y[2 * k] = v.re;
        y[n - 1 - 2 * k] = -v.im;
    }
}

/// Low delay MDCT of the `2 N_F` samples of the time buffer `t` (equation 8),
/// writing the `N_F` spectral coefficients to `x`.
pub(crate) fn forward(config: &Config, t: &[f32], x: &mut [f32], scratch: &mut Scratch) {
    let w = window(config);
    let n = x.len();
    let half = n / 2;
    let Scratch {
        fft,
        fft_scratch,
        aliased,
    } = scratch;
    // Time domain aliasing: with the windowed input made of the quarters
    // (a, b, c, d), the DCT-IV input is (-c_r - d, a - b_r).
    for i in 0..half {
        let c = t[n + half - 1 - i] * w[n + half - 1 - i];
        let d = t[n + half + i] * w[n + half + i];
        aliased[i] = -c - d;
        let a = t[i] * w[i];
        let b = t[n - 1 - i] * w[n - 1 - i];
        aliased[half + i] = a - b;
    }
    dct4(&aliased[..n], x, fft, fft_scratch);
}

/// Low delay MDCT synthesis of the `N_F` spectral coefficients `x`, with
/// windowing and overlap-add (equations 125 to 129). `overlap` holds the
/// `N_F - Z` samples of the overlap-add memory and the `N_F` reconstructed
/// samples are written to `y`.
pub(crate) fn inverse(
    config: &Config,
    x: &[f32],
    overlap: &mut [f32],
    y: &mut [f32],
    scratch: &mut Scratch,
) {
    let w = window(config);
    let z = config.mdct_zeros();
    let n = x.len();
    let half = n / 2;
    let Scratch {
        fft,
        fft_scratch,
        aliased,
    } = scratch;
    dct4(x, &mut aliased[..n], fft, fft_scratch);

    // Time aliased buffer t(k), k = 0 .. 2N-1, from the DCT-IV output u:
    // (u(h + k), -u(n - 1 - k), -u(h - 1 - k), -u(k)) on the four quarters.
    let u = &*aliased;
    let aliased_sample = |k: usize| -> f32 {
        if k < half {
            u[half + k]
        } else if k < n {
            -u[n - 1 - (k - half)]
        } else if k < n + half {
            -u[half - 1 - (k - n)]
        } else {
            -u[k - n - half]
        }
    };
    // Windowing with the time reversed window, and overlap-add.
    let windowed = |k: usize| aliased_sample(k) * w[2 * n - 1 - k];
    for (i, yi) in y.iter_mut().enumerate().take(n) {
        let v = windowed(z + i);
        *yi = if i < n - z { overlap[i] + v } else { v };
    }
    for (i, mem) in overlap.iter_mut().enumerate().take(n - z) {
        *mem = windowed(n + z + i);
    }
}

#[cfg(test)]
mod tests {
    use super::{Scratch, forward, window};
    use crate::config::{Config, FrameDuration, SampleRate};

    /// MDCT of equation 8, computed directly in double precision.
    fn direct_mdct(t: &[f32], w: &[f32], n: usize) -> Vec<f64> {
        let pi = core::f64::consts::PI;
        (0..n)
            .map(|k| {
                let sum: f64 = (0..2 * n)
                    .map(|i| {
                        f64::from(w[i])
                            * f64::from(t[i])
                            * ((pi / n as f64)
                                * (i as f64 + 0.5 + n as f64 / 2.0)
                                * (k as f64 + 0.5))
                                .cos()
                    })
                    .sum();
                sum * (2.0 / n as f64).sqrt()
            })
            .collect()
    }

    #[test]
    fn forward_matches_the_definition() {
        for frame_duration in [FrameDuration::Ms7_5, FrameDuration::Ms10] {
            for hz in [8_000, 16_000, 24_000, 32_000, 48_000] {
                let config = Config::new(SampleRate::from_hz(hz).unwrap(), frame_duration);
                let n = config.frame_samples();
                let t: Vec<f32> = (0..2 * n)
                    .map(|i| (((i * 37 + 11) % 101) as f32 - 50.0) * 10.0)
                    .collect();
                let mut x = vec![0.0; n];
                forward(&config, &t, &mut x, &mut Scratch::new());

                let expected = direct_mdct(&t, window(&config), n);
                for (k, (&got, &want)) in x.iter().zip(&expected).enumerate() {
                    assert!((f64::from(got) - want).abs() < 0.05, "N_F {n}, bin {k}");
                }
            }
        }
    }
}
