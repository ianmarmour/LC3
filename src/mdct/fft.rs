//! Complex FFT of the sizes used by the MDCT.
//!
//! The transform is a self-sorting (Stockham) mixed-radix FFT supporting the
//! radices 2, 3, 4 and 5, which covers the half frame sizes of all the
//! configurations (30 to 240 points).

use core::ops::{Add, Mul, Sub};

/// A complex number.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Complex {
    /// Real part.
    pub(crate) re: f32,
    /// Imaginary part.
    pub(crate) im: f32,
}

impl Complex {
    /// Zero.
    pub(crate) const ZERO: Self = Self::new(0.0, 0.0);

    /// Complex number of real part `re` and imaginary part `im`.
    pub(crate) const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    /// Product by `-i`.
    fn mul_neg_i(self) -> Self {
        Self::new(self.im, -self.re)
    }

    /// Product by the real `s`.
    fn scale(self, s: f32) -> Self {
        Self::new(self.re * s, self.im * s)
    }
}

impl Add for Complex {
    type Output = Self;

    fn add(self, o: Self) -> Self {
        Self::new(self.re + o.re, self.im + o.im)
    }
}

impl Sub for Complex {
    type Output = Self;

    fn sub(self, o: Self) -> Self {
        Self::new(self.re - o.re, self.im - o.im)
    }
}

impl Mul for Complex {
    type Output = Self;

    fn mul(self, o: Self) -> Self {
        Self::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
}

include!(concat!(env!("OUT_DIR"), "/fft_twiddles.rs"));

/// Largest radix of the FFT stages.
const MAX_RADIX: usize = 5;

/// `sin(2 pi / 3)`.
const SIN_2PI_3: f32 = 0.866_025_4;
/// `cos(2 pi / 5)`.
const COS_2PI_5: f32 = 0.309_017;
/// `cos(4 pi / 5)`.
const COS_4PI_5: f32 = -0.809_017;
/// `sin(2 pi / 5)`.
const SIN_2PI_5: f32 = 0.951_056_5;
/// `sin(4 pi / 5)`.
const SIN_4PI_5: f32 = 0.587_785_25;

/// DFT of the first `radix` values of `v`, in place.
fn butterfly(v: &mut [Complex; MAX_RADIX], radix: usize) {
    match radix {
        2 => {
            let (a, b) = (v[0], v[1]);
            v[0] = a + b;
            v[1] = a - b;
        }
        3 => {
            let s = v[1] + v[2];
            let d = (v[1] - v[2]).scale(SIN_2PI_3).mul_neg_i();
            let t = v[0] - s.scale(0.5);
            v[0] = v[0] + s;
            v[1] = t + d;
            v[2] = t - d;
        }
        4 => {
            let t0 = v[0] + v[2];
            let t1 = v[0] - v[2];
            let t2 = v[1] + v[3];
            let t3 = (v[1] - v[3]).mul_neg_i();
            v[0] = t0 + t2;
            v[1] = t1 + t3;
            v[2] = t0 - t2;
            v[3] = t1 - t3;
        }
        5 => {
            let s14 = v[1] + v[4];
            let d14 = v[1] - v[4];
            let s23 = v[2] + v[3];
            let d23 = v[2] - v[3];
            let r1 = v[0] + s14.scale(COS_2PI_5) + s23.scale(COS_4PI_5);
            let r2 = v[0] + s14.scale(COS_4PI_5) + s23.scale(COS_2PI_5);
            let i1 = (d14.scale(SIN_2PI_5) + d23.scale(SIN_4PI_5)).mul_neg_i();
            let i2 = (d14.scale(SIN_4PI_5) - d23.scale(SIN_2PI_5)).mul_neg_i();
            v[0] = v[0] + s14 + s23;
            v[1] = r1 + i1;
            v[4] = r1 - i1;
            v[2] = r2 + i2;
            v[3] = r2 - i2;
        }
        _ => unreachable!("unsupported FFT radix {radix}"),
    }
}

/// Radix of the next stage of an FFT with `remaining` points left to combine.
fn next_radix(remaining: usize) -> usize {
    [4, 2, 3, 5]
        .into_iter()
        .find(|radix| remaining % radix == 0)
        .expect("FFT size with prime factors other than 2, 3 and 5")
}

/// Stage of radix `radix` of the FFT of `src` into `dst`, after `combined`
/// points have been combined by the previous stages.
fn stage(src: &[Complex], dst: &mut [Complex], radix: usize, combined: usize) {
    let stride = src.len() / radix;
    let twiddle_step = FFT_TWIDDLES / (combined * radix);
    for j in 0..stride {
        let k = j % combined;
        let mut v = [Complex::ZERO; MAX_RADIX];
        v[0] = src[j];
        for r in 1..radix {
            v[r] = src[j + r * stride] * FFT_TW[r * k * twiddle_step];
        }
        butterfly(&mut v, radix);
        let base = (j / combined) * combined * radix + k;
        for (r, &vr) in v.iter().enumerate().take(radix) {
            dst[base + r * combined] = vr;
        }
    }
}

/// Forward FFT of `x` in place, `scratch` being a buffer of the same size.
pub(crate) fn fft(x: &mut [Complex], scratch: &mut [Complex]) {
    debug_assert!(FFT_TWIDDLES % x.len() == 0);
    let mut combined = 1;
    let mut remaining = x.len();
    let mut result_in_x = true;
    while remaining > 1 {
        let radix = next_radix(remaining);
        if result_in_x {
            stage(x, scratch, radix, combined);
        } else {
            stage(scratch, x, radix, combined);
        }
        result_in_x = !result_in_x;
        combined *= radix;
        remaining /= radix;
    }
    if !result_in_x {
        x.copy_from_slice(scratch);
    }
}

#[cfg(test)]
mod tests {
    use super::{Complex, fft};

    #[test]
    fn matches_the_dft() {
        for size in [30, 40, 60, 80, 90, 120, 160, 180, 240] {
            let input: Vec<Complex> = (0..size)
                .map(|n| {
                    Complex::new(
                        ((n * 7 + 3) % 11) as f32 - 5.0,
                        ((n * 5 + 1) % 13) as f32 - 6.0,
                    )
                })
                .collect();
            let mut x = input.clone();
            let mut scratch = vec![Complex::ZERO; size];
            fft(&mut x, &mut scratch);

            for (k, xk) in x.iter().enumerate() {
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for (n, v) in input.iter().enumerate() {
                    let a = -2.0 * core::f64::consts::PI * (n * k) as f64 / size as f64;
                    re += f64::from(v.re) * a.cos() - f64::from(v.im) * a.sin();
                    im += f64::from(v.re) * a.sin() + f64::from(v.im) * a.cos();
                }
                assert!((f64::from(xk.re) - re).abs() < 1e-3, "size {size}, bin {k}");
                assert!((f64::from(xk.im) - im).abs() < 1e-3, "size {size}, bin {k}");
            }
        }
    }
}
