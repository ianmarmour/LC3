//! Generates the twiddle factor tables of the FFT and of the MDCT.
//!
//! The tables are computed in double precision on the build host and rounded
//! to `f32`, so that the library does not evaluate trigonometric functions at
//! run time.

use std::env;
use std::f64::consts::PI;
use std::fmt::Write;
use std::fs;
use std::path::Path;

/// Size of the table of complex roots of unity of the FFT, a common multiple
/// of all the FFT sizes (30, 40, 60, 80, 90, 120, 160, 180 and 240).
const FFT_TWIDDLES: usize = 1440;

/// Sizes of the MDCT (number of spectral coefficients) of all the configurations.
const MDCT_SIZES: [usize; 9] = [60, 80, 120, 160, 180, 240, 320, 360, 480];

/// Appends a table of complex numbers `(re, im)` named `name` to `out`.
fn write_table(
    out: &mut String,
    doc: &str,
    name: &str,
    values: impl ExactSizeIterator<Item = (f64, f64)>,
) {
    writeln!(out, "/// {doc}").unwrap();
    writeln!(out, "static {name}: [Complex; {}] = [", values.len()).unwrap();
    for (re, im) in values {
        writeln!(out, "    Complex::new({:e}, {:e}),", re as f32, im as f32).unwrap();
    }
    out.push_str("];\n\n");
}

/// Table of the FFT roots of unity, `exp(-2 i pi k / FFT_TWIDDLES)`.
fn fft_twiddles() -> String {
    let mut out = String::new();
    writeln!(out, "/// Size of the table of roots of unity `FFT_TW`.").unwrap();
    writeln!(out, "const FFT_TWIDDLES: usize = {FFT_TWIDDLES};\n").unwrap();
    let roots = (0..FFT_TWIDDLES).map(|k| {
        let a = -2.0 * PI * k as f64 / FFT_TWIDDLES as f64;
        (a.cos(), a.sin())
    });
    write_table(
        &mut out,
        "Roots of unity `exp(-2 i pi k / FFT_TWIDDLES)`.",
        "FFT_TW",
        roots,
    );
    out
}

/// Pre-twiddles `exp(-i pi (n + 1/4) / N)` and post-twiddles
/// `sqrt(2 / N) exp(-i pi k / N)` of the DCT-IV of each size `N`.
fn mdct_twiddles() -> String {
    let mut out = String::new();
    for n in MDCT_SIZES {
        let pre = (0..n / 2).map(|k| {
            let a = -PI * (k as f64 + 0.25) / n as f64;
            (a.cos(), a.sin())
        });
        write_table(
            &mut out,
            &format!("Pre-twiddles of the DCT-IV of size {n}."),
            &format!("PRE_{n}"),
            pre,
        );
        let scale = (2.0 / n as f64).sqrt();
        let post = (0..n / 2).map(|k| {
            let a = -PI * k as f64 / n as f64;
            (scale * a.cos(), scale * a.sin())
        });
        write_table(
            &mut out,
            &format!("Post-twiddles of the DCT-IV of size {n}."),
            &format!("POST_{n}"),
            post,
        );
    }
    out
}

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();
    fs::write(Path::new(&out_dir).join("fft_twiddles.rs"), fft_twiddles()).unwrap();
    fs::write(
        Path::new(&out_dir).join("mdct_twiddles.rs"),
        mdct_twiddles(),
    )
    .unwrap();
    println!("cargo:rerun-if-changed=build.rs");
}
