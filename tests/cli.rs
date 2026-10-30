//! Tests of the `lc3` command.

#![cfg(feature = "cli")]

use std::ffi::OsStr;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitStatus};

/// Sampling rate of the test signal.
const SAMPLE_RATE: u32 = 48_000;

/// Number of samples per channel of the test signal, not a multiple of the
/// frame size.
const SIGNAL_LEN: usize = 10_000;

/// Size of the header of the WAVE files written by the command.
const WAVE_HEADER_SIZE: usize = 44;

/// Directory of the files of a test.
fn test_dir(name: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    fs::create_dir_all(&dir).unwrap();
    dir
}

/// Writes a stereo 16 bits WAVE file of a chirp.
fn write_input(path: &Path) {
    let data_size = (SIGNAL_LEN * 2 * 2) as u32;
    let mut bytes = Vec::new();
    bytes.extend_from_slice(b"RIFF");
    bytes.extend_from_slice(&(36 + data_size).to_le_bytes());
    bytes.extend_from_slice(b"WAVEfmt ");
    for field in [
        16u32,
        1 | (2 << 16),
        SAMPLE_RATE,
        SAMPLE_RATE * 4,
        4 | (16 << 16),
    ] {
        bytes.extend_from_slice(&field.to_le_bytes());
    }
    bytes.extend_from_slice(b"data");
    bytes.extend_from_slice(&data_size.to_le_bytes());
    for n in 0..SIGNAL_LEN {
        let t = n as f64 / f64::from(SAMPLE_RATE);
        let sample = (10_000.0 * (2.0 * std::f64::consts::PI * 2000.0 * t * t).sin()) as i16;
        bytes.extend_from_slice(&sample.to_le_bytes());
        bytes.extend_from_slice(&(-sample).to_le_bytes());
    }
    fs::write(path, bytes).unwrap();
}

/// Runs the command quietly with arguments.
fn lc3(args: &[&dyn AsRef<OsStr>]) -> ExitStatus {
    Command::new(env!("CARGO_BIN_EXE_lc3"))
        .arg("-q")
        .args(args)
        .status()
        .unwrap()
}

#[test]
fn encodes_and_decodes_files() {
    let dir = test_dir("encodes_and_decodes_files");
    let (input, bitstream, output) = (dir.join("in.wav"), dir.join("out.bin"), dir.join("out.wav"));
    write_input(&input);

    assert!(lc3(&[&"-E", &input, &bitstream, &"64000"]).success());
    let header = fs::read(&bitstream).unwrap();
    assert_eq!(header[..2], 0xcc1cu16.to_le_bytes());

    assert!(lc3(&[&"-D", &bitstream, &output]).success());
    let decoded = fs::read(&output).unwrap();
    assert_eq!(decoded.len(), WAVE_HEADER_SIZE + SIGNAL_LEN * 2 * 2);

    // Encoding and decoding at once gives the same output
    let direct = dir.join("direct.wav");
    assert!(lc3(&[&input, &direct, &"64000"]).success());
    assert_eq!(fs::read(&direct).unwrap(), decoded);
}

#[test]
fn conceals_lost_frames() {
    let dir = test_dir("conceals_lost_frames");
    let (input, bitstream) = (dir.join("in.wav"), dir.join("out.bin"));
    let (output, concealed) = (dir.join("out.wav"), dir.join("concealed.wav"));
    let loss_pattern = dir.join("loss.dat");
    write_input(&input);
    fs::write(&loss_pattern, [0, 0, 1, 0]).unwrap();

    assert!(lc3(&[&"-E", &input, &bitstream, &"32000"]).success());
    assert!(lc3(&[&"-D", &bitstream, &output]).success());
    assert!(lc3(&[&"-D", &"-epf", &loss_pattern, &bitstream, &concealed]).success());
    let (output, concealed) = (fs::read(output).unwrap(), fs::read(concealed).unwrap());
    assert_eq!(output.len(), concealed.len());
    assert_ne!(output, concealed);
}

#[test]
fn fails_on_invalid_input() {
    let dir = test_dir("fails_on_invalid_input");
    let input = dir.join("invalid.wav");
    fs::write(&input, b"not a wave file").unwrap();
    assert!(!lc3(&[&input, &dir.join("out.bin"), &"64000"]).success());
}
