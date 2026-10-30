//! `lc3`: command line encoder and decoder of LC3 audio.
//!
//! Encodes WAVE files to LC3 bitstream files, decodes LC3 bitstream files to
//! WAVE files, or both. The command line and the bitstream file format are
//! those of the reference executable of the LC3 conformance software, so
//! that the command runs with the LC3 conformance script.

#![forbid(unsafe_code)]
#![warn(missing_docs, clippy::missing_docs_in_private_items)]

mod cli;

use std::process::ExitCode;

use tracing::{Level, error};

use cli::{Args, ArgsError, USAGE, Verbosity};

/// Logs the messages of the command to the standard error.
fn init_logging(verbosity: Verbosity) {
    let level = match verbosity {
        Verbosity::Quiet => Level::ERROR,
        Verbosity::Normal => Level::INFO,
        Verbosity::Verbose => Level::DEBUG,
    };
    tracing_subscriber::fmt()
        .with_max_level(level)
        .with_writer(std::io::stderr)
        .with_target(false)
        .without_time()
        .init();
}

/// Runs the command with the arguments of the process.
fn main() -> ExitCode {
    let args = match Args::parse(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(ArgsError::Help) => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Err(e) => {
            eprintln!("error: {e}\n\n{USAGE}");
            return ExitCode::FAILURE;
        }
    };
    init_logging(args.verbosity);
    match cli::run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            error!("{e}");
            ExitCode::FAILURE
        }
    }
}
