//! Errors of the command.

use std::fmt;
use std::io;
use std::path::PathBuf;

/// Error of the command.
#[derive(Debug)]
pub(crate) enum Error {
    /// A file could not be read or written.
    Io(PathBuf, io::Error),
    /// A file is not a valid WAVE, LC3 bitstream, bitrate or loss pattern file.
    InvalidFile(PathBuf, &'static str),
    /// The sampling rate of the input is not supported by LC3.
    UnsupportedSampleRate(u32),
    /// The codec rejected its input.
    Codec(lc3::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(path, error) => write!(f, "{}: {error}", path.display()),
            Self::InvalidFile(path, reason) => write!(f, "{}: {reason}", path.display()),
            Self::UnsupportedSampleRate(hz) => write!(f, "unsupported sampling rate {hz} Hz"),
            Self::Codec(error) => write!(f, "codec error: {error}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(_, error) => Some(error),
            Self::Codec(error) => Some(error),
            Self::InvalidFile(..) | Self::UnsupportedSampleRate(_) => None,
        }
    }
}

impl From<lc3::Error> for Error {
    fn from(error: lc3::Error) -> Self {
        Self::Codec(error)
    }
}
