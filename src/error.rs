//! Errors of the encoder and of the decoder.

use core::fmt;

use crate::config::{MAX_FRAME_BYTES, MIN_FRAME_BYTES};

/// Error returned by the encoder and the decoder on invalid arguments.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The size of the encoded frame is out of the range
    /// [`MIN_FRAME_BYTES`] to [`MAX_FRAME_BYTES`].
    InvalidFrameSize,
    /// The PCM buffer is too short for a frame of samples.
    InvalidPcmLength,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidFrameSize => write!(
                f,
                "encoded frame size out of the range {MIN_FRAME_BYTES} to {MAX_FRAME_BYTES} bytes"
            ),
            Self::InvalidPcmLength => write!(f, "PCM buffer too short for a frame"),
        }
    }
}

impl core::error::Error for Error {}
