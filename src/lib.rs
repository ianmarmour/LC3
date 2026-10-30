//! A `no_std` implementation of the Bluetooth Low Complexity Communication
//! Codec (LC3), as specified by the LC3 specification v1.0.
//!
//! The codec codes a single audio channel: multi-channel audio is coded with
//! one [`Encoder`] and one [`Decoder`] per channel. Neither allocates, their
//! whole state is held in the structures themselves.
//!
//! ```
//! use lc3::{Config, Decoder, Encoder, FrameDuration, SampleRate};
//!
//! let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
//! let mut encoder = Encoder::new(config);
//! let mut decoder = Decoder::new(config);
//!
//! let pcm = [0i16; 480];
//! let mut frame = [0u8; 120];
//! encoder.encode(&pcm, &mut frame)?;
//!
//! let mut output = [0i16; 480];
//! decoder.decode(Some(&frame), &mut output)?;
//! # Ok::<(), lc3::Error>(())
//! ```

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs, clippy::missing_docs_in_private_items)]

mod bitstream;
mod config;
mod decoder;
mod encoder;
mod error;
mod ltpf;
mod math;
mod mdct;
mod sample;
mod sns;
mod spectrum;
#[cfg(test)]
mod test_vectors;
mod tns;

pub use config::{
    Config, FrameDuration, MAX_FRAME_BYTES, MAX_FRAME_SAMPLES, MIN_FRAME_BYTES, SampleRate,
};
pub use decoder::{Decoder, FrameStatus};
pub use encoder::Encoder;
pub use error::Error;
pub use sample::{I24, Sample};
