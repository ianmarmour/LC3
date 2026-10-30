//! A `no_std` implementation of the Bluetooth Low Complexity Communication
//! Codec (LC3), as specified by the LC3 specification v1.0.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs, clippy::missing_docs_in_private_items)]

mod config;
mod error;
mod sample;

pub use config::{
    Config, FrameDuration, MAX_FRAME_BYTES, MAX_FRAME_SAMPLES, MIN_FRAME_BYTES, SampleRate,
};
pub use error::Error;
pub use sample::{I24, Sample};
