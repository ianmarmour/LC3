//! Encoding and decoding of files.

mod args;
mod error;
mod stream;
mod wav;

use std::fs;
use std::path::Path;

use lc3::{
    Config, Decoder, Encoder, FrameDuration, FrameStatus, MAX_FRAME_BYTES, MIN_FRAME_BYTES,
    SampleRate,
};
use tracing::{debug, info, warn};

pub(crate) use args::{Args, ArgsError, USAGE, Verbosity};
use args::{Bitrate, DelayCompensation, Mode, OutputFormat};
pub(crate) use error::Error;
use stream::Stream;
use wav::{Samples, Wave, WaveSample};

/// Size in bytes of the bitrates of a bitrate switching file.
const BITRATE_BYTES: usize = 8;

/// Size in bytes of the values of a frame loss pattern file.
const LOSS_PATTERN_BYTES: usize = 2;

/// Reads a file.
fn read(path: &Path) -> Result<Vec<u8>, Error> {
    fs::read(path).map_err(|e| Error::Io(path.into(), e))
}

/// Writes a file.
fn write(path: &Path, bytes: &[u8]) -> Result<(), Error> {
    fs::write(path, bytes).map_err(|e| Error::Io(path.into(), e))
}

/// Bitrate of each frame, in bits per second, looped over the frames.
fn read_bitrates(bitrate: &Bitrate) -> Result<Vec<u32>, Error> {
    match bitrate {
        Bitrate::Fixed(bps) => Ok(vec![*bps]),
        Bitrate::Switching(path) => {
            let bitrates: Vec<u32> = read(path)?
                .chunks_exact(BITRATE_BYTES)
                .map(|c| u64::from_le_bytes(c.try_into().expect("chunks of 8 bytes")) as u32)
                .collect();
            if bitrates.is_empty() {
                return Err(Error::InvalidFile(
                    path.clone(),
                    "empty bitrate switching file",
                ));
            }
            Ok(bitrates)
        }
    }
}

/// Whether each frame is lost, looped over the frames.
fn read_loss_pattern(path: &Path) -> Result<Vec<bool>, Error> {
    let lost: Vec<bool> = read(path)?
        .chunks_exact(LOSS_PATTERN_BYTES)
        .map(|c| u16::from_le_bytes([c[0], c[1]]) != 0)
        .collect();
    if lost.is_empty() {
        return Err(Error::InvalidFile(
            path.into(),
            "empty frame loss pattern file",
        ));
    }
    Ok(lost)
}

/// Encodes the interleaved `samples` of `channels` channels.
fn encode_samples<S: WaveSample>(
    config: Config,
    channels: usize,
    samples: &[S],
    bitrates: &[u32],
) -> Result<Vec<Vec<u8>>, Error> {
    // Incomplete sample frames are dropped
    let samples = &samples[..samples.len() / channels * channels];
    let frame_samples = config.frame_samples() * channels;
    let mut encoders = vec![Encoder::new(config); channels];
    let mut frames = Vec::with_capacity(samples.len().div_ceil(frame_samples) * channels);
    let mut pcm = vec![S::default(); frame_samples];
    for (i, chunk) in samples.chunks(frame_samples).enumerate() {
        // The last frame is padded with zeros
        pcm[..chunk.len()].copy_from_slice(chunk);
        pcm[chunk.len()..].fill(S::default());
        let nbytes = config
            .frame_bytes(bitrates[i % bitrates.len()])
            .clamp(MIN_FRAME_BYTES, MAX_FRAME_BYTES);
        for (channel, encoder) in encoders.iter_mut().enumerate() {
            let mut frame = vec![0; nbytes];
            encoder.encode_interleaved(&pcm[channel..], channels, &mut frame)?;
            frames.push(frame);
        }
    }
    Ok(frames)
}

/// Encodes a WAVE file.
fn encode(input: &Wave, frame_duration: FrameDuration, bitrates: &[u32]) -> Result<Stream, Error> {
    let sample_rate = SampleRate::from_hz(input.sample_rate)
        .ok_or(Error::UnsupportedSampleRate(input.sample_rate))?;
    let config = Config::new(sample_rate, frame_duration);
    let channels = input.channels;
    info!(
        "encoding {} channels at {} Hz in frames of {} us, at {} bit/s",
        channels,
        sample_rate.hz(),
        frame_duration.micros(),
        bitrates[0],
    );
    let frames = match &input.samples {
        Samples::I16(s) => encode_samples(config, channels, s, bitrates)?,
        Samples::I24(s) => encode_samples(config, channels, s, bitrates)?,
        Samples::I32(s) => encode_samples(config, channels, s, bitrates)?,
        Samples::F32(s) => encode_samples(config, channels, s, bitrates)?,
    };
    Ok(Stream {
        config,
        channels,
        bitrate: bitrates[0],
        signal_len: input.frames(),
        frames,
    })
}

/// Decodes a stream to interleaved samples of the format `S`, with the
/// length of the encoded signal.
fn decode_samples<S: WaveSample>(
    stream: &Stream,
    lost: &[bool],
    delay_compensation: DelayCompensation,
) -> Result<Vec<S>, Error> {
    let config = stream.config;
    let channels = stream.channels;
    let frame_samples = config.frame_samples() * channels;
    let mut decoders = vec![Decoder::new(config); channels];
    let mut samples = vec![S::default(); stream.frames.len() * config.frame_samples()];
    let mut concealed = 0;
    for (i, (frames, pcm)) in stream
        .frames
        .chunks(channels)
        .zip(samples.chunks_mut(frame_samples))
        .enumerate()
    {
        let frame_lost = !lost.is_empty() && lost[i % lost.len()];
        for (channel, (frame, decoder)) in frames.iter().zip(&mut decoders).enumerate() {
            let valid = (MIN_FRAME_BYTES..=MAX_FRAME_BYTES).contains(&frame.len());
            let payload = (valid && !frame_lost).then_some(frame.as_slice());
            let status = decoder.decode_interleaved(payload, &mut pcm[channel..], channels)?;
            if status == FrameStatus::Concealed {
                debug!("frame {i} of channel {channel} concealed");
                concealed += 1;
            }
        }
    }
    if concealed > 0 {
        warn!("{concealed} frames concealed");
    }

    let delay = match delay_compensation {
        DelayCompensation::None => 0,
        DelayCompensation::Decoder => config.delay_samples() * channels,
    };
    samples.drain(..delay.min(samples.len()));
    samples.resize(stream.signal_len * channels, S::default());
    Ok(samples)
}

/// Decodes a stream to a WAVE file.
fn decode(
    stream: &Stream,
    lost: &[bool],
    delay_compensation: DelayCompensation,
    format: OutputFormat,
) -> Result<Wave, Error> {
    info!(
        "decoding {} frames of {} channels at {} Hz",
        stream.frames.len() / stream.channels,
        stream.channels,
        stream.config.sample_rate.hz(),
    );
    let samples = match format {
        OutputFormat::Int16 => Samples::I16(decode_samples(stream, lost, delay_compensation)?),
        OutputFormat::Int24 => Samples::I24(decode_samples(stream, lost, delay_compensation)?),
        OutputFormat::Int32 => Samples::I32(decode_samples(stream, lost, delay_compensation)?),
    };
    Ok(Wave {
        sample_rate: stream.config.sample_rate.hz(),
        channels: stream.channels,
        samples,
    })
}

/// Runs the command.
pub(crate) fn run(args: &Args) -> Result<(), Error> {
    let stream = match &args.mode {
        Mode::Decode => Stream::parse(&read(&args.input)?)
            .map_err(|e| Error::InvalidFile(args.input.clone(), e))?,
        Mode::Encode(bitrate) | Mode::EncodeDecode(bitrate) => {
            let input = Wave::parse(&read(&args.input)?)
                .map_err(|e| Error::InvalidFile(args.input.clone(), e))?;
            encode(&input, args.frame_duration, &read_bitrates(bitrate)?)?
        }
    };
    if let Mode::Encode(_) = args.mode {
        return write(&args.output, &stream.to_bytes());
    }

    let lost = match &args.loss_pattern {
        Some(path) => read_loss_pattern(path)?,
        None => Vec::new(),
    };
    let output = decode(&stream, &lost, args.delay_compensation, args.output_format)?;
    write(&args.output, &output.to_bytes())
}
