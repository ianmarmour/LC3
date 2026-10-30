//! Reading and writing of PCM WAVE files.

use lc3::I24;

/// Format tag of integer PCM samples.
const FORMAT_PCM: u16 = 1;

/// Format tag of floating point samples.
const FORMAT_FLOAT: u16 = 3;

/// Format tag of the extensible format, the actual tag following in the
/// extension of the format chunk.
const FORMAT_EXTENSIBLE: u16 = 0xfffe;

/// Size of the format chunk without extension.
const FORMAT_CHUNK_SIZE: usize = 16;

/// Offset of the actual format tag in an extensible format chunk.
const EXTENSIBLE_TAG_OFFSET: usize = 24;

/// Size of the header of a RIFF chunk: its identifier and its size.
const CHUNK_HEADER_SIZE: usize = 8;

/// Size of the header written by [`Wave::to_bytes`].
const HEADER_SIZE: usize = 44;

/// A sample format of WAVE files.
pub(crate) trait WaveSample: lc3::Sample + Default {
    /// Format tag of the samples.
    const FORMAT: u16;
    /// Size of a sample in bytes.
    const BYTES: usize;

    /// Sample of its little-endian representation of [`Self::BYTES`] bytes.
    fn read_le(bytes: &[u8]) -> Self;

    /// Appends the little-endian representation of the sample to `out`.
    fn write_le(self, out: &mut Vec<u8>);
}

impl WaveSample for i16 {
    const FORMAT: u16 = FORMAT_PCM;
    const BYTES: usize = 2;

    fn read_le(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1]])
    }

    fn write_le(self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.to_le_bytes());
    }
}

impl WaveSample for I24 {
    const FORMAT: u16 = FORMAT_PCM;
    const BYTES: usize = 3;

    fn read_le(bytes: &[u8]) -> Self {
        // Sign extension through the most significant byte of an i32
        Self(i32::from_le_bytes([0, bytes[0], bytes[1], bytes[2]]) >> 8)
    }

    fn write_le(self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.0.to_le_bytes()[..Self::BYTES]);
    }
}

impl WaveSample for i32 {
    const FORMAT: u16 = FORMAT_PCM;
    const BYTES: usize = 4;

    fn read_le(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn write_le(self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.to_le_bytes());
    }
}

impl WaveSample for f32 {
    const FORMAT: u16 = FORMAT_FLOAT;
    const BYTES: usize = 4;

    fn read_le(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn write_le(self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.to_le_bytes());
    }
}

/// Interleaved samples of a WAVE file.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Samples {
    /// 16 bits integer samples.
    I16(Vec<i16>),
    /// 24 bits integer samples.
    I24(Vec<I24>),
    /// 32 bits integer samples.
    I32(Vec<i32>),
    /// 32 bits floating point samples.
    F32(Vec<f32>),
}

impl Samples {
    /// Total number of samples of all the channels.
    pub(crate) fn len(&self) -> usize {
        match self {
            Self::I16(s) => s.len(),
            Self::I24(s) => s.len(),
            Self::I32(s) => s.len(),
            Self::F32(s) => s.len(),
        }
    }
}

/// Content of a WAVE file.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Wave {
    /// Sampling rate in Hz.
    pub(crate) sample_rate: u32,
    /// Number of channels.
    pub(crate) channels: usize,
    /// Interleaved samples.
    pub(crate) samples: Samples,
}

/// Little-endian 16 bits integer at `offset`.
fn u16_at(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

/// Little-endian 32 bits integer at `offset`.
fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

/// Samples of the data chunk.
fn read_samples<S: WaveSample>(data: &[u8]) -> Vec<S> {
    data.chunks_exact(S::BYTES).map(S::read_le).collect()
}

/// Format of the samples: format tag, number of channels, sampling rate
/// and bits per sample.
struct Format {
    /// Format tag, resolved for the extensible format.
    tag: u16,
    /// Number of channels.
    channels: u16,
    /// Sampling rate in Hz.
    sample_rate: u32,
    /// Number of bits per sample.
    bits_per_sample: u16,
}

impl Format {
    /// Format of the body of a format chunk.
    fn parse(body: &[u8]) -> Result<Self, &'static str> {
        if body.len() < FORMAT_CHUNK_SIZE {
            return Err("invalid format chunk");
        }
        let mut tag = u16_at(body, 0);
        if tag == FORMAT_EXTENSIBLE && body.len() >= EXTENSIBLE_TAG_OFFSET + 2 {
            tag = u16_at(body, EXTENSIBLE_TAG_OFFSET);
        }
        Ok(Self {
            tag,
            channels: u16_at(body, 2),
            sample_rate: u32_at(body, 4),
            bits_per_sample: u16_at(body, 14),
        })
    }
}

impl Wave {
    /// Number of samples per channel.
    pub(crate) fn frames(&self) -> usize {
        self.samples.len() / self.channels
    }

    /// Parses a WAVE file of 16, 24 or 32 bits integer samples, or of 32
    /// bits floating point samples.
    pub(crate) fn parse(bytes: &[u8]) -> Result<Self, &'static str> {
        if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
            return Err("not a WAVE file");
        }
        let mut format = None;
        let mut offset = 12;
        while offset + CHUNK_HEADER_SIZE <= bytes.len() {
            let id = &bytes[offset..offset + 4];
            let size = u32_at(bytes, offset + 4) as usize;
            let start = offset + CHUNK_HEADER_SIZE;
            let body = &bytes[start..start.saturating_add(size).min(bytes.len())];
            match id {
                b"fmt " => format = Some(Format::parse(body)?),
                b"data" => {
                    let format = format.ok_or("missing format chunk")?;
                    if format.channels == 0 {
                        return Err("invalid number of channels");
                    }
                    let samples = match (format.tag, format.bits_per_sample) {
                        (FORMAT_PCM, 16) => Samples::I16(read_samples(body)),
                        (FORMAT_PCM, 24) => Samples::I24(read_samples(body)),
                        (FORMAT_PCM, 32) => Samples::I32(read_samples(body)),
                        (FORMAT_FLOAT, 32) => Samples::F32(read_samples(body)),
                        _ => return Err("unsupported sample format"),
                    };
                    return Ok(Self {
                        sample_rate: format.sample_rate,
                        channels: usize::from(format.channels),
                        samples,
                    });
                }
                _ => {}
            }
            // Chunks are padded to an even size
            offset = start.saturating_add(size + (size & 1));
        }
        Err("missing data chunk")
    }

    /// The WAVE file of the samples.
    pub(crate) fn to_bytes(&self) -> Vec<u8> {
        match &self.samples {
            Samples::I16(s) => self.write(s),
            Samples::I24(s) => self.write(s),
            Samples::I32(s) => self.write(s),
            Samples::F32(s) => self.write(s),
        }
    }

    /// The WAVE file of the samples `samples` of the format `S`.
    fn write<S: WaveSample>(&self, samples: &[S]) -> Vec<u8> {
        let data_size = samples.len() * S::BYTES;
        let block_align = self.channels * S::BYTES;
        let bits_per_sample = 8 * S::BYTES as u16;
        let mut out = Vec::with_capacity(HEADER_SIZE + data_size);
        out.extend_from_slice(b"RIFF");
        out.extend_from_slice(
            &((HEADER_SIZE - CHUNK_HEADER_SIZE + data_size) as u32).to_le_bytes(),
        );
        out.extend_from_slice(b"WAVEfmt ");
        out.extend_from_slice(&(FORMAT_CHUNK_SIZE as u32).to_le_bytes());
        out.extend_from_slice(&S::FORMAT.to_le_bytes());
        out.extend_from_slice(&(self.channels as u16).to_le_bytes());
        out.extend_from_slice(&self.sample_rate.to_le_bytes());
        out.extend_from_slice(&(self.sample_rate * block_align as u32).to_le_bytes());
        out.extend_from_slice(&(block_align as u16).to_le_bytes());
        out.extend_from_slice(&bits_per_sample.to_le_bytes());
        out.extend_from_slice(b"data");
        out.extend_from_slice(&(data_size as u32).to_le_bytes());
        for &sample in samples {
            sample.write_le(&mut out);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::{Samples, Wave};
    use lc3::I24;

    #[test]
    fn round_trip_of_all_formats() {
        for samples in [
            Samples::I16(vec![0, -1, i16::MAX, i16::MIN]),
            Samples::I24(vec![I24(0), I24(-1), I24(0x7f_ffff), I24(-0x80_0000)]),
            Samples::I32(vec![0, -1, i32::MAX, i32::MIN]),
            Samples::F32(vec![0.0, -1.0, 0.5, 1.0]),
        ] {
            let wave = Wave {
                sample_rate: 48_000,
                channels: 2,
                samples,
            };
            let bytes = wave.to_bytes();
            assert_eq!(Wave::parse(&bytes), Ok(wave));
        }
    }

    #[test]
    fn rejects_invalid_files() {
        assert_eq!(Wave::parse(b"RIFF\0\0\0\0WAVX"), Err("not a WAVE file"));
        assert_eq!(Wave::parse(b"RIFF\0\0\0\0WAVE"), Err("missing data chunk"));
        assert_eq!(
            Wave::parse(b"RIFF\0\0\0\0WAVEdata\0\0\0\0"),
            Err("missing format chunk")
        );
    }
}
