//! LC3 bitstream files of the reference software: a header followed by the
//! frames, each preceded by its size.

use lc3::{Config, FrameDuration, SampleRate};

/// Identifier of the files, the first header field.
const FILE_ID: u16 = 0xcc1c;

/// Size of the header in bytes.
const HEADER_SIZE: usize = 18;

/// Unit of the sampling rate and bitrate fields of the header.
const RATE_UNIT: u32 = 100;

/// Unit of the frame duration field of the header, in microseconds.
const FRAME_DURATION_UNIT: u32 = 10;

/// Size in bytes of the frame size preceding each frame.
const FRAME_SIZE_BYTES: usize = 2;

/// Content of an LC3 bitstream file.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Stream {
    /// Configuration of the codec.
    pub(crate) config: Config,
    /// Number of channels.
    pub(crate) channels: usize,
    /// Bitrate of the first frame, in bits per second.
    pub(crate) bitrate: u32,
    /// Number of samples per channel of the encoded signal.
    pub(crate) signal_len: usize,
    /// Frames, those of the channels of a frame interval being consecutive.
    pub(crate) frames: Vec<Vec<u8>>,
}

impl Stream {
    /// Parses a bitstream file.
    pub(crate) fn parse(bytes: &[u8]) -> Result<Self, &'static str> {
        let field = |i: usize| -> Result<u16, &'static str> {
            bytes
                .get(2 * i..2 * i + 2)
                .map(|v| u16::from_le_bytes([v[0], v[1]]))
                .ok_or("truncated header")
        };
        if field(0)? != FILE_ID {
            return Err("not an LC3 bitstream file");
        }
        let header_size = usize::from(field(1)?);
        let sample_rate = SampleRate::from_hz(u32::from(field(2)?) * RATE_UNIT)
            .ok_or("unsupported sampling rate")?;
        let bitrate = u32::from(field(3)?) * RATE_UNIT;
        let channels = usize::from(field(4)?).max(1);
        let frame_duration = FrameDuration::from_micros(u32::from(field(5)?) * FRAME_DURATION_UNIT)
            .ok_or("unsupported frame duration")?;
        let mut signal_len = usize::from(field(7)?);
        // Older files have a shorter header, without the high half of the length
        if header_size >= HEADER_SIZE {
            signal_len |= usize::from(field(8)?) << 16;
        }

        let mut frames = Vec::new();
        let mut offset = header_size;
        while let Some(size) = bytes.get(offset..offset + FRAME_SIZE_BYTES) {
            let size = usize::from(u16::from_le_bytes([size[0], size[1]]));
            let start = offset + FRAME_SIZE_BYTES;
            let end = (start + size).min(bytes.len());
            frames.push(bytes[start..end].to_vec());
            offset = end;
        }
        // Incomplete frame intervals are dropped
        frames.truncate(frames.len() / channels * channels);

        Ok(Self {
            config: Config::new(sample_rate, frame_duration),
            channels,
            bitrate,
            signal_len,
            frames,
        })
    }

    /// The bitstream file.
    pub(crate) fn to_bytes(&self) -> Vec<u8> {
        let header: [u16; HEADER_SIZE / 2] = [
            FILE_ID,
            HEADER_SIZE as u16,
            (self.config.sample_rate.hz() / RATE_UNIT) as u16,
            (self.bitrate / RATE_UNIT) as u16,
            self.channels as u16,
            (self.config.frame_duration.micros() / FRAME_DURATION_UNIT) as u16,
            0,
            (self.signal_len & 0xffff) as u16,
            (self.signal_len >> 16) as u16,
        ];
        let mut out = Vec::new();
        for field in header {
            out.extend_from_slice(&field.to_le_bytes());
        }
        for frame in &self.frames {
            out.extend_from_slice(&(frame.len() as u16).to_le_bytes());
            out.extend_from_slice(frame);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::Stream;
    use lc3::{Config, FrameDuration, SampleRate};

    #[test]
    fn round_trip() {
        let stream = Stream {
            config: Config::new(SampleRate::Hz44100, FrameDuration::Ms7_5),
            channels: 2,
            bitrate: 64_000,
            signal_len: 70_000,
            frames: vec![vec![1; 40], vec![2; 40], vec![3; 41], vec![4; 41]],
        };
        assert_eq!(Stream::parse(&stream.to_bytes()), Ok(stream));
    }

    #[test]
    fn rejects_invalid_files() {
        assert_eq!(Stream::parse(&[0x1c]), Err("truncated header"));
        assert_eq!(Stream::parse(&[0; 18]), Err("not an LC3 bitstream file"));
    }
}
