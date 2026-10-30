//! Session configuration and general codec parameters (section 3.2).

/// Maximum number of samples of a frame (`N_F`), for any configuration.
pub const MAX_FRAME_SAMPLES: usize = 480;

/// Minimum size of an encoded frame, in bytes.
pub const MIN_FRAME_BYTES: usize = 20;

/// Maximum size of an encoded frame, in bytes.
pub const MAX_FRAME_BYTES: usize = 400;

/// Maximum number of encoded spectral lines (`N_E`), for any configuration.
pub(crate) const MAX_SPECTRAL_LINES: usize = 400;

/// Number of bands of the band tables (`N_B`), at most.
pub(crate) const MAX_BANDS: usize = 64;

/// Frame duration (`N_ms`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FrameDuration {
    /// 7.5 ms frames.
    Ms7_5,
    /// 10 ms frames.
    Ms10,
}

impl FrameDuration {
    /// Duration of a frame in microseconds.
    pub const fn micros(self) -> u32 {
        match self {
            Self::Ms7_5 => 7_500,
            Self::Ms10 => 10_000,
        }
    }

    /// Frame duration from a duration in microseconds.
    pub const fn from_micros(micros: u32) -> Option<Self> {
        match micros {
            7_500 => Some(Self::Ms7_5),
            10_000 => Some(Self::Ms10),
            _ => None,
        }
    }
}

/// Sampling rate (`f_s`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SampleRate {
    /// 8 kHz.
    Hz8000,
    /// 16 kHz.
    Hz16000,
    /// 24 kHz.
    Hz24000,
    /// 32 kHz.
    Hz32000,
    /// 44.1 kHz, coded with the frame size of 48 kHz.
    Hz44100,
    /// 48 kHz.
    Hz48000,
}

impl SampleRate {
    /// Sampling rate in Hz.
    pub const fn hz(self) -> u32 {
        match self {
            Self::Hz8000 => 8_000,
            Self::Hz16000 => 16_000,
            Self::Hz24000 => 24_000,
            Self::Hz32000 => 32_000,
            Self::Hz44100 => 44_100,
            Self::Hz48000 => 48_000,
        }
    }

    /// Sampling rate from a frequency in Hz.
    pub const fn from_hz(hz: u32) -> Option<Self> {
        match hz {
            8_000 => Some(Self::Hz8000),
            16_000 => Some(Self::Hz16000),
            24_000 => Some(Self::Hz24000),
            32_000 => Some(Self::Hz32000),
            44_100 => Some(Self::Hz44100),
            48_000 => Some(Self::Hz48000),
            _ => None,
        }
    }

    /// Sampling rate index `fs_ind` (equation 1), 44.1 kHz sharing the index of 48 kHz.
    pub(crate) const fn index(self) -> usize {
        match self {
            Self::Hz8000 => 0,
            Self::Hz16000 => 1,
            Self::Hz24000 => 2,
            Self::Hz32000 => 3,
            Self::Hz44100 | Self::Hz48000 => 4,
        }
    }
}

/// Audio bandwidth of a frame (`P_bw`, table 3.6).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum Bandwidth {
    /// Narrow band, 0 to 4 kHz.
    Narrow,
    /// Wide band, 0 to 8 kHz.
    Wide,
    /// Semi super wide band, 0 to 12 kHz.
    SemiSuperWide,
    /// Super wide band, 0 to 16 kHz.
    SuperWide,
    /// Full band, 0 to 20 kHz.
    Full,
}

impl Bandwidth {
    /// All the bandwidths, by increasing index.
    const ALL: [Self; 5] = [
        Self::Narrow,
        Self::Wide,
        Self::SemiSuperWide,
        Self::SuperWide,
        Self::Full,
    ];

    /// Bandwidth of an index `P_bw`.
    pub(crate) fn from_index(index: usize) -> Option<Self> {
        Self::ALL.get(index).copied()
    }

    /// Index `P_bw` of the bandwidth.
    pub(crate) const fn index(self) -> usize {
        self as usize
    }

    /// Number of spectral lines covered by the bandwidth, `bw_stop`
    /// (tables 3.16 and 3.18).
    pub(crate) const fn stop_line(self, frame_duration: FrameDuration) -> usize {
        let lines_10ms = 80 * (self.index() + 1);
        match frame_duration {
            FrameDuration::Ms10 => lines_10ms,
            FrameDuration::Ms7_5 => lines_10ms * 3 / 4,
        }
    }
}

/// Configuration of an encoder or of a decoder.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Config {
    /// Sampling rate.
    pub sample_rate: SampleRate,
    /// Frame duration.
    pub frame_duration: FrameDuration,
}

impl Config {
    /// New configuration.
    pub const fn new(sample_rate: SampleRate, frame_duration: FrameDuration) -> Self {
        Self {
            sample_rate,
            frame_duration,
        }
    }

    /// Number of samples of a frame (`N_F`).
    pub const fn frame_samples(&self) -> usize {
        let samples_10ms = match self.sample_rate {
            SampleRate::Hz8000 => 80,
            SampleRate::Hz16000 => 160,
            SampleRate::Hz24000 => 240,
            SampleRate::Hz32000 => 320,
            SampleRate::Hz44100 | SampleRate::Hz48000 => 480,
        };
        match self.frame_duration {
            FrameDuration::Ms10 => samples_10ms,
            FrameDuration::Ms7_5 => samples_10ms * 3 / 4,
        }
    }

    /// Delay in samples of the decoded signal relative to the input signal,
    /// in addition to the frame buffering: the decoded sample `n` corresponds
    /// to the input sample `n - delay_samples()`.
    pub const fn delay_samples(&self) -> usize {
        self.frame_samples() - 2 * self.mdct_zeros()
    }

    /// Size in bytes of the frames encoded at `bitrate` bits per second
    /// (section 3.2.5), not limited to the range of frame sizes.
    pub const fn frame_bytes(&self, bitrate: u32) -> usize {
        // nbytes = floor(bitrate * N_ms * fscal / 8000), with fscal = 48000 / 44100 at 44.1 kHz
        let (scale_num, scale_den): (u64, u64) = match self.sample_rate {
            SampleRate::Hz44100 => (48_000, 44_100),
            _ => (1, 1),
        };
        let bits = bitrate as u64 * self.frame_duration.micros() as u64 * scale_num;
        (bits / (8 * 1_000_000 * scale_den)) as usize
    }

    /// Bitrate in bits per second of frames of `nbytes` bytes (section 3.2.5).
    pub const fn bitrate(&self, nbytes: usize) -> u32 {
        let bits_per_second = 8 * nbytes as u64 * self.sample_rate.hz() as u64;
        bits_per_second.div_ceil(self.frame_samples() as u64) as u32
    }

    /// Number of encoded spectral lines `N_E` (equation 9).
    pub(crate) const fn spectral_lines(&self) -> usize {
        match self.frame_samples() {
            480 => 400,
            360 => 300,
            samples => samples,
        }
    }

    /// Number of leading zeros `Z` of the MDCT window (equation 3).
    pub(crate) const fn mdct_zeros(&self) -> usize {
        match self.frame_duration {
            FrameDuration::Ms10 => 3 * self.frame_samples() / 8,
            FrameDuration::Ms7_5 => 7 * self.frame_samples() / 30,
        }
    }

    /// Band edges `I_fs` (sections 3.7.1 and 3.7.2), `N_B + 1` values.
    pub(crate) const fn bands(&self) -> &'static [u16] {
        match (self.frame_duration, self.sample_rate.index()) {
            (FrameDuration::Ms10, 0) => &BANDS_10MS_8000,
            (FrameDuration::Ms10, 1) => &BANDS_10MS_16000,
            (FrameDuration::Ms10, 2) => &BANDS_10MS_24000,
            (FrameDuration::Ms10, 3) => &BANDS_10MS_32000,
            (FrameDuration::Ms10, _) => &BANDS_10MS_48000,
            (FrameDuration::Ms7_5, 0) => &BANDS_7M5_8000,
            (FrameDuration::Ms7_5, 1) => &BANDS_7M5_16000,
            (FrameDuration::Ms7_5, 2) => &BANDS_7M5_24000,
            (FrameDuration::Ms7_5, 3) => &BANDS_7M5_32000,
            (FrameDuration::Ms7_5, _) => &BANDS_7M5_48000,
        }
    }

    /// Number of bands `N_B`.
    pub(crate) const fn band_count(&self) -> usize {
        self.bands().len() - 1
    }

    /// Widest bandwidth of the sampling rate (`N_bw` of table 3.6).
    pub(crate) const fn max_bandwidth(&self) -> Bandwidth {
        Bandwidth::ALL[self.sample_rate.index()]
    }

    /// Number of bits of the bandwidth `P_bw` in the bitstream (table 3.6).
    pub(crate) const fn bandwidth_bits(&self) -> u32 {
        match self.sample_rate.index() {
            0 => 0,
            1 => 1,
            2 | 3 => 2,
            _ => 3,
        }
    }

    /// Number of bits of the last non-zero 2-tuple index, `ceil(log2(N_E / 2))`.
    pub(crate) const fn lastnz_bits(&self) -> u32 {
        let tuples = (self.spectral_lines() / 2) as u32;
        u32::BITS - (tuples - 1).leading_zeros()
    }
}

/// Band edges for 8 kHz at 10 ms (section 3.7.1).
const BANDS_10MS_8000: [u16; 65] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 80,
];

/// Band edges for 16 kHz at 10 ms (section 3.7.1).
const BANDS_10MS_16000: [u16; 65] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 55, 58, 61, 64, 67, 70, 73, 76, 80,
    84, 88, 92, 96, 101, 106, 111, 116, 121, 127, 133, 139, 146, 153, 160,
];

/// Band edges for 24 kHz at 10 ms (section 3.7.1).
const BANDS_10MS_24000: [u16; 65] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 25, 27,
    29, 31, 33, 35, 37, 39, 41, 43, 46, 49, 52, 55, 58, 61, 64, 68, 72, 76, 80, 85, 90, 95, 100,
    106, 112, 118, 125, 132, 139, 147, 155, 164, 173, 183, 193, 204, 215, 227, 240,
];

/// Band edges for 32 kHz at 10 ms (section 3.7.1).
const BANDS_10MS_32000: [u16; 65] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 24, 26, 28, 30,
    32, 34, 36, 38, 41, 44, 47, 50, 53, 56, 60, 64, 68, 72, 76, 81, 86, 91, 97, 103, 109, 116, 123,
    131, 139, 148, 157, 166, 176, 187, 199, 211, 224, 238, 252, 268, 284, 302, 320,
];

/// Band edges for 44.1 and 48 kHz at 10 ms (section 3.7.1).
const BANDS_10MS_48000: [u16; 65] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 22, 24, 26, 28, 30, 32,
    34, 36, 39, 42, 45, 48, 51, 55, 59, 63, 67, 71, 76, 81, 86, 92, 98, 105, 112, 119, 127, 135,
    144, 154, 164, 175, 186, 198, 211, 225, 240, 256, 273, 291, 310, 330, 352, 375, 400,
];

/// Band edges for 8 kHz at 7.5 ms (section 3.7.2).
const BANDS_7M5_8000: [u16; 61] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
];

/// Band edges for 16 kHz at 7.5 ms (section 3.7.2).
const BANDS_7M5_16000: [u16; 65] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 65,
    68, 71, 74, 77, 80, 83, 86, 90, 94, 98, 102, 106, 110, 115, 120,
];

/// Band edges for 24 kHz at 7.5 ms (section 3.7.2).
const BANDS_7M5_24000: [u16; 65] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 52, 55, 58, 61, 64, 67, 70, 74, 78, 82, 86,
    90, 95, 100, 105, 110, 115, 121, 127, 134, 141, 148, 155, 163, 171, 180,
];

/// Band edges for 32 kHz at 7.5 ms (section 3.7.2).
const BANDS_7M5_32000: [u16; 65] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 26,
    28, 30, 32, 34, 36, 38, 40, 42, 45, 48, 51, 54, 57, 60, 63, 67, 71, 75, 79, 84, 89, 94, 99,
    105, 111, 117, 124, 131, 138, 146, 154, 163, 172, 182, 192, 203, 215, 227, 240,
];

/// Band edges for 44.1 and 48 kHz at 7.5 ms (section 3.7.2).
const BANDS_7M5_48000: [u16; 65] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 24, 26, 28,
    30, 32, 34, 36, 38, 40, 43, 46, 49, 52, 55, 59, 63, 67, 71, 75, 80, 85, 90, 96, 102, 108, 115,
    122, 129, 137, 146, 155, 165, 175, 186, 197, 209, 222, 236, 251, 266, 283, 300,
];

#[cfg(test)]
mod tests {
    use super::{Config, FrameDuration, SampleRate};

    #[test]
    fn frame_parameters() {
        let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
        assert_eq!(config.frame_samples(), 480);
        assert_eq!(config.spectral_lines(), 400);
        assert_eq!(config.mdct_zeros(), 180);
        assert_eq!(config.delay_samples(), 120);

        let config = Config::new(SampleRate::Hz44100, FrameDuration::Ms7_5);
        assert_eq!(config.frame_samples(), 360);
        assert_eq!(config.spectral_lines(), 300);
        assert_eq!(config.mdct_zeros(), 84);
        assert_eq!(config.delay_samples(), 192);

        let config = Config::new(SampleRate::Hz8000, FrameDuration::Ms7_5);
        assert_eq!(config.band_count(), 60);
        assert_eq!(config.lastnz_bits(), 5);
    }

    #[test]
    fn band_tables_end_at_the_spectral_lines() {
        for frame_duration in [FrameDuration::Ms7_5, FrameDuration::Ms10] {
            for hz in [8_000, 16_000, 24_000, 32_000, 48_000] {
                let config = Config::new(SampleRate::from_hz(hz).unwrap(), frame_duration);
                let bands = config.bands();
                assert!(bands.windows(2).all(|w| w[0] < w[1]));
                assert_eq!(*bands.last().unwrap() as usize, config.spectral_lines());
            }
        }
    }

    #[test]
    fn frame_bytes_of_the_test_suite_bitrates() {
        let cases = [
            (SampleRate::Hz44100, FrameDuration::Ms10, 79_380, 108),
            (SampleRate::Hz44100, FrameDuration::Ms10, 95_550, 130),
            (SampleRate::Hz44100, FrameDuration::Ms10, 123_480, 168),
            (SampleRate::Hz44100, FrameDuration::Ms7_5, 79_380, 81),
            (SampleRate::Hz44100, FrameDuration::Ms7_5, 95_060, 97),
            (SampleRate::Hz44100, FrameDuration::Ms7_5, 123_480, 126),
            (SampleRate::Hz8000, FrameDuration::Ms7_5, 27_734, 26),
            (SampleRate::Hz32000, FrameDuration::Ms7_5, 61_867, 58),
            (SampleRate::Hz48000, FrameDuration::Ms7_5, 124_800, 117),
        ];
        for (sample_rate, frame_duration, bitrate, nbytes) in cases {
            let config = Config::new(sample_rate, frame_duration);
            assert_eq!(config.frame_bytes(bitrate), nbytes);
        }
    }
}
