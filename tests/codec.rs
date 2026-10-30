//! Tests of the public API over all the configurations of the codec.

use lc3::{
    Config, Decoder, Encoder, FrameDuration, FrameStatus, I24, MAX_FRAME_BYTES, MIN_FRAME_BYTES,
    SampleRate,
};

/// All the sampling rates.
const SAMPLE_RATES: [SampleRate; 6] = [
    SampleRate::Hz8000,
    SampleRate::Hz16000,
    SampleRate::Hz24000,
    SampleRate::Hz32000,
    SampleRate::Hz44100,
    SampleRate::Hz48000,
];

/// All the frame durations.
const FRAME_DURATIONS: [FrameDuration; 2] = [FrameDuration::Ms7_5, FrameDuration::Ms10];

/// All the configurations.
fn configs() -> impl Iterator<Item = Config> {
    FRAME_DURATIONS
        .into_iter()
        .flat_map(|fd| SAMPLE_RATES.into_iter().map(move |sr| Config::new(sr, fd)))
}

/// Xorshift pseudo-random generator.
struct Random(u32);

impl Random {
    /// Next pseudo-random value.
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0
    }
}

/// Two tones at 440 Hz and 1250 Hz, sample `n` at the sampling rate `hz`.
fn tones(n: usize, hz: u32) -> i16 {
    let t = n as f64 / f64::from(hz);
    let phase = 2.0 * std::f64::consts::PI * t;
    (8000.0 * (440.0 * phase).sin() + 3000.0 * (1250.0 * phase).sin()) as i16
}

/// The decoded signal follows the input signal, delayed by
/// [`Config::delay_samples`], at all bitrates.
#[test]
fn round_trip_signal_to_noise_ratio() {
    const FRAMES: usize = 40;
    /// Frames ignored while the codec converges.
    const SETTLING_FRAMES: usize = 4;
    /// Smallest acceptable signal to noise ratio, at the lowest bitrates.
    const MIN_SNR_DB: f64 = 10.0;

    for config in configs() {
        let samples = config.frame_samples();
        let delay = config.delay_samples();
        let input: Vec<i16> = (0..FRAMES * samples)
            .map(|n| tones(n, config.sample_rate.hz()))
            .collect();
        for nbytes in [MIN_FRAME_BYTES, 40, 60, 100, 160, 250, MAX_FRAME_BYTES] {
            let mut encoder = Encoder::new(config);
            let mut decoder = Decoder::new(config);
            let mut frame = vec![0; nbytes];
            let mut output = vec![0i16; samples];
            let (mut signal, mut noise) = (0.0f64, 0.0f64);
            for (i, pcm) in input.chunks(samples).enumerate() {
                encoder.encode(pcm, &mut frame).unwrap();
                let status = decoder.decode(Some(&frame), &mut output).unwrap();
                assert_eq!(status, FrameStatus::Decoded);
                if i < SETTLING_FRAMES {
                    continue;
                }
                for (n, &y) in output.iter().enumerate() {
                    let x = f64::from(input[i * samples + n - delay]);
                    signal += x * x;
                    noise += (f64::from(y) - x).powi(2);
                }
            }
            let snr = 10.0 * (signal / noise).log10();
            assert!(
                snr > MIN_SNR_DB,
                "{config:?}, {nbytes} bytes: SNR {snr:.1} dB"
            );
        }
    }
}

/// The sample formats are converted to the same internal representation.
#[test]
fn sample_formats_encode_identically() {
    let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
    let pcm: Vec<i16> = (0..config.frame_samples())
        .map(|n| tones(n, 48_000))
        .collect();

    let mut reference = [0u8; 120];
    Encoder::new(config).encode(&pcm, &mut reference).unwrap();

    let mut frame = [0u8; 120];
    let pcm_24: Vec<I24> = pcm.iter().map(|&s| I24(i32::from(s) << 8)).collect();
    Encoder::new(config).encode(&pcm_24, &mut frame).unwrap();
    assert_eq!(frame, reference);

    let pcm_32: Vec<i32> = pcm.iter().map(|&s| i32::from(s) << 16).collect();
    Encoder::new(config).encode(&pcm_32, &mut frame).unwrap();
    assert_eq!(frame, reference);

    let pcm_float: Vec<f32> = pcm.iter().map(|&s| f32::from(s) / 32_768.0).collect();
    Encoder::new(config).encode(&pcm_float, &mut frame).unwrap();
    assert_eq!(frame, reference);
}

/// Decoding random payloads never panics: the frames are decoded, or
/// concealed when a bit error is detected.
#[test]
fn decoder_survives_random_payloads() {
    const FRAMES: usize = 2000;
    /// Period of the lost frames.
    const LOSS_PERIOD: usize = 97;

    let mut random = Random(0x1234_5678);
    for config in configs() {
        let mut decoder = Decoder::new(config);
        let mut output = vec![0i16; config.frame_samples()];
        let (mut decoded, mut concealed) = (0, 0);
        for i in 0..FRAMES {
            let nbytes =
                MIN_FRAME_BYTES + random.next() as usize % (MAX_FRAME_BYTES - MIN_FRAME_BYTES + 1);
            let frame: Vec<u8> = (0..nbytes).map(|_| random.next() as u8).collect();
            let payload = (i % LOSS_PERIOD != 0).then_some(frame.as_slice());
            match decoder.decode(payload, &mut output).unwrap() {
                FrameStatus::Decoded => decoded += 1,
                FrameStatus::Concealed => concealed += 1,
            }
        }
        assert!(decoded > 0 && concealed > 0, "{config:?}");
    }
}

/// Encoding extreme signals never panics and produces valid frames.
#[test]
fn encoder_survives_extreme_signals() {
    /// Number of frames of each signal.
    const FRAMES_PER_SIGNAL: usize = 8;

    let signals: [fn(usize, usize) -> f32; 6] = [
        // Full scale square wave
        |_, n| if (n / 7) % 2 == 0 { 1.0 } else { -1.0 },
        // Full scale signal at the Nyquist frequency
        |_, n| if n % 2 == 0 { 1.0 } else { -1.0 },
        // Single pulse
        |_, n| if n == 3 { 1.0 } else { 0.0 },
        // Invalid and out of range values
        |_, n| [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 2.0][n % 4],
        // Full scale noise
        |i, n| ((n * 7919 + i * 104_729) % 65_536) as f32 / 32_768.0 - 1.0,
        // Silence
        |_, _| 0.0,
    ];
    for config in configs() {
        let samples = config.frame_samples();
        for nbytes in [MIN_FRAME_BYTES, 57, 150, MAX_FRAME_BYTES] {
            let mut encoder = Encoder::new(config);
            let mut decoder = Decoder::new(config);
            let mut frame = vec![0; nbytes];
            let mut output = vec![0i16; samples];
            for (s, signal) in signals.iter().enumerate() {
                for i in 0..FRAMES_PER_SIGNAL {
                    let pcm: Vec<f32> = (0..samples).map(|n| signal(i, n)).collect();
                    encoder.encode(&pcm, &mut frame).unwrap();
                    let status = decoder.decode(Some(&frame), &mut output).unwrap();
                    assert_eq!(
                        status,
                        FrameStatus::Decoded,
                        "{config:?}, {nbytes} bytes, signal {s}"
                    );
                }
            }
            let pcm: Vec<i32> = (0..samples)
                .map(|n| if n % 2 == 0 { i32::MIN } else { i32::MAX })
                .collect();
            encoder.encode(&pcm, &mut frame).unwrap();
            assert_eq!(
                decoder.decode(Some(&frame), &mut output).unwrap(),
                FrameStatus::Decoded
            );
        }
    }
}
