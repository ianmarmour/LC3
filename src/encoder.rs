//! LC3 encoder (section 3.3).

mod attack;
mod bandwidth;

use crate::bitstream::Writer;
use crate::config::{
    Config, FrameDuration, MAX_BANDS, MAX_FRAME_BYTES, MAX_FRAME_SAMPLES, MAX_SPECTRAL_LINES,
    MIN_FRAME_BYTES, SampleRate,
};
use crate::error::Error;
use crate::sample::{MAX_SCALED, MIN_SCALED, Sample};
use crate::spectrum::{self, GAIN_INDEX_BITS, LsbBits, LsbMode, NOISE_LEVEL_BITS, RateControl};
use crate::{ltpf, mdct, sns, tns};
use attack::AttackDetector;

/// Energy ratio between the highest bands and the rest of the spectrum
/// detecting a signal near the Nyquist frequency (equation 11).
const NEAR_NYQUIST_RATIO: f32 = 30.0;

/// Number of highest bands of the near Nyquist detector.
const fn near_nyquist_bands(frame_duration: FrameDuration) -> usize {
    match frame_duration {
        FrameDuration::Ms10 => 2,
        FrameDuration::Ms7_5 => 4,
    }
}

/// Number of bits of the side information besides the bandwidth, TNS and
/// LTPF data and the arithmetic coder side information (equation 106): the
/// SNS data, the global gain and the noise level.
const FIXED_SIDE_INFO_BITS: i32 =
    sns::SIDE_INFO_BITS as i32 + GAIN_INDEX_BITS as i32 + NOISE_LEVEL_BITS as i32;

/// Mean energy of each band of the spectrum `x` (equation 10).
fn band_energies(config: &Config, x: &[f32]) -> [f32; MAX_BANDS] {
    let mut eb = [0.0f32; MAX_BANDS];
    for (e, edges) in eb.iter_mut().zip(config.bands().windows(2)) {
        let (start, stop) = (usize::from(edges[0]), usize::from(edges[1]));
        let energy: f32 = x[start..stop].iter().map(|v| v * v).sum();
        *e = energy / (stop - start) as f32;
    }
    eb
}

/// Near Nyquist detector (equation 11): whether the energy of the band
/// energies `eb` is concentrated in the highest bands.
fn near_nyquist(config: &Config, eb: &[f32]) -> bool {
    if matches!(
        config.sample_rate,
        SampleRate::Hz44100 | SampleRate::Hz48000
    ) {
        return false;
    }
    let split = eb.len() - near_nyquist_bands(config.frame_duration);
    let low: f32 = eb[..split].iter().sum();
    let high: f32 = eb[split..].iter().sum();
    high > NEAR_NYQUIST_RATIO * low
}

/// LC3 encoder of a single audio channel.
///
/// Multi-channel audio is encoded with one encoder per channel. The encoder
/// keeps state from frame to frame, so the frames of a channel must be
/// encoded in order with the same encoder.
#[derive(Clone)]
pub struct Encoder {
    /// Configuration of the encoder.
    config: Config,
    /// Scaled input signal: the `N_F - Z` samples of the previous frames
    /// followed by the current frame, the last `Z` samples of the MDCT input
    /// being kept zero.
    input: [f32; 2 * MAX_FRAME_SAMPLES],
    /// Time domain attack detector.
    attack: AttackDetector,
    /// LTPF analysis.
    ltpf: ltpf::Analysis,
    /// Rate control of the global gain estimation.
    rate: RateControl,
    /// Scratch buffers of the MDCT.
    mdct: mdct::Scratch,
    /// Spectrum of the current frame.
    spectrum: [f32; MAX_FRAME_SAMPLES],
    /// Quantized spectrum of the current frame.
    quantized: [i32; MAX_SPECTRAL_LINES],
    /// Residual bits of the current frame.
    residual: [u8; MAX_SPECTRAL_LINES],
    /// Least significant bits of the current frame in the
    /// [`LsbMode::Truncation`] mode.
    lsbs: LsbBits,
}

impl Encoder {
    /// New encoder for a configuration.
    pub const fn new(config: Config) -> Self {
        Self {
            config,
            input: [0.0; 2 * MAX_FRAME_SAMPLES],
            attack: AttackDetector::new(),
            ltpf: ltpf::Analysis::new(),
            rate: RateControl::new(),
            mdct: mdct::Scratch::new(),
            spectrum: [0.0; MAX_FRAME_SAMPLES],
            quantized: [0; MAX_SPECTRAL_LINES],
            residual: [0; MAX_SPECTRAL_LINES],
            lsbs: LsbBits::new(),
        }
    }

    /// Configuration of the encoder.
    pub const fn config(&self) -> &Config {
        &self.config
    }

    /// Encodes a frame of [`Config::frame_samples`] samples into `frame`,
    /// whose length, from [`MIN_FRAME_BYTES`] to [`MAX_FRAME_BYTES`], is the
    /// size of the encoded frame.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidFrameSize`] when the length of `frame` is out of
    /// range, [`Error::InvalidPcmLength`] when `pcm` holds fewer samples than
    /// a frame.
    pub fn encode<S: Sample>(&mut self, pcm: &[S], frame: &mut [u8]) -> Result<(), Error> {
        self.encode_interleaved(pcm, 1, frame)
    }

    /// Encodes a frame of a channel of interleaved samples: the samples of
    /// the channel are read from `pcm` with a step of `stride`, the number of
    /// interleaved channels, the first sample being `pcm[0]`.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidFrameSize`] when the length of `frame` is out of
    /// range, [`Error::InvalidPcmLength`] when `stride` is zero or `pcm`
    /// holds fewer samples than a frame.
    pub fn encode_interleaved<S: Sample>(
        &mut self,
        pcm: &[S],
        stride: usize,
        frame: &mut [u8],
    ) -> Result<(), Error> {
        let samples = self.config.frame_samples();
        if !(MIN_FRAME_BYTES..=MAX_FRAME_BYTES).contains(&frame.len()) {
            return Err(Error::InvalidFrameSize);
        }
        if stride == 0 || pcm.len() < (samples - 1) * stride + 1 {
            return Err(Error::InvalidPcmLength);
        }
        let history = samples - self.config.mdct_zeros();

        // Input signal scaling and clipping (equations 4 and 5)
        let current = &mut self.input[history..history + samples];
        for (x, s) in current.iter_mut().zip(pcm.iter().step_by(stride)) {
            let v = s.to_scaled();
            *x = if v.is_nan() {
                0.0
            } else {
                v.clamp(MIN_SCALED, MAX_SCALED)
            };
        }

        self.encode_frame(frame);

        // Update of the time buffer (equation 6)
        self.input.copy_within(samples..samples + history, 0);
        Ok(())
    }

    /// Encodes the frame in the input buffer into `frame`.
    fn encode_frame(&mut self, frame: &mut [u8]) {
        let config = self.config;
        let nbytes = frame.len();
        let nbits = 8 * nbytes as i32;
        let samples = config.frame_samples();
        let lines = config.spectral_lines();
        let history = samples - config.mdct_zeros();

        // Low delay MDCT (section 3.3.4)
        let x = &mut self.spectrum[..samples];
        mdct::forward(&config, &self.input[..2 * samples], x, &mut self.mdct);
        let eb = band_energies(&config, x);
        let eb = &eb[..config.band_count()];
        let near_nyquist = near_nyquist(&config, eb);

        // Bandwidth detector (section 3.3.5)
        let bw = bandwidth::detect(&config, eb);

        // Time domain attack detector (section 3.3.6)
        let attack = AttackDetector::enabled(&config, nbytes)
            && self
                .attack
                .run(&config, &self.input[history..history + samples]);

        // Spectral noise shaping (section 3.3.7)
        let scf = sns::analyze(&config, eb, attack);
        let (sns_indices, scfq) = sns::quantize(&scf);
        let xf = &mut x[..lines];
        sns::shape(&config, &scfq, sns::Shaping::Analysis, xf);

        // Temporal noise shaping (section 3.3.8)
        let tns = tns::analyze(&config, bw, near_nyquist, nbytes, xf);
        tns.filter_analysis(xf);
        let xf = &*xf;

        // Long term postfilter (section 3.3.9)
        let pitch = self.ltpf.run(
            &config,
            &self.input[..history + samples],
            history,
            nbytes,
            near_nyquist,
        );

        // Bit budget of the spectrum (equation 106)
        let nbits_spec = nbits
            - config.bandwidth_bits() as i32
            - tns.nbits() as i32
            - ltpf::nbits(pitch) as i32
            - FIXED_SIDE_INFO_BITS
            - spectrum::nbits_ari(&config, nbytes);

        // Global gain estimation and quantization (section 3.3.10)
        let gain_offset = spectrum::global_gain_offset(&config, nbytes);
        let (mut gain_index, min_index) = self.rate.estimate_gain(&config, xf, nbytes, nbits_spec);
        let xq = &mut self.quantized[..lines];
        let mut gain = spectrum::global_gain(gain_index, gain_offset);
        spectrum::quantize(xf, gain, xq);
        let mut estimate = spectrum::estimate_bits(&config, nbytes, nbits_spec, xq);
        self.rate.set_bit_estimate(estimate.nbits);

        // Global gain adjustment and requantization (section 3.3.10.6)
        if let Some(adjusted) =
            spectrum::adjust_global_gain(&config, gain_index, min_index, estimate.nbits, nbits_spec)
        {
            gain_index = adjusted;
            gain = spectrum::global_gain(gain_index, gain_offset);
            spectrum::quantize(xf, gain, xq);
            estimate = spectrum::estimate_bits(&config, nbytes, nbits_spec, xq);
        }

        // Truncation of the spectrum to the bit budget (section 3.3.10.5)
        let lastnz = estimate.lastnz_truncated;
        xq[lastnz..estimate.lastnz].fill(0);
        let lsb_mode =
            if spectrum::lsb_truncation_allowed(&config, nbytes) && estimate.nbits > nbits_spec {
                LsbMode::Truncation
            } else {
                LsbMode::Refinement
            };

        // Residual coding (section 3.3.11)
        let residual_bits = match lsb_mode {
            LsbMode::Truncation => 0,
            LsbMode::Refinement => {
                let max_bits = nbits_spec - estimate.nbits_truncated + 4;
                spectrum::residual_bits(xf, xq, gain, max_bits, &mut self.residual)
            }
        };

        // Noise level estimation (section 3.3.12)
        let noise_level = spectrum::noise_level(&config, bw, xf, xq, gain);

        // Side information (section 3.3.13.3)
        let mut w = Writer::new(frame);
        if config.bandwidth_bits() > 0 {
            w.write_uint(bw.index() as u32, config.bandwidth_bits());
        }
        w.write_uint((lastnz / 2 - 1) as u32, config.lastnz_bits());
        w.write_bit(lsb_mode.bit());
        w.write_uint(gain_index as u32, GAIN_INDEX_BITS);
        tns.write_flags(&mut w);
        w.write_bit(u32::from(pitch.is_some()));
        sns::write(&mut w, &sns_indices);
        ltpf::write(&mut w, pitch);
        w.write_uint(noise_level, NOISE_LEVEL_BITS);

        // Arithmetic coded data (section 3.3.13.4)
        tns.write_coefficients(&mut w);
        spectrum::encode(
            &config, nbytes, &mut w, xq, lastnz, lsb_mode, &mut self.lsbs,
        );

        // Residual data (section 3.3.13.5)
        let nbits_left = nbits - (w.nbits_side() + w.nbits_ari());
        let bits = match lsb_mode {
            LsbMode::Truncation => self.lsbs.as_slice(),
            LsbMode::Refinement => &self.residual[..residual_bits],
        };
        let count = nbits_left.clamp(0, bits.len() as i32) as usize;
        for &bit in &bits[..count] {
            w.write_bit(u32::from(bit));
        }
        w.finish();
    }
}

#[cfg(test)]
mod tests {
    use super::Encoder;
    use crate::config::{Config, FrameDuration, MAX_FRAME_BYTES, MIN_FRAME_BYTES, SampleRate};
    use crate::error::Error;

    const CONFIG: Config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);

    #[test]
    fn rejects_invalid_frame_sizes() {
        let mut encoder = Encoder::new(CONFIG);
        let pcm = [0i16; 480];
        let mut frame = [0u8; MAX_FRAME_BYTES + 1];
        assert_eq!(
            encoder.encode(&pcm, &mut frame[..MIN_FRAME_BYTES - 1]),
            Err(Error::InvalidFrameSize)
        );
        assert_eq!(
            encoder.encode(&pcm, &mut frame),
            Err(Error::InvalidFrameSize)
        );
    }

    #[test]
    fn rejects_short_pcm() {
        let mut encoder = Encoder::new(CONFIG);
        let mut frame = [0u8; 100];
        assert_eq!(
            encoder.encode(&[0i16; 479], &mut frame),
            Err(Error::InvalidPcmLength)
        );
        assert_eq!(
            encoder.encode_interleaved(&[0i16; 958], 2, &mut frame),
            Err(Error::InvalidPcmLength)
        );
        assert_eq!(
            encoder.encode_interleaved(&[0i16; 480], 0, &mut frame),
            Err(Error::InvalidPcmLength)
        );
        assert_eq!(
            encoder.encode_interleaved(&[0i16; 959], 2, &mut frame),
            Ok(())
        );
    }

    #[test]
    fn interleaved_channels_encode_like_separate_channels() {
        let left: [i16; 480] = core::array::from_fn(|n| (n as i16 * 37) % 1000);
        let right: [i16; 480] = core::array::from_fn(|n| 500 - (n as i16 * 11) % 1000);
        let interleaved: [i16; 960] = core::array::from_fn(|n| {
            if n % 2 == 0 {
                left[n / 2]
            } else {
                right[n / 2]
            }
        });

        let (mut left_frame, mut right_frame) = ([0u8; 100], [0u8; 100]);
        Encoder::new(CONFIG).encode(&left, &mut left_frame).unwrap();
        Encoder::new(CONFIG)
            .encode(&right, &mut right_frame)
            .unwrap();

        let mut frame = [0u8; 100];
        Encoder::new(CONFIG)
            .encode_interleaved(&interleaved, 2, &mut frame)
            .unwrap();
        assert_eq!(frame, left_frame);
        Encoder::new(CONFIG)
            .encode_interleaved(&interleaved[1..], 2, &mut frame)
            .unwrap();
        assert_eq!(frame, right_frame);
    }
}
