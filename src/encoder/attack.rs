//! Time domain attack detector (section 3.3.6).

use crate::config::{Config, FrameDuration, SampleRate};

/// Number of samples of the blocks of the downsampled signal.
const BLOCK_SAMPLES: usize = 40;

/// Energy ratio between a block and the temporal envelope detecting an
/// attack (equation 18).
const ATTACK_RATIO: f32 = 8.5;

/// Decay of the temporal envelope from block to block (equation 17).
const ENVELOPE_DECAY: f32 = 0.25;

/// Coefficients of the second order high-pass filter (equation 15).
const HIGH_PASS: [f32; 3] = [0.375, -0.5, 0.125];

/// Number of samples of a frame downsampled to 16 kHz (`M_F`).
const fn downsampled_samples(frame_duration: FrameDuration) -> usize {
    match frame_duration {
        FrameDuration::Ms10 => 160,
        FrameDuration::Ms7_5 => 120,
    }
}

/// State of the attack detector, kept from frame to frame.
#[derive(Clone, Debug)]
pub(super) struct AttackDetector {
    /// Last two samples of the downsampled signal (`x_att`).
    history: [f32; 2],
    /// Temporal envelope of the last block (`A_att`).
    envelope: f32,
    /// Energy of the last block (`E_att`).
    energy: f32,
    /// Block of the attack in the previous frame (`P_att`).
    previous_attack: Option<usize>,
}

impl AttackDetector {
    /// Detector without history.
    pub(super) const fn new() -> Self {
        Self {
            history: [0.0; 2],
            envelope: 0.0,
            energy: 0.0,
            previous_attack: None,
        }
    }

    /// Whether the detector is used for a configuration and frame size
    /// (section 3.3.6.1).
    pub(super) fn enabled(config: &Config, nbytes: usize) -> bool {
        match (config.frame_duration, config.sample_rate) {
            (FrameDuration::Ms10, SampleRate::Hz32000) => nbytes > 80,
            (FrameDuration::Ms10, SampleRate::Hz44100 | SampleRate::Hz48000) => nbytes >= 100,
            (FrameDuration::Ms7_5, SampleRate::Hz32000) => (61..150).contains(&nbytes),
            (FrameDuration::Ms7_5, SampleRate::Hz44100 | SampleRate::Hz48000) => {
                (75..150).contains(&nbytes)
            }
            _ => false,
        }
    }

    /// Runs the detector on the scaled input frame `xs`, returns the attack
    /// flag `F_att` (equations 14 to 19).
    pub(super) fn run(&mut self, config: &Config, xs: &[f32]) -> bool {
        let samples = downsampled_samples(config.frame_duration);
        let ratio = xs.len() / samples;
        let blocks = samples / BLOCK_SAMPLES;

        let mut attack = None;
        let mut energy = 0.0f32;
        for (n, chunk) in xs.chunks_exact(ratio).take(samples).enumerate() {
            // Downsampling (equation 14) and high-pass filtering (equation 15)
            let x: f32 = chunk.iter().sum();
            let filtered =
                HIGH_PASS[0] * x + HIGH_PASS[1] * self.history[0] + HIGH_PASS[2] * self.history[1];
            self.history = [x, self.history[0]];
            energy += filtered * filtered;

            if n % BLOCK_SAMPLES == BLOCK_SAMPLES - 1 {
                // Energy and temporal envelope (equations 16 and 17)
                let envelope = f32::max(ENVELOPE_DECAY * self.envelope, self.energy);
                if energy > ATTACK_RATIO * envelope {
                    attack = Some(n / BLOCK_SAMPLES);
                }
                self.envelope = envelope;
                self.energy = energy;
                energy = 0.0;
            }
        }

        // Attack flag (equation 19)
        let late_attack = self
            .previous_attack
            .is_some_and(|block| block >= blocks / 2);
        self.previous_attack = attack;
        attack.is_some() || late_attack
    }
}

#[cfg(test)]
mod tests {
    use super::AttackDetector;
    use crate::config::{Config, FrameDuration, SampleRate};
    use crate::test_vectors::{ATTACK_7M5, ATTACK_10MS};

    #[test]
    fn detects_the_attacks_of_appendix_c() {
        let frames: [(FrameDuration, &[i16]); 4] = [
            (FrameDuration::Ms10, &ATTACK_10MS[0]),
            (FrameDuration::Ms10, &ATTACK_10MS[1]),
            (FrameDuration::Ms7_5, &ATTACK_7M5[0]),
            (FrameDuration::Ms7_5, &ATTACK_7M5[1]),
        ];
        let mut detectors = [AttackDetector::new(), AttackDetector::new()];
        for (frame_duration, frame) in frames {
            let config = Config::new(SampleRate::Hz48000, frame_duration);
            let detector = &mut detectors[usize::from(frame_duration == FrameDuration::Ms7_5)];
            let xs: Vec<f32> = frame.iter().map(|&v| f32::from(v)).collect();
            assert!(detector.run(&config, &xs));
        }
    }

    #[test]
    fn detects_onsets() {
        let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
        let mut detector = AttackDetector::new();
        let silence = [0.0f32; 480];
        assert!(!detector.run(&config, &silence));

        let mut onset = [0.0f32; 480];
        onset[300..].fill(10_000.0);
        assert!(detector.run(&config, &onset));
    }

    #[test]
    fn late_attacks_carry_over() {
        let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
        let mut detector = AttackDetector::new();
        let mut onset = [0.0f32; 480];
        onset[460..].fill(10_000.0);
        assert!(detector.run(&config, &onset));
        let mut steady = [10_000.0f32; 480];
        steady[0] = 9_000.0;
        assert!(detector.run(&config, &steady));
    }

    #[test]
    fn enabled_only_at_high_bitrates() {
        let config = Config::new(SampleRate::Hz48000, FrameDuration::Ms10);
        assert!(!AttackDetector::enabled(&config, 99));
        assert!(AttackDetector::enabled(&config, 100));
        let config = Config::new(SampleRate::Hz16000, FrameDuration::Ms10);
        assert!(!AttackDetector::enabled(&config, 400));
    }
}
