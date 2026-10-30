//! Packet loss concealment (appendix B): the spectrum of the last good frame
//! is repeated with random signs and a growing attenuation.

use crate::config::MAX_SPECTRAL_LINES;

/// Initial seed of the random sign generator.
const SEED: u32 = 24_607;

/// Multiplier of the random sign generator.
const SEED_MULTIPLIER: u32 = 12_821;

/// Increment of the random sign generator.
const SEED_INCREMENT: u32 = 16_831;

/// Mask of the 16 bits seed of the random sign generator.
const SEED_MASK: u32 = 0xffff;

/// Seeds below this value keep the sign of a spectral value.
const SIGN_THRESHOLD: u32 = 0x8000;

/// Number of consecutive lost frames from which the spectrum is attenuated,
/// with the attenuation applied from frame to frame.
const ATTENUATIONS: [(u32, f32); 2] = [(8, 0.85), (4, 0.9)];

/// State of the concealment, kept from frame to frame.
#[derive(Clone)]
pub(super) struct Concealment {
    /// Seed of the random sign generator.
    seed: u32,
    /// Number of consecutive lost frames.
    lost: u32,
    /// Attenuation of the repeated spectrum (`alpha`).
    attenuation: f32,
    /// Spectrum of the last good frame.
    spectrum: [f32; MAX_SPECTRAL_LINES],
}

impl Concealment {
    /// Concealment state before the first frame.
    pub(super) const fn new() -> Self {
        Self {
            seed: SEED,
            lost: 0,
            attenuation: 1.0,
            spectrum: [0.0; MAX_SPECTRAL_LINES],
        }
    }

    /// Records the spectrum of a good frame.
    pub(super) fn save(&mut self, x: &[f32]) {
        self.lost = 0;
        self.attenuation = 1.0;
        self.spectrum[..x.len()].copy_from_slice(x);
    }

    /// Spectrum `x` of a lost frame (appendix B.3).
    pub(super) fn conceal(&mut self, x: &mut [f32]) {
        self.lost += 1;
        if let Some(&(_, factor)) = ATTENUATIONS.iter().find(|(lost, _)| self.lost >= *lost) {
            self.attenuation *= factor;
        }
        for (v, &saved) in x.iter_mut().zip(&self.spectrum) {
            self.seed = (SEED_INCREMENT + self.seed * SEED_MULTIPLIER) & SEED_MASK;
            let attenuated = self.attenuation * saved;
            *v = if self.seed < SIGN_THRESHOLD {
                attenuated
            } else {
                -attenuated
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Concealment;

    #[test]
    fn repeats_the_last_spectrum_with_random_signs() {
        let mut plc = Concealment::new();
        plc.save(&[1.0, 2.0, 3.0, 4.0]);
        let mut x = [0.0f32; 4];
        plc.conceal(&mut x);
        // Seeds 0x324a, 0xd5d1, 0x9de4 and 0xbd73
        assert_eq!(x, [1.0, -2.0, -3.0, -4.0]);
    }

    #[test]
    fn attenuates_from_the_fourth_lost_frame() {
        let mut plc = Concealment::new();
        plc.save(&[1.0]);
        let mut x = [0.0f32; 1];
        let mut magnitudes = [0.0f32; 9];
        for m in &mut magnitudes {
            plc.conceal(&mut x);
            *m = x[0].abs();
        }
        assert_eq!(magnitudes[..3], [1.0; 3]);
        assert!((magnitudes[3] - 0.9).abs() < 1e-6);
        assert!((magnitudes[6] - 0.9 * 0.9 * 0.9 * 0.9).abs() < 1e-6);
        assert!((magnitudes[7] - 0.9 * 0.9 * 0.9 * 0.9 * 0.85).abs() < 1e-6);

        plc.save(&[1.0]);
        plc.conceal(&mut x);
        assert_eq!(x[0].abs(), 1.0);
    }
}
