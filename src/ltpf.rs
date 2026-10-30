//! Long Term Postfilter (sections 3.3.9 and 3.4.9).
//!
//! The encoder estimates the pitch of the signal and decides whether the
//! decoder applies a pitch-based postfilter, which attenuates the
//! quantization noise between the harmonics.

mod analysis;
mod tables;

pub(crate) use analysis::Analysis;

use crate::bitstream::{Reader, Writer};
use crate::config::{Config, FrameDuration};

/// Number of bits of the pitch index.
const PITCH_INDEX_BITS: u32 = 9;

/// Smallest integer pitch lag at 12.8 kHz.
const MIN_PITCH: i32 = 32;

/// Smallest integer pitch lag at 12.8 kHz coded with a resolution of 1/2.
const HALF_RESOLUTION_PITCH: i32 = 127;

/// Smallest integer pitch lag at 12.8 kHz coded without fractional part.
const INTEGER_RESOLUTION_PITCH: i32 = 157;

/// Pitch index of the lags coded with a resolution of 1/2.
const HALF_RESOLUTION_INDEX: i32 = 380;

/// Pitch index of the lags coded without fractional part.
const INTEGER_RESOLUTION_INDEX: i32 = 440;

/// Coded pitch lag at 12.8 kHz (`pitch_index`, equations 102 and 139 to 140).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct PitchIndex(u16);

impl PitchIndex {
    /// Index of the pitch lag `pitch_int + pitch_fr / 4` (equation 102).
    fn from_lag(pitch_int: i32, pitch_fr: i32) -> Self {
        let index = if pitch_int >= INTEGER_RESOLUTION_PITCH {
            pitch_int + (INTEGER_RESOLUTION_INDEX - INTEGER_RESOLUTION_PITCH)
        } else if pitch_int >= HALF_RESOLUTION_PITCH {
            2 * pitch_int + pitch_fr / 2 + (HALF_RESOLUTION_INDEX - 2 * HALF_RESOLUTION_PITCH)
        } else {
            4 * pitch_int + pitch_fr - 4 * MIN_PITCH
        };
        Self(index as u16)
    }

    /// Pitch lag at 12.8 kHz of the index, `pitch_int + pitch_fr / 4`
    /// (equations 139 to 141).
    fn lag(self) -> f32 {
        let index = i32::from(self.0);
        let (pitch_int, pitch_fr) = if index >= INTEGER_RESOLUTION_INDEX {
            (
                index - (INTEGER_RESOLUTION_INDEX - INTEGER_RESOLUTION_PITCH),
                0,
            )
        } else if index >= HALF_RESOLUTION_INDEX {
            let pitch_int = index / 2 - (HALF_RESOLUTION_INDEX / 2 - HALF_RESOLUTION_PITCH);
            (
                pitch_int,
                2 * index - 4 * pitch_int - (2 * HALF_RESOLUTION_INDEX - 4 * HALF_RESOLUTION_PITCH),
            )
        } else {
            let pitch_int = index / 4 + MIN_PITCH;
            (pitch_int, index - 4 * pitch_int + 4 * MIN_PITCH)
        };
        pitch_int as f32 + pitch_fr as f32 / 4.0
    }
}

/// Pitch information of a frame, transmitted when the signal is periodic
/// (`pitch_present`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Pitch {
    /// Coded pitch lag.
    pub(crate) index: PitchIndex,
    /// Whether the decoder applies the postfilter (`ltpf_active`).
    pub(crate) active: bool,
}

/// Number of bits of the LTPF data of a frame (equation 95).
pub(crate) const fn nbits(pitch: Option<Pitch>) -> u32 {
    if pitch.is_some() {
        1 + 1 + PITCH_INDEX_BITS
    } else {
        1
    }
}

/// Writes the LTPF data that follows the SNS data in the side information.
pub(crate) fn write(w: &mut Writer, pitch: Option<Pitch>) {
    if let Some(pitch) = pitch {
        w.write_uint(u32::from(pitch.active), 1);
        w.write_uint(u32::from(pitch.index.0), PITCH_INDEX_BITS);
    }
}

/// Reads the LTPF data that follows the SNS data in the side information,
/// `present` being the pitch present flag.
pub(crate) fn read(r: &mut Reader, present: bool) -> Option<Pitch> {
    present.then(|| {
        let active = r.read_uint(1) == 1;
        let index = PitchIndex(r.read_uint(PITCH_INDEX_BITS) as u16);
        Pitch { index, active }
    })
}

/// Index of the postfilter gain at a bitrate (section 3.4.9.4), `None` when
/// the bitrate is too high for the postfilter to be used.
pub(crate) fn gain_index(config: &Config, nbytes: usize) -> Option<usize> {
    let nbits = 8 * nbytes as i32;
    let nbits_10ms = match config.frame_duration {
        FrameDuration::Ms10 => nbits,
        // round(nbits * 10 / 7.5), never a tie as nbits is a multiple of 8
        FrameDuration::Ms7_5 => (nbits * 4 + 1) / 3,
    };
    let offset = config.sample_rate.index() as i32 * 80;
    [320, 400, 480, 560]
        .into_iter()
        .position(|threshold| nbits_10ms < threshold + offset)
}

#[cfg(test)]
mod tests {
    use super::PitchIndex;

    #[test]
    fn pitch_index_round_trip() {
        for (pitch_int, pitch_fr) in [
            (32, 0),
            (32, 3),
            (100, 1),
            (126, 3),
            (127, 0),
            (140, 2),
            (156, 2),
            (157, 0),
            (228, 0),
        ] {
            let index = PitchIndex::from_lag(pitch_int, pitch_fr);
            assert_eq!(index.lag(), pitch_int as f32 + pitch_fr as f32 / 4.0);
        }
        assert_eq!(PitchIndex::from_lag(32, 0).0, 0);
        assert_eq!(PitchIndex::from_lag(228, 0).0, 511);
    }
}
