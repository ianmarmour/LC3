//! PCM sample formats.

/// Scale of 24 bits samples relative to the 16 bits range, `2^8`.
const SCALE_24_BITS: f32 = 256.0;

/// Scale of 32 bits samples relative to the 16 bits range, `2^16`.
const SCALE_32_BITS: f32 = 65_536.0;

/// Scale of floating point samples relative to the 16 bits range, `2^15`.
const SCALE_FLOAT: f32 = 32_768.0;

/// Smallest sample of the 16 bits range.
pub(crate) const MIN_SCALED: f32 = -32_768.0;

/// Largest sample of the 16 bits range.
pub(crate) const MAX_SCALED: f32 = 32_767.0;

/// A PCM sample format.
///
/// The codec works on samples scaled to the 16 bits range (section 3.3.3 of
/// the specification), this trait converts samples from and to this range.
/// It is implemented for 16 bits ([`i16`]), 24 bits ([`I24`]) and 32 bits
/// ([`i32`]) integers, and for floating point samples from -1.0 to 1.0
/// ([`f32`]).
pub trait Sample: Copy {
    /// Converts the sample to the 16 bits range, keeping its precision.
    fn to_scaled(self) -> f32;

    /// Converts a sample of the 16 bits range, within -32768.0 to 32767.0,
    /// to this format, rounding to the nearest integer (equation 151).
    fn from_scaled(x: f32) -> Self;
}

impl Sample for i16 {
    fn to_scaled(self) -> f32 {
        f32::from(self)
    }

    fn from_scaled(x: f32) -> Self {
        libm::roundf(x) as i16
    }
}

impl Sample for i32 {
    fn to_scaled(self) -> f32 {
        self as f32 / SCALE_32_BITS
    }

    fn from_scaled(x: f32) -> Self {
        libm::roundf(x * SCALE_32_BITS) as i32
    }
}

impl Sample for f32 {
    fn to_scaled(self) -> f32 {
        self * SCALE_FLOAT
    }

    fn from_scaled(x: f32) -> Self {
        x / SCALE_FLOAT
    }
}

/// A 24 bits PCM sample, held in the 24 least significant bits of an `i32`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct I24(pub i32);

impl Sample for I24 {
    fn to_scaled(self) -> f32 {
        self.0 as f32 / SCALE_24_BITS
    }

    fn from_scaled(x: f32) -> Self {
        Self(libm::roundf(x * SCALE_24_BITS) as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::{I24, Sample};

    #[test]
    fn full_scale_of_each_format() {
        assert_eq!(i16::MIN.to_scaled(), -32_768.0);
        assert_eq!(I24(-(1 << 23)).to_scaled(), -32_768.0);
        assert_eq!(i32::MIN.to_scaled(), -32_768.0);
        assert_eq!((-1.0f32).to_scaled(), -32_768.0);
    }

    #[test]
    fn conversions_round_to_nearest() {
        assert_eq!(i16::from_scaled(-4.5), -5);
        assert_eq!(i16::from_scaled(4.5), 5);
        assert_eq!(I24::from_scaled(1.0), I24(256));
        assert_eq!(i32::from_scaled(-1.0), -65_536);
    }
}
