//! Bitstream writing (section 3.3.13) and reading (section 3.4.2).
//!
//! The side information and the signs and least significant bits of the
//! spectrum are written backward from the end of a frame, while the
//! arithmetic coded data is written forward from its beginning.

mod reader;
mod writer;

pub(crate) use reader::Reader;
pub(crate) use writer::Writer;

/// Resolution of the bit counts of the arithmetic coding tables, which are
/// in units of 1/2048 bit.
pub(crate) const BIT_RESOLUTION: u32 = 2048;

/// Number of bits of the registers of the arithmetic coder.
const REGISTER_BITS: i32 = 24;

/// Mask of the registers of the arithmetic coder.
const REGISTER_MASK: u32 = (1 << REGISTER_BITS) - 1;

/// Number of bits of a byte.
const BYTE_BITS: i32 = 8;

/// Mask of a byte.
const BYTE_MASK: u32 = 0xff;

/// Number of bits of the symbol frequencies, which sum to 1024.
const FREQUENCY_BITS: u32 = 10;

/// Range below which the arithmetic coder renormalizes, shifting out a byte.
const RENORMALIZATION_RANGE: u32 = 0x1_0000;

/// Mask of the most significant bit of a byte.
const BYTE_MSB: u32 = 0x80;

/// Position of a bit written or read backward: a byte index and a bit mask.
#[derive(Clone, Copy, Debug)]
struct BackwardPosition {
    /// Index of the current byte, negative once the beginning of the frame is passed.
    byte: isize,
    /// Mask of the current bit in the byte.
    mask: u32,
}

impl BackwardPosition {
    /// Position of the first bit, the least significant bit of the last byte.
    fn start(len: usize) -> Self {
        Self {
            byte: len as isize - 1,
            mask: 1,
        }
    }

    /// Moves to the next bit, toward the beginning of the frame.
    fn advance(&mut self) {
        if self.mask == BYTE_MSB {
            self.mask = 1;
            self.byte -= 1;
        } else {
            self.mask <<= 1;
        }
    }

    /// Number of bits used so far in a frame of `len` bytes (`nbits_side`).
    fn bits_used(&self, len: usize) -> i32 {
        let nbits = BYTE_BITS * len as i32;
        nbits - (BYTE_BITS * (self.byte as i32 + 1) - self.mask.trailing_zeros() as i32)
    }
}

/// Floor of the base 2 logarithm of the range of the arithmetic coder.
fn log2_floor(range: u32) -> i32 {
    (u32::BITS - 1 - range.leading_zeros()) as i32
}

#[cfg(test)]
mod tests {
    use super::{Reader, Writer};

    #[test]
    fn backward_bits_round_trip() {
        let mut buf = [0u8; 20];
        let mut w = Writer::new(&mut buf);
        w.write_uint(5, 3);
        w.write_uint(0x1234, 13);
        w.write_bit(1);
        assert_eq!(w.nbits_side(), 17);
        w.finish();

        let mut r = Reader::new(&buf);
        assert_eq!(r.read_uint(3), 5);
        assert_eq!(r.read_uint(13), 0x1234);
        assert_eq!(r.read_bit(), 1);
        assert_eq!(r.nbits_side(), 17);
    }

    #[test]
    fn arithmetic_coding_round_trip() {
        let cum_freq: [u16; 4] = [0, 100, 600, 1000];
        let freq: [u16; 4] = [100, 500, 400, 24];
        let symbols = [
            0, 1, 2, 3, 1, 1, 2, 0, 3, 3, 2, 1, 0, 1, 2, 2, 1, 0, 0, 1, 2, 3,
        ];

        let mut buf = [0u8; 40];
        let mut w = Writer::new(&mut buf);
        for &s in &symbols {
            w.ac_encode(cum_freq[s], freq[s]);
        }
        w.finish();

        let mut r = Reader::new(&buf);
        r.ac_start();
        for &s in &symbols {
            assert_eq!(r.ac_decode(&cum_freq, &freq), Some(s));
        }
    }
}
