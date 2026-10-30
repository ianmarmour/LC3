//! Reading of a frame (section 3.4.2).

use super::{
    BYTE_BITS, BackwardPosition, FREQUENCY_BITS, REGISTER_BITS, REGISTER_MASK,
    RENORMALIZATION_RANGE, log2_floor,
};

/// Number of bytes read by the arithmetic decoder initialization.
const INITIAL_BYTES: usize = 3;

/// Reader of an encoded frame.
pub(crate) struct Reader<'a> {
    /// Bytes of the frame.
    bytes: &'a [u8],
    /// Position of the next arithmetic coded byte (`bp`).
    bp: usize,
    /// Position of the next backward read bit (`bp_side`, `mask_side`).
    side: BackwardPosition,
    /// Offset of the code value in the arithmetic decoder interval.
    low: u32,
    /// Width of the arithmetic decoder interval.
    range: u32,
}

impl<'a> Reader<'a> {
    /// Reader of the frame `bytes`.
    pub(crate) fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            bp: 0,
            side: BackwardPosition::start(bytes.len()),
            low: 0,
            range: REGISTER_MASK,
        }
    }

    /// Byte of the frame, zero outside of the frame.
    fn byte(&self, index: isize) -> u32 {
        usize::try_from(index)
            .ok()
            .and_then(|i| self.bytes.get(i))
            .map_or(0, |&b| u32::from(b))
    }

    /// Reads a bit backward (`read_bit`).
    pub(crate) fn read_bit(&mut self) -> u32 {
        let bit = u32::from(self.byte(self.side.byte) & self.side.mask != 0);
        self.side.advance();
        bit
    }

    /// Reads a `numbits` bits unsigned integer backward, least significant
    /// bit first (`read_uint`).
    pub(crate) fn read_uint(&mut self, numbits: u32) -> u32 {
        (0..numbits).fold(0, |value, i| value | (self.read_bit() << i))
    }

    /// Reads the next arithmetic coded byte.
    fn next_byte(&mut self) -> u32 {
        let byte = self.byte(self.bp as isize);
        self.bp += 1;
        byte
    }

    /// Initializes the arithmetic decoder (`ac_dec_init`).
    pub(crate) fn ac_start(&mut self) {
        self.low = 0;
        self.range = REGISTER_MASK;
        for _ in 0..INITIAL_BYTES {
            self.low = (self.low << BYTE_BITS) + self.next_byte();
        }
    }

    /// Arithmetic decoding of a symbol (`ac_decode`), `None` on a bit error.
    pub(crate) fn ac_decode(&mut self, cum_freq: &[u16], sym_freq: &[u16]) -> Option<usize> {
        let r = self.range >> FREQUENCY_BITS;
        if self.low >= (r << FREQUENCY_BITS) {
            return None;
        }
        let mut symbol = cum_freq.len() - 1;
        while self.low < r * u32::from(cum_freq[symbol]) {
            symbol -= 1;
        }
        self.low -= r * u32::from(cum_freq[symbol]);
        self.range = r * u32::from(sym_freq[symbol]);
        while self.range < RENORMALIZATION_RANGE {
            self.low = ((self.low << BYTE_BITS) & REGISTER_MASK) + self.next_byte();
            self.range <<= BYTE_BITS;
        }
        Some(symbol)
    }

    /// Whether the arithmetic coded data overruns the backward read data,
    /// a bit error condition.
    pub(crate) fn overrun(&self) -> bool {
        self.bp as isize - self.side.byte > INITIAL_BYTES as isize
    }

    /// Number of bits read backward so far (`nbits_side`).
    pub(crate) fn nbits_side(&self) -> i32 {
        self.side.bits_used(self.bytes.len())
    }

    /// Number of bits consumed by the arithmetic decoder so far (`nbits_ari`).
    pub(crate) fn nbits_ari(&self) -> i32 {
        (self.bp as i32 - INITIAL_BYTES as i32) * BYTE_BITS + REGISTER_BITS + 1
            - log2_floor(self.range)
    }
}
