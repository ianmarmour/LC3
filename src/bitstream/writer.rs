//! Writing of a frame (section 3.3.13).

use super::{
    BYTE_BITS, BYTE_MASK, BYTE_MSB, BackwardPosition, FREQUENCY_BITS, REGISTER_BITS, REGISTER_MASK,
    RENORMALIZATION_RANGE, log2_floor,
};

/// Value of `low` from which a carry may still propagate to the cached byte.
const CARRY_THRESHOLD: u32 = 0x00ff_0000;

/// Writer of an encoded frame.
pub(crate) struct Writer<'a> {
    /// Bytes of the frame.
    bytes: &'a mut [u8],
    /// Position of the next arithmetic coded byte (`bp`).
    bp: usize,
    /// Position of the next backward written bit (`bp_side`, `mask_side`).
    side: BackwardPosition,
    /// Lower bound of the arithmetic coder interval.
    low: u32,
    /// Width of the arithmetic coder interval.
    range: u32,
    /// Byte held back until the carry is known, negative when none.
    cache: i32,
    /// Carry to propagate to the cached byte.
    carry: u32,
    /// Number of 0xff bytes held back after the cached byte.
    carry_count: u32,
}

impl<'a> Writer<'a> {
    /// Writer of the frame `bytes`, zeroed first (section 3.3.13.2 and erratum 16703).
    pub(crate) fn new(bytes: &'a mut [u8]) -> Self {
        bytes.fill(0);
        let side = BackwardPosition::start(bytes.len());
        Self {
            bytes,
            bp: 0,
            side,
            low: 0,
            range: REGISTER_MASK,
            cache: -1,
            carry: 0,
            carry_count: 0,
        }
    }

    /// Writes a bit backward (`write_bit_backward`).
    pub(crate) fn write_bit(&mut self, bit: u32) {
        if let Some(byte) = usize::try_from(self.side.byte)
            .ok()
            .and_then(|i| self.bytes.get_mut(i))
        {
            if bit == 0 {
                *byte &= !(self.side.mask as u8);
            } else {
                *byte |= self.side.mask as u8;
            }
        }
        self.side.advance();
    }

    /// Writes the `numbits` least significant bits of `val` backward,
    /// least significant bit first (`write_uint_backward`).
    pub(crate) fn write_uint(&mut self, mut val: u32, numbits: u32) {
        for _ in 0..numbits {
            self.write_bit(val & 1);
            val >>= 1;
        }
    }

    /// Writes an arithmetic coded byte, ignored past the end of the frame.
    fn put_byte(&mut self, val: u32) {
        if let Some(byte) = self.bytes.get_mut(self.bp) {
            *byte = val as u8;
        }
        self.bp += 1;
    }

    /// Shifts a byte out of the arithmetic coder (`ac_shift`).
    fn ac_shift(&mut self) {
        if self.low < CARRY_THRESHOLD || self.carry == 1 {
            if self.cache >= 0 {
                self.put_byte(self.cache as u32 + self.carry);
            }
            while self.carry_count > 0 {
                self.put_byte((self.carry + BYTE_MASK) & BYTE_MASK);
                self.carry_count -= 1;
            }
            self.cache = (self.low >> (REGISTER_BITS - BYTE_BITS)) as i32;
            self.carry = 0;
        } else {
            self.carry_count += 1;
        }
        self.low = (self.low << BYTE_BITS) & REGISTER_MASK;
    }

    /// Arithmetic encoding of a symbol of cumulative frequency `cum_freq`
    /// and frequency `sym_freq` (`ac_encode`).
    pub(crate) fn ac_encode(&mut self, cum_freq: u16, sym_freq: u16) {
        let r = self.range >> FREQUENCY_BITS;
        self.low += r * u32::from(cum_freq);
        if (self.low >> REGISTER_BITS) != 0 {
            self.carry = 1;
        }
        self.low &= REGISTER_MASK;
        self.range = r * u32::from(sym_freq);
        while self.range < RENORMALIZATION_RANGE {
            self.range <<= BYTE_BITS;
            self.ac_shift();
        }
    }

    /// Number of bits written backward so far (`nbits_side`).
    pub(crate) fn nbits_side(&self) -> i32 {
        self.side.bits_used(self.bytes.len())
    }

    /// Number of bits used by the arithmetic coder so far (`nbits_ari`).
    pub(crate) fn nbits_ari(&self) -> i32 {
        let mut nbits = BYTE_BITS * self.bp as i32 + REGISTER_BITS + 1 - log2_floor(self.range);
        if self.cache >= 0 {
            nbits += BYTE_BITS;
        }
        nbits + BYTE_BITS * self.carry_count as i32
    }

    /// Writes the `numbits` most significant bits of the byte `val` to the
    /// most significant bits of the current arithmetic coded byte
    /// (`write_uint_forward`).
    fn write_uint_forward(&mut self, val: u32, numbits: u32) {
        let Some(byte) = self.bytes.get_mut(self.bp) else {
            return;
        };
        let mut mask = BYTE_MSB;
        for _ in 0..numbits {
            if val & mask == 0 {
                *byte &= !(mask as u8);
            } else {
                *byte |= mask as u8;
            }
            mask >>= 1;
        }
    }

    /// Terminates the arithmetic coded data (`ac_enc_finish`).
    pub(crate) fn finish(mut self) {
        let mut bits: i32 = 1;
        while (self.range >> (REGISTER_BITS - bits)) == 0 {
            bits += 1;
        }
        let mut mask = REGISTER_MASK >> bits;
        let mut val = self.low + mask;
        let over1 = val >> REGISTER_BITS;
        val &= REGISTER_MASK;
        let high = self.low + self.range;
        let over2 = high >> REGISTER_BITS;
        let high = high & REGISTER_MASK;
        val &= !mask;
        if over1 == over2 {
            if val + mask >= high {
                bits += 1;
                mask >>= 1;
                val = ((self.low + mask) & REGISTER_MASK) & !mask;
            }
            if val < self.low {
                self.carry = 1;
            }
        }
        self.low = val;
        while bits > 0 {
            self.ac_shift();
            bits -= BYTE_BITS;
        }
        bits += BYTE_BITS;
        if self.carry_count > 0 {
            self.put_byte(self.cache as u32);
            while self.carry_count > 1 {
                self.put_byte(BYTE_MASK);
                self.carry_count -= 1;
            }
            self.write_uint_forward(BYTE_MASK >> (BYTE_BITS - bits), bits as u32);
        } else {
            self.write_uint_forward(self.cache as u32, bits as u32);
        }
    }
}
