//! The floating-point core (ROM `2E18`-`2F5x`, `333x`-`36FF`).
//!
//! Every routine works on reals stored in RAM (see the [module documentation](super) for the
//! 5-byte format) and is addressed the way the firmware does it: `HL` = address of the first
//! operand *and of the result*, `DE` = address of the second operand.  The arithmetic follows
//! the Z80 code step by step, including its rounding behaviour (round half up on a single
//! guard byte, truncating multiplication) and its habit of leaving scratch values in the
//! operand bytes, because programs can observe those through memory.
//!
//! Naming used in comments: `DEHL` is the 32-bit mantissa (D = most significant) with the
//! implicit leading 1 made explicit in bit 31, `C` is the guard byte below it, `B` carries
//! the sign (only bit 7 matters).

use super::Flags;
use crate::{Cpc, Hardware};

/// ROM `2F53`: the ten constants 10^1 .. 10^13 (five bytes each).
pub const POWERS_OF_TEN: u16 = 0x2F53;

/// The mantissa accumulator `D E H L : C` used by the multiply, divide and add loops.
#[derive(Clone, Copy, Default, Debug)]
pub(crate) struct Acc {
    /// `D:E:H:L`
    pub m: u32,
    /// Guard byte.
    pub c: u8,
}

impl Acc {
    pub fn d(&self) -> u8 { (self.m >> 24) as u8 }
    pub fn e(&self) -> u8 { (self.m >> 16) as u8 }
    pub fn h(&self) -> u8 { (self.m >> 8) as u8 }
    pub fn l(&self) -> u8 { self.m as u8 }
    /// `SLA C; RL L; RL H; RL E; RL D`; returns the bit shifted out of `D`.
    pub fn shl1(&mut self) -> bool {
        let out = self.m >> 31 != 0;
        self.m = (self.m << 1) | (self.c >> 7) as u32;
        self.c <<= 1;
        out
    }
    /// `RR D; RR E; RR H; RR L; RR C` with `carry` rotated into `D`.
    pub fn shr1(&mut self, carry: bool) -> bool {
        let out = self.c & 1 != 0;
        self.c = (self.c >> 1) | ((self.m & 1) as u8) << 7;
        self.m = (self.m >> 1) | (carry as u32) << 31;
        out
    }
}

impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Small helpers
    // ===============================================================================

    fn mant_at(&self, p: u16) -> u32 {
        (self.read(p) as u32) | (self.read(p + 1) as u32) << 8 | (self.read(p + 2) as u32) << 16
            | (self.read(p + 3) as u32) << 24
    }

    // ===============================================================================
    // Copy and conversions                                           (2E18 .. 2EB5)
    // ===============================================================================

    /// `BD3D` (ROM `2E18`): copy a real.  Entry: `HL` = destination, `DE` = source.
    /// Exit: `A` = exponent of the copy, carry set.
    pub fn real_copy(&mut self, dest: u16, src: u16) -> u8 {
        for i in 0..5 {
            let v = self.read(src.wrapping_add(i));
            self.wr8(dest.wrapping_add(i), v);
        }
        self.read(dest.wrapping_add(4))
    }

    /// `BD40` (ROM `2E29`): convert an unsigned 16-bit integer to a real.
    ///
    /// Entry: `value` = HL, `dest` = DE, `a` = sign (bit 7 set = negative).  Exit: `HL` = `dest`.
    pub fn int_to_real(&mut self, value: u16, dest: u16, a: u8) -> u16 {
        let b = a | 0x7F;                    // mask applied to the top byte: keeps/clears the sign bit
        self.wr8(dest, 0);
        self.wr8(dest.wrapping_add(1), 0);
        let (mut hl, mut c) = (value, 0x90u8);
        let a_out;
        if hl >> 8 != 0 {
            a_out = self.int_normalise(&mut hl, &mut c, b);
        } else {
            c = 0;
            hl = hl << 8;                    // H = L, L = 0
            if hl >> 8 == 0 {
                a_out = 0;                   // the value is zero: exponent 0
            } else {
                c = 0x88;
                a_out = self.int_normalise(&mut hl, &mut c, b);
            }
        }
        self.wr8(dest.wrapping_add(2), hl as u8);
        self.wr8(dest.wrapping_add(3), a_out);
        self.wr8(dest.wrapping_add(4), c);
        dest
    }

    /// ROM `2E41`-`2E4B`: shift `hl` left until bit 15 is set, counting the exponent `c`
    /// down; then mask the top byte with `b` (this inserts the sign).
    fn int_normalise(&self, hl: &mut u16, c: &mut u8, b: u8) -> u8 {
        if *hl & 0x8000 == 0 {
            loop {
                *hl <<= 1;
                *c = c.wrapping_sub(1);
                if *hl & 0x8000 != 0 { break; }
            }
        }
        ((*hl >> 8) as u8) & b
    }

    /// `BD43` (ROM `2E55`): convert the 32-bit binary integer at `addr` (little endian) to a
    /// real in place.  `a` bit 7 = sign.  Exit: `HL` = `addr`.
    pub fn bin_to_real(&mut self, addr: u16, a: u8) -> u16 { self.bin_to_real_exp(addr, a, 0xA0) }

    /// `BD94` (ROM `2E5E`): like [`Self::bin_to_real`] but the integer is taken to be scaled by
    /// 2^-8 (exponent 0xA8) – used by the random number generator.
    ///
    /// The ROM routine also takes the register `C` as a guard byte below the 32-bit
    /// integer (it is `LD B,A8 / CALL 36A1` with the caller's `C`); this entry passes 0,
    /// see [`Self::bin_to_real_a8_guard`].
    pub fn bin_to_real_a8(&mut self, addr: u16, a: u8) -> u16 { self.bin_to_real_a8_guard(addr, a, 0) }

    /// `BD94` with the guard byte `C` given: the 40-bit value `(addr..addr+3):guard` scaled
    /// by 2^-8.  (BASIC's number parser uses this to convert 40-bit decimal mantissas.)
    pub fn bin_to_real_a8_guard(&mut self, addr: u16, a: u8, guard: u8) -> u16 {
        self.bin_to_real_exp_guard(addr, a, 0xA8, guard)
    }

    fn bin_to_real_exp(&mut self, addr: u16, sign: u8, exp: u8) -> u16 {
        self.bin_to_real_exp_guard(addr, sign, exp, 0)
    }

    fn bin_to_real_exp_guard(&mut self, addr: u16, sign: u8, exp: u8, guard: u8) -> u16 {
        self.wr8(addr.wrapping_add(4), exp);
        let acc = Acc { m: self.mant_at(addr), c: guard };
        self.real_normalise_store(addr, acc, sign)
    }

    /// `BD46` (ROM `2E66`): convert a real to a 16-bit integer.
    ///
    /// Exit: `carry` = the value fits in 16 bits (as a magnitude), `HL` = the magnitude and
    /// `sign` = the S flag (the real was negative).  Zero gives `HL = 0` with carry set.
    pub fn real_to_int(&mut self, addr: u16) -> (u16, Flags) {
        let exp = self.read(addr.wrapping_add(4));
        if exp == 0 {
            return (0, Flags { carry: true, zero: true, sign: false });
        }
        // A = 0x90 - exp; carry if the number is not too large.
        if exp > 0x90 { return (addr, Flags { carry: false, zero: false, sign: true }); }
        let a = (0x90 - exp).wrapping_add(0x10);
        let (acc, _b) = self.shift_right_from(addr, a);
        // SLA C: bit 7 of the guard byte is the rounding bit; ADC HL,DE adds it (DE is 0 here).
        let rnd = (acc.c >> 7) as u32;
        let hl16 = (acc.m & 0xFFFF) as u32;
        let de16 = (acc.m >> 16) & 0xFFFF;
        let sum = hl16 + de16 + rnd;
        let hl = sum as u16;
        if hl == 0 {
            // SBC A,A with the carry of the addition, then CCF: carry = "no overflow"
            let c = sum > 0xFFFF;
            return (hl, Flags { carry: !c, zero: !c, sign: c });
        }
        let a = self.read(addr.wrapping_add(3));
        (hl, Flags { carry: true, zero: a == 0, sign: a & 0x80 != 0 })
    }

    /// `BD49` (ROM `2E8E`): convert a real to a 32-bit binary integer, rounding to nearest
    /// (halves away from zero).  See [`Self::real_fix`] for the registers.
    pub fn real_to_bin(&mut self, addr: u16) -> (FixResult, Flags) {
        let (r, f) = self.real_fix(addr);
        if !f.carry || !f.sign { return (r, f); }
        // Guard byte >= 0x80: round the magnitude up.
        let r = self.bin_increment(addr, r);
        (r, Flags { carry: true, zero: f.zero, sign: f.sign })
    }

    /// `BD4C` (ROM `2EA1`): truncate a real to a 32-bit binary integer *in place*.
    ///
    /// Exit: `carry` = success; `B` bit 7 = sign of the number; `C` = number of significant
    /// bytes of the result (1..4).  The Z flag reports "the number was an exact integer"; the S
    /// flag is bit 7 of the guard (the discarded fraction), used for rounding.
    pub fn real_fix(&mut self, addr: u16) -> (FixResult, Flags) {
        let exp = self.read(addr.wrapping_add(4));
        if exp == 0 {
            for i in 0..4 { self.wr8(addr.wrapping_add(i), 0); }
            return (FixResult { b: 0, c: 1 }, Flags { carry: true, zero: true, sign: false });
        }
        let a = 0xA0u8.wrapping_sub(exp);                // 0xA0 - exp
        if exp > 0xA0 { return (FixResult { b: 0, c: 0 }, Flags { carry: false, zero: false, sign: true }); }
        let (acc, bsticky) = self.shift_right_from(addr, a);
        let frac = (bsticky != 0) as u8 | acc.c;         // non-zero if anything was shifted out
        self.wr8(addr, acc.l());
        self.wr8(addr.wrapping_add(1), acc.h());
        self.wr8(addr.wrapping_add(2), acc.e());
        let old3 = self.read(addr.wrapping_add(3));
        self.wr8(addr.wrapping_add(3), acc.d());
        let b = old3 & 0x80;
        let mut c = 4u8;
        let mut p = addr.wrapping_add(3);
        loop {
            if self.read(p) != 0 { break; }
            p = p.wrapping_sub(1);
            c -= 1;
            if c == 0 { c = 1; break; }
        }
        (FixResult { b, c }, Flags { carry: true, zero: frac == 0, sign: frac & 0x80 != 0 })
    }

    /// `BD4F` (ROM `2EAC`): floor of a real as a binary integer (see [`Self::real_fix`]).
    pub fn real_int(&mut self, addr: u16) -> (FixResult, Flags) {
        let (r, f) = self.real_fix(addr);
        if !f.carry || f.zero { return (r, f); }
        if r.b & 0x80 == 0 { return (r, Flags { carry: true, zero: false, sign: f.sign }); }
        let r = self.bin_increment(addr, r);
        (r, Flags { carry: true, zero: false, sign: f.sign })
    }

    /// ROM `2E93`: add 1 to the `r.c`-byte magnitude at `addr`; the byte count grows if the
    /// carry runs off the end.
    fn bin_increment(&mut self, addr: u16, mut r: FixResult) -> FixResult {
        let mut p = addr;
        let mut a = r.c;
        loop {
            let v = self.read(p).wrapping_add(1);
            self.wr8(p, v);
            if v != 0 { return r; }
            p = p.wrapping_add(1);
            a = a.wrapping_sub(1);
            if a == 0 {
                let v = self.read(p).wrapping_add(1);
                self.wr8(p, v);
                r.c = r.c.wrapping_add(1);
                return r;
            }
        }
    }

    /// ROM `363D`/`3643`: load the mantissa of the real at `addr` (leading 1 made explicit)
    /// and shift it right `n` bits (at most 33).  Returns the accumulator (`C` = guard byte)
    /// and `B` = the OR of the bytes that passed through `C`.
    pub(crate) fn shift_right_from(&self, addr: u16, n: u8) -> (Acc, u8) {
        let mut n = n.min(0x21);
        let mut acc = Acc { m: self.mant_at(addr) | 0x8000_0000, c: 0 };
        let mut b = 0u8;
        while n >= 8 {
            n -= 8;
            b |= acc.l();
            acc.c = acc.l();
            acc.m >>= 8;
        }
        while n > 0 {
            acc.shr1(false);
            n -= 1;
        }
        (acc, b)
    }

    // ===============================================================================
    // Normalise, round and store                                     (3673 .. 36EE)
    // ===============================================================================

    /// ROM `3673`: shift `acc` left until bit 31 is set, decrementing the exponent `a`.
    /// Returns the new exponent (0 if the mantissa is zero or the exponent ran out).
    ///
    /// Quirk kept from the Z80 code: when the top mantissa byte is zero the routine first
    /// stores the *exponent* in `D` (to free `A`); if it then gives up (zero mantissa or
    /// underflow) that exponent stays in the top byte of the accumulator, and from there it
    /// leaks into the sign byte of the stored result.
    fn real_normalise(&self, acc: &mut Acc, a: u8) -> u8 {
        if acc.d() & 0x80 != 0 { return a; }
        let mut a = a;
        if acc.d() == 0 {
            // L_3678
            acc.m = (acc.m & 0x00FF_FFFF) | (a as u32) << 24;      // LD D,A
            if acc.e() | acc.h() | acc.l() | acc.c == 0 { return 0; }
            // L_367F: shift whole bytes while the top byte is zero
            loop {
                if a < 8 { return 0; }                              // L_369F
                a -= 8;
                if a == 0 { return 0; }
                acc.m = (acc.m << 8) | acc.c as u32;                // D=E, E=H, H=L, L=C
                acc.c = 0;
                if acc.d() == 0 { continue; }
                if acc.d() & 0x80 != 0 { return a; }
                break;
            }
        }
        // L_368F: shift bit by bit
        loop {
            a = a.wrapping_sub(1);
            if a == 0 { return 0; }
            acc.shl1();
            if acc.d() & 0x80 != 0 { return a; }
        }
    }

    /// ROM `36B1`: normalise the accumulator, then round and store (`36BA`).  The exponent
    /// is taken from byte 4 of the destination.  Returns (carry, `HL`).
    pub(crate) fn real_normalise_store(&mut self, ix: u16, mut acc: Acc, b: u8) -> u16 {
        let a = self.read(ix.wrapping_add(4));
        let a = self.real_normalise(&mut acc, a);
        self.wr8(ix.wrapping_add(4), a);
        self.real_round_store(ix, acc, b).1
    }

    /// ROM `36BA`: round on the guard byte (half up) and store the result.  Returns
    /// (carry, `HL`); carry is clear when the exponent overflowed.
    pub(crate) fn real_round_store(&mut self, ix: u16, acc: Acc, b: u8) -> (bool, u16) {
        let mut m = acc.m;
        if acc.c & 0x80 != 0 {
            let (r, wrapped) = m.overflowing_add(1);
            m = r;
            if wrapped {
                // The mantissa was all ones: it becomes 0x80000000 with a larger exponent.
                let e = self.read(ix.wrapping_add(4)).wrapping_add(1);
                self.wr8(ix.wrapping_add(4), e);
                if e == 0 {
                    // 36CD -> 36EE: the Z80 leaves HL = the (wrapped) low word of the mantissa
                    self.real_overflow(ix, b);
                    return (false, 0);
                }
                m = 0x8000_0000;
            }
        }
        self.wr8(ix.wrapping_add(3), (b | 0x7F) & (m >> 24) as u8);
        self.wr8(ix.wrapping_add(2), (m >> 16) as u8);
        self.wr8(ix.wrapping_add(1), (m >> 8) as u8);
        self.wr8(ix, m as u8);
        (true, ix)
    }

    /// ROM `36E6`: the result is zero – only the exponent byte is cleared.
    pub(crate) fn real_zero_result(&mut self, ix: u16) -> u16 {
        self.wr8(ix.wrapping_add(4), 0);
        ix
    }

    /// ROM `36EE`: overflow – store the largest number with the sign in `b`.
    /// (Entry `36EC` is the same with a positive sign.)  Returns `HL`.
    pub(crate) fn real_overflow(&mut self, ix: u16, b: u8) -> u16 {
        self.wr8(ix.wrapping_add(3), b | 0x7F);
        self.wr8(ix.wrapping_add(4), 0xFF);
        self.wr8(ix, 0xFF);
        self.wr8(ix.wrapping_add(1), 0xFF);
        self.wr8(ix.wrapping_add(2), 0xFF);
        ix
    }
}

/// Registers `B` and `C` on exit from the real → binary routines.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FixResult {
    /// `B`: bit 7 = the number was negative.
    pub b: u8,
    /// `C`: number of significant bytes in the binary result.
    pub c: u8,
}

/// Outcome of the exponent range check that precedes multiplication and division.
enum ExpCheck {
    /// The first operand is zero.
    FirstZero,
    /// The result exponent would be below the smallest one.
    Underflow,
    /// The result exponent would exceed 255.
    Overflow,
    /// In range; the value is the provisional result exponent.
    Ok(u8),
}

impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Addition and subtraction                                       (3337 .. 340F)
    // ===============================================================================

    /// `BD58` (ROM `333F`): `HL = HL + DE`.  Carry clear = overflow.
    pub fn real_add(&mut self, hl: u16, de: u16) -> bool { self.real_add_core(hl, de, 0).0 }
    /// `BD5B` (ROM `3337`): `HL = HL - DE`.
    pub fn real_sub(&mut self, hl: u16, de: u16) -> bool { self.real_add_core(hl, de, 1).0 }
    /// `BD5E` (ROM `333B`): `HL = DE - HL`.
    pub fn real_sub_reverse(&mut self, hl: u16, de: u16) -> bool { self.real_add_core(hl, de, 0x80).0 }

    /// ROM `3340`: `mode` 0 = add, 1 = flip the sign of the second operand, `0x80` = flip the
    /// sign of the first.  The operand with the smaller exponent is shifted right to line the
    /// mantissae up (its lost bits feed the guard byte), then the two are added or subtracted.
    ///
    /// Returns `(carry, HL)`.  `HL` is the result address except after an overflow that happens
    /// once the mantissa registers are in use, when the Z80 leaves them in `HL` (the same holds
    /// for [`Self::real_mul_x`] and [`Self::real_div_x`]); callers that chain operations on `HL`
    /// without reloading it, like [`Self::real_pow10`], inherit that quirk.
    pub(crate) fn real_add_core(&mut self, hl: u16, de: u16, mode: u8) -> (bool, u16) {
        let ix = hl;
        let iy = de;
        let mut b = self.read(ix.wrapping_add(3));
        let mut c = self.read(iy.wrapping_add(3));
        if mode != 0 {
            if mode & 0x80 != 0 { b ^= mode } else { c ^= 0x80 }
        }
        let a = self.read(ix.wrapping_add(4));
        let e2 = self.read(iy.wrapping_add(4));
        let (small, big, count);
        if a < e2 {
            // The second operand is bigger: its sign decides, its exponent is the result's.
            std::mem::swap(&mut b, &mut c);
            self.wr8(ix.wrapping_add(4), e2);
            if a == 0 { return (self.real_take_second(ix, iy, b), ix); }
            let diff = e2 - a;
            if diff >= 0x21 { return (self.real_take_second(ix, iy, b), ix); }
            small = ix; big = iy; count = diff;
        } else {
            if e2 == 0 { return (self.real_take_first(ix, b), ix); }
            let diff = a - e2;
            if diff >= 0x21 { return (self.real_take_first(ix, b), ix); }
            small = iy; big = ix; count = diff;
        }
        // L_3387
        let signs_differ = (b ^ c) & 0x80 != 0;
        let (mut acc, _sticky) = self.shift_right_from(small, count);
        let bigm = self.mant_at(big) | 0x8000_0000;
        if !signs_differ {
            let (sum, carry) = acc.m.overflowing_add(bigm);
            acc.m = sum;
            if !carry { return self.real_round_store(ix, acc, b); }
            acc.shr1(true);
            let e = self.read(ix.wrapping_add(4)).wrapping_add(1);
            self.wr8(ix.wrapping_add(4), e);
            if e == 0 { self.real_overflow(ix, b); return (false, acc.m as u16); }
            return self.real_round_store(ix, acc, b);
        }
        // L_33DA: |big| - |small| as a 40-bit quantity (guard byte included)
        let bigv = (bigm as u64) << 8;
        let smallv = ((acc.m as u64) << 8) | acc.c as u64;
        let (diff, borrow) = if bigv >= smallv { (bigv - smallv, false) } else { (smallv - bigv, true) };
        let mut b = b;
        if borrow { b = !b; }
        let acc = Acc { m: (diff >> 8) as u32, c: diff as u8 };
        if acc.d() & 0x80 != 0 { self.real_round_store(ix, acc, b) } else { self.real_normalise_store_c(ix, acc, b) }
    }

    fn real_normalise_store_c(&mut self, ix: u16, acc: Acc, b: u8) -> (bool, u16) {
        let a = self.read(ix.wrapping_add(4));
        let mut acc = acc;
        let a = self.real_normalise(&mut acc, a);
        self.wr8(ix.wrapping_add(4), a);
        self.real_round_store(ix, acc, b)
    }

    /// ROM `33C3`: the result is the second operand (the first is negligible or zero).
    fn real_take_second(&mut self, ix: u16, iy: u16, b: u8) -> bool {
        for i in 0..3 {
            let v = self.read(iy.wrapping_add(i));
            self.wr8(ix.wrapping_add(i), v);
        }
        self.wr8(ix.wrapping_add(3), b);
        true
    }

    /// ROM `33D5`: the result is the first operand (with the sign byte `b`).
    fn real_take_first(&mut self, ix: u16, b: u8) -> bool {
        self.wr8(ix.wrapping_add(3), b);
        true
    }

    // ===============================================================================
    // Multiplication and division                                   (3415 .. 3568)
    // ===============================================================================

    /// ROM `3548`: combine the exponents of `ix` and `iy` (with `c_in` already adjusted by the
    /// caller) and decide whether the result is representable.  Also returns the XOR of the two
    /// sign bytes.  On success the leading 1 of the first operand's mantissa is made explicit
    /// in memory (bit 7 of byte 3), which the mantissa loops rely on.
    fn real_exp_check(&mut self, ix: u16, iy: u16, c_in: u8) -> (ExpCheck, u8) {
        let b = self.read(ix.wrapping_add(3)) ^ self.read(iy.wrapping_add(3));
        let e1 = self.read(ix.wrapping_add(4));
        if e1 == 0 { return (ExpCheck::FirstZero, b); }
        let sum = e1 as u16 + c_in as u16;
        let a = sum as u8;
        let carry = sum > 0xFF;
        if carry == (a & 0x80 != 0) {
            // L_3568: the 9-bit sum is below 0x80 (underflow) or at least 0x180 (overflow)
            return (if a & 0x80 != 0 { ExpCheck::Overflow } else { ExpCheck::Underflow }, b);
        }
        let b3 = self.read(ix.wrapping_add(3)) | 0x80;
        self.wr8(ix.wrapping_add(3), b3);
        (ExpCheck::Ok(a.wrapping_sub(0x7F)), b)
    }

    /// `BD61` (ROM `3415`): `HL = HL * DE`.  Carry clear = overflow.
    pub fn real_mul(&mut self, hl: u16, de: u16) -> bool { self.real_mul_x(hl, de).0 }

    /// [`Self::real_mul`] also returning the exit value of `HL` (see [`Self::real_add_core`]).
    pub(crate) fn real_mul_x(&mut self, hl: u16, de: u16) -> (bool, u16) {
        let (ix, iy) = (hl, de);
        let e2 = self.read(iy.wrapping_add(4));
        if e2 == 0 { self.real_zero_result(ix); return (true, ix); }
        let (check, b) = self.real_exp_check(ix, iy, e2 - 1);
        match check {
            ExpCheck::FirstZero | ExpCheck::Underflow => { self.real_zero_result(ix); (true, ix) }
            ExpCheck::Overflow => { self.real_overflow(ix, b); (false, ix) }
            ExpCheck::Ok(mut a) => {
                let mut acc = self.mul_mantissae(ix, iy);
                if acc.d() & 0x80 == 0 {
                    a = a.wrapping_sub(1);
                    if a == 0 { self.real_zero_result(ix); return (true, ix); }
                    acc.shl1();
                }
                self.wr8(ix.wrapping_add(4), a);
                if a == 0 {
                    // Quirk: an exponent of exactly 0 is treated as an overflow, and HL is
                    // left holding the low word of the product.
                    self.real_overflow(ix, b);
                    return (false, acc.m as u16);
                }
                self.real_round_store(ix, acc, b)
            }
        }
    }

    /// ROM `3450`: 32 x 32 bit multiplication keeping the top 40 bits (the low 24 bits of the
    /// product are discarded one multiplier byte at a time, so the result is truncated).
    fn mul_mantissae(&self, ix: u16, iy: u16) -> Acc {
        let x = self.mant_at(ix);
        let mut acc = Acc { m: 0, c: 0 };
        for k in 0..4u16 {
            let mut a = self.read(iy.wrapping_add(k));
            if k == 3 { a |= 0x80; }
            if a == 0 {
                acc.m >>= 8;                       // L=H, H=E, E=D, D=0
                continue;
            }
            let mut cflag = a & 1 != 0;
            let mut cc = a >> 1;
            for _ in 0..8 {
                let mut add_carry = false;
                if cflag {
                    let (s, c) = acc.m.overflowing_add(x);
                    acc.m = s;
                    add_carry = c;
                }
                let l0 = (acc.m & 1) as u8;
                acc.m = (acc.m >> 1) | (add_carry as u32) << 31;
                cflag = cc & 1 != 0;
                cc = (cc >> 1) | l0 << 7;
            }
            acc.c = cc;
        }
        acc
    }

    /// `BD64` (ROM `349E`): `HL = HL / DE`.  Carry clear = overflow or division by zero (zero
    /// flag set for division by zero).
    pub fn real_div(&mut self, hl: u16, de: u16) -> Flags { self.real_div_x(hl, de).0 }

    /// [`Self::real_div`] also returning the exit value of `HL` (see [`Self::real_add_core`]).
    pub(crate) fn real_div_x(&mut self, hl: u16, de: u16) -> (Flags, u16) {
        let (ix, iy) = (hl, de);
        let e2 = self.read(iy.wrapping_add(4));
        if e2 == 0 {
            // L_3502 (division by zero): the largest number with the dividend's sign
            let b = self.read(ix.wrapping_add(3));
            self.real_overflow(ix, b);
            return (Flags { carry: false, zero: true, sign: false }, hl);
        }
        let (check, b) = self.real_exp_check(ix, iy, 0u8.wrapping_sub(e2));
        let a = match check {
            ExpCheck::FirstZero | ExpCheck::Underflow => {
                self.real_zero_result(ix);
                return (Flags { carry: true, zero: true, sign: false }, ix);
            }
            ExpCheck::Overflow => {
                self.real_overflow(ix, b);
                return (Flags { carry: false, zero: false, sign: true }, hl);
            }
            ExpCheck::Ok(a) => a,
        };
        let divisor = self.mant_at(iy) | 0x8000_0000;
        let mut acc = Acc { m: self.mant_at(ix), c: 0 };
        let mut c = a;
        let carry;
        if acc.m >= divisor {
            if c == 0 {
                self.real_overflow(ix, b);
                return (Flags { carry: false, zero: false, sign: true }, acc.m as u16);
            }
            carry = false;
        } else {
            c = c.wrapping_sub(1);
            carry = acc.m >> 31 != 0;
            acc.m <<= 1;
        }
        let mut carry = carry;
        self.wr8(ix.wrapping_add(4), c);
        let mut q = [0u8; 4];
        for qb in q.iter_mut() { *qb = Self::div_byte(&mut acc.m, divisor, &mut carry); }
        self.wr8(ix.wrapping_add(3), q[0]);
        self.wr8(ix.wrapping_add(2), q[1]);
        self.wr8(ix.wrapping_add(1), q[2]);
        if !carry { carry = acc.m >= divisor; }                    // CALL NC,L_3532
        let guard = if carry { 0xFF } else { 0 };                   // SBC A,A
        let m = (q[0] as u32) << 24 | (q[1] as u32) << 16 | (q[2] as u32) << 8 | q[3] as u32;
        let (ok, h) = self.real_round_store(ix, Acc { m, c: guard }, b);
        (Flags { carry: ok, zero: false, sign: false }, h)
    }

    /// ROM `3507`: produce eight quotient bits by restoring division.  `carry` is the bit
    /// shifted out of the remainder on the previous step (a 33rd bit that forces a subtraction).
    fn div_byte(rem: &mut u32, divisor: u32, carry: &mut bool) -> u8 {
        let mut q = 1u8;
        loop {
            let mut bit = false;
            if *carry || *rem >= divisor {
                *rem = rem.wrapping_sub(divisor);
                bit = true;
            }
            let done = q & 0x80 != 0;
            q = (q << 1) | bit as u8;
            *carry = *rem >> 31 != 0;
            *rem <<= 1;
            if done { return q; }
        }
    }

    // ===============================================================================
    // Comparison, sign, negation, scaling                           (3578 .. 35FB)
    // ===============================================================================

    /// `BD67` (ROM `3578`): multiply by 2^`n` (`n` signed) by adding to the exponent.
    /// Carry clear = overflow (the Z80 then falls into the compare routine, so its other flags
    /// and `A` are meaningless).  Note: a zero is *not* special-cased.
    pub fn real_mul_pow2(&mut self, addr: u16, n: u8) -> bool {
        let e = self.read(addr.wrapping_add(4));
        if n & 0x80 == 0 {
            let (r, c) = e.overflowing_add(n);
            self.wr8(addr.wrapping_add(4), r);
            if !c { return true; }
            let b = self.read(addr.wrapping_add(3));
            self.real_overflow(addr, b);
            return false;
        }
        let (r, c) = e.overflowing_add(n);
        self.wr8(addr.wrapping_add(4), if c { r } else { 0 });
        true
    }

    /// `BD6A` (ROM `359A`): compare the reals at `hl` and `de`.  `A` = `0xFF` (carry) if the
    /// first is smaller, 0 (zero) if equal, 1 if larger.
    pub fn real_compare(&self, hl: u16, de: u16) -> (u8, Flags) {
        let (ix, iy) = (hl, de);
        let e1 = self.read(ix.wrapping_add(4));
        let e2 = self.read(iy.wrapping_add(4));
        let s1 = self.read(ix.wrapping_add(3));
        let s2 = self.read(iy.wrapping_add(3));
        let a;
        if e1 < e2 {
            a = !s2;
        } else if e1 != e2 {
            a = s1;
        } else {
            if e1 == 0 { return (0, Flags { carry: false, zero: true, sign: false }); }
            if (s1 ^ s2) & 0x80 != 0 {
                a = s1;
            } else {
                let mut r = None;
                for k in (0..4u16).rev() {
                    let (x, y) = (self.read(ix.wrapping_add(k)), self.read(iy.wrapping_add(k)));
                    if x != y { r = Some(if x < y { 0xFFu8 } else { 0 } ^ s2); break; }
                }
                match r {
                    None => return (0, Flags { carry: false, zero: true, sign: false }),
                    Some(v) => a = v,
                }
            }
        }
        // L_35D8
        if a & 0x80 != 0 { (0xFF, Flags { carry: true, zero: false, sign: true }) }
        else { (1, Flags::default()) }
    }

    /// `BD70` (ROM `35E8`): sign of a real: `A` = 0 (zero flag), 1, or `0xFF` (carry).
    pub fn real_sign(&self, addr: u16) -> (u8, Flags) {
        if self.read(addr.wrapping_add(4)) == 0 { return (0, Flags { carry: false, zero: true, sign: false }); }
        if self.read(addr.wrapping_add(3)) & 0x80 != 0 { (0xFF, Flags { carry: true, zero: false, sign: true }) }
        else { (1, Flags::default()) }
    }

    /// `BD6D` (ROM `35F8`): negate a real by flipping its sign bit (a zero is not special-cased).
    pub fn real_negate(&mut self, addr: u16) {
        let v = self.read(addr.wrapping_add(3)) ^ 0x80;
        self.wr8(addr.wrapping_add(3), v);
    }
}
