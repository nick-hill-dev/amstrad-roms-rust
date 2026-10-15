//! Powers of ten, decimal preparation, random numbers, logarithms, exponentials, powers and
//! trigonometry (ROM `2EB6`-`2FFF`, `300F`-`32FC`).
//!
//! The transcendental functions are classic table-driven routines.  A few tiny helpers recur:
//!
//! * a **series evaluator** (`32A9` / `32AC`, [`Cpc::real_series`]) that reads its coefficients
//!   from a table placed *inline after the `CALL`* (the Z80 code pops its own return address to
//!   find it and returns past the table); here the table address is simply a parameter;
//! * the scratch reals `T1`, `T2`, `T3` in the system variable area, which hold intermediate
//!   values – they are observable in memory, so they are written exactly as the ROM does;
//! * "range reduction" ([`Cpc::real_reduce`], ROM `32D5`).

use super::real::Acc;
use crate::sysvars::*;
use crate::{Cpc, Hardware};

/// ROM `3332`: the constant 1.0.
const ONE: u16 = 0x3332;
/// ROM `30CC`: the constant 0.5.
const HALF: u16 = 0x30CC;
/// ROM `2F53`: 10^1; the table holds 10^1 .. 10^13.
const TEN: u16 = 0x2F53;
/// ROM `2F8F`: 10^13.
const TEN_13: u16 = 0x2F8F;

/// Result of [`Cpc::real_decimal_prepare`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DecimalPrep {
    /// `HL`: address of the most significant byte of the binary mantissa (kept in place at the
    /// start of the real).
    pub hl: u16,
    /// `B`: sign – 1 for positive, `0xFF` for negative (0 for zero).
    pub b: u8,
    /// `C`: number of significant bytes of the mantissa.
    pub c: u8,
    /// `E`: decimal exponent (the number is `mantissa * 10^(E - 9)`, mantissa in 100000000..).
    pub e: u8,
    /// `D`: sign extension of the binary exponent – quirk, leaked from the estimate.
    pub d: u8,
}

impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Powers of ten and decimal preparation                          (2EB6 .. 2F52)
    // ===============================================================================

    /// `BD52` (ROM `2EB6`): prepare a real for decimal printing.
    ///
    /// The number is made positive, scaled by powers of ten into the range
    /// ~3,125,000 .. ~1,000,000,000 (so that it has nine significant digits as an integer) and
    /// converted to a binary integer *in place*; the decimal exponent needed to undo the scaling
    /// is returned in `E`.
    pub fn real_decimal_prepare(&mut self, hl: u16) -> DecimalPrep {
        let (b, f) = self.real_sign(hl);
        if f.zero {
            // L_2F0E: a zero – only the first byte is cleared
            self.wr8(hl, 0);
            return DecimalPrep { hl, b: 0, c: 1, e: 0, d: 0 };
        }
        if b == 0xFF { self.real_negate(hl); }                       // CALL M,35FB
        // estimate = 77 * (exponent - 0x80) / 256 - 9  (about log10(2) * exponent)
        let exp = self.read(hl.wrapping_add(4));
        let diff = exp.wrapping_sub(0x80);
        let d = if exp < 0x80 { 0xFF } else { 0 };                   // SBC A,A
        let de = ((d as u16) << 8) | diff as u16;
        let mut hl16 = de;
        for _ in 0..3 { hl16 = hl16.wrapping_add(hl16); }
        hl16 = hl16.wrapping_add(de);
        hl16 = hl16.wrapping_add(hl16);
        hl16 = hl16.wrapping_add(de);
        hl16 = hl16.wrapping_add(hl16);
        hl16 = hl16.wrapping_add(hl16);
        hl16 = hl16.wrapping_add(de);
        let est = ((hl16 >> 8) as u8).wrapping_sub(9);
        let mut e = est;
        if est != 0 { self.real_scale_pow10(hl, est); }               // CALL NZ,L_2F1F
        // Scale until 2F13 <= x <= 2F18 (roughly 3.1e6 .. 1e9).
        loop {
            let (_, f) = self.real_compare(hl, 0x2F13);
            if f.zero { break; }
            if f.carry {
                self.real_mul(hl, TEN);                                // L_3412
                e = e.wrapping_sub(1);
                continue;
            }
            // x > low limit: divide down while it exceeds the high limit
            loop {
                let (_, f) = self.real_compare(hl, 0x2F18);
                if f.carry { break; }
                self.real_div(hl, TEN);                                // L_349B
                e = e.wrapping_add(1);
            }
            break;
        }
        let (fix, _f) = self.real_to_bin(hl);
        let top = hl.wrapping_add(fix.c as u16).wrapping_sub(1);
        DecimalPrep { hl: top, b, c: fix.c, e, d }
    }

    /// `BD55` (ROM `2F1D`): multiply the real at `hl` by 10^`a` (`a` signed; negative divides).
    /// Carry = no overflow.
    pub fn real_pow10(&mut self, hl: u16, a: u8) -> bool {
        self.real_scale_pow10(hl, a.wrapping_neg())
    }

    /// ROM `2F1F`: multiply `hl` by 10^(-`a`).  Powers above 12 are applied in steps of 10^13.
    pub(crate) fn real_scale_pow10(&mut self, hl: u16, a: u8) -> bool {
        if a == 0 { return true; }
        let c = a;
        let mut hl = hl;
        let mut n = if a & 0x80 != 0 { a.wrapping_neg() } else { a };
        loop {
            // L_2F3E: locate the table entry
            let (de, last) = if n >= 13 {
                n -= 13;
                (TEN_13, n == 0)
            } else {
                (TEN + 5 * (n as u16 - 1), true)
            };
            // L_2F36: positive c divides, negative multiplies
            // HL is not reloaded between steps: after an overflow the Z80 continues with the
            // register contents the failed operation left behind.
            let (ok, h) = if c & 0x80 == 0 {
                let (f, h) = self.real_div_x(hl, de);
                (f.carry, h)
            } else {
                self.real_mul_x(hl, de)
            };
            hl = h;
            if last { return ok; }
        }
    }

    // ===============================================================================
    // Random numbers                                                  (2F94 .. 2FFA)
    // ===============================================================================

    /// `BD97` (ROM `2F94`): reset the seed to its initial value.
    pub fn random_reset(&mut self) {
        self.wr16(MATHS_SEED + 2, 0x8965);
        self.wr16(MATHS_SEED, 0x6C07);
    }

    /// `BD9A` (ROM `2FA1`): reset the seed, then fold the low four bytes of the real at `hl` into it.
    pub fn random_seed_from_real(&mut self, hl: u16) {
        self.random_reset();
        let (_, f) = self.real_sign(hl);
        if f.zero { return; }
        for i in 0..4u16 {
            let v = self.rd8(MATHS_SEED + i) ^ self.read(hl.wrapping_add(i));
            self.wr8(MATHS_SEED + i, v);
        }
    }

    /// `BD9D` (ROM `2FB7`): `RND` – advance the generator and return the new number as a real
    /// in [0, 1) at `hl` (the ROM simply falls through into [`Self::random_to_real`]).
    pub fn random_new(&mut self, hl: u16) -> u16 {
        self.random_next();
        self.random_to_real(hl)
    }

    /// The generator step of `BD9D`: advance the 32-bit linear congruential generator.
    ///
    /// With `seed = hi:lo` (16-bit halves) the ROM computes the low 32 bits of
    /// `lo*0x8965 + (lo*0x8965 hi) ...`; the code below transcribes it step for step.
    pub fn random_next(&mut self) {
        let mul = |a: u16, b: u16| -> (u16, u16) {
            let p = a as u32 * b as u32;
            ((p >> 16) as u16, p as u16)                    // (DE high, HL low)
        };
        let lo = self.rd16(MATHS_SEED);
        let hi = self.rd16(MATHS_SEED + 2);
        let (_h1, l1) = mul(hi, 0x6C07);
        let (h2, l2) = mul(lo, 0x8965);
        let (_h3, l3) = mul(hi, 0x8965);
        let (new_lo, c1) = l2.overflowing_add(0x8965);
        self.wr16(MATHS_SEED, new_lo);
        let t = l3 as u32 + 0x6C07 + c1 as u32;
        let new_hi = (t as u16).wrapping_add(h2).wrapping_add(l1);
        self.wr16(MATHS_SEED + 2, new_hi);
    }

    /// `BDA0` (ROM `2FE6`): the last random number again – turn the current seed into a real in
    /// [0, 1) at `hl`.
    pub fn random_to_real(&mut self, hl: u16) -> u16 {
        let m = self.rd16(MATHS_SEED) as u32 | (self.rd16(MATHS_SEED + 2) as u32) << 16;
        self.wr8(hl.wrapping_add(4), 0x80);
        self.real_normalise_store(hl, Acc { m, c: 0 }, 0)
    }

    // ===============================================================================
    // Helpers                                                          (3307 .. 3330)
    // ===============================================================================

    /// ROM `3328`: set the real at `hl` to 1.0 (carry set).
    pub fn real_set_one(&mut self, hl: u16) -> bool {
        self.real_copy(hl, ONE);
        true
    }

    /// ROM `3307`: is the exponent of the real at `hl` above `b` (both relative to 0x80, as
    /// signed bytes)?  Returns `(carry, a)`: carry set if so; `a` is the relative exponent.
    fn real_exp_above(&self, hl: u16, b: u8) -> (bool, u8) {
        let e = self.read(hl.wrapping_add(4));
        if e == 0 { return (false, 0); }
        let a = e.wrapping_sub(0x80);
        if a & 0x80 == 0 { return (true, a); }
        if a == b { return (false, a); }
        (a > b, a)
    }

    /// ROM `331D`: `T2 := x; x := x * x`.  Returns the multiplication's carry.
    fn real_square_keep(&mut self, x: u16) -> bool {
        self.real_copy(MATHS_T2, x);
        self.real_mul(x, MATHS_T2)
    }

    /// ROM `32A9` / `32AC`: evaluate a polynomial by Horner's rule.
    ///
    /// `table` is the ROM address of a coefficient table (`count`, then `count` 5-byte reals).
    /// The variable is `T3`, which is first set to `x*x` (`squared`, entry `32A9`; the original
    /// is saved in `T2`) or to `x` itself (entry `32AC`).  The result replaces `x`.
    ///
    /// `x` may be any address; the ROM's callers sometimes evaluate in `T1` instead of the
    /// argument.  Returns nothing; the Z80 returns to the instruction after the table.
    pub(crate) fn real_series(&mut self, x: u16, table: u16, squared: bool) {
        if squared { self.real_square_keep(x); }
        self.real_copy(MATHS_T3, x);
        let n = self.read(table);
        let mut p = table.wrapping_add(1);
        self.real_copy(x, p);                                   // x := c0
        let mut b = n;
        loop {
            p = p.wrapping_add(5);
            b = b.wrapping_sub(1);
            if b == 0 { return; }
            self.real_mul(x, MATHS_T3);
            self.real_add(x, p);
        }
    }

    /// ROM `32D5` (`32D4` with `round == false`): range reduction.
    ///
    /// `x := x * [k]`, plus 0.5 if `round`; then the integer nearest `x` is split off:
    /// afterwards `x` holds the remainder and the return value holds the integer's magnitude
    /// and sign.  `None` if the integer does not fit in 16 bits.
    pub(crate) fn real_reduce(&mut self, x: u16, k: u16, round: bool) -> Option<(u16, u8)> {
        self.real_mul(x, k);
        if round { self.real_add(x, HALF); }
        let (n, f) = self.real_to_int(x);
        if !f.carry { return None; }
        // `A` as BD46 leaves it: the sign byte of x*k, or 0 when the integer part is zero.
        let a = if n == 0 { 0 } else { self.read(x.wrapping_add(3)) };
        self.int_to_real(n, MATHS_T2, a);
        self.real_sub(x, MATHS_T2);
        Some((n, a))
    }

    // ===============================================================================
    // Logarithms and exponentials                                       (300F .. 3182)
    // ===============================================================================

    /// `BD82` (ROM `300F`): common logarithm.  Carry clear if `x <= 0`.
    pub fn real_log10(&mut self, hl: u16) -> bool { self.real_log_with(hl, 0x308B) }

    /// `BD7F` (ROM `3014`): natural logarithm.  Carry clear if `x <= 0`.
    pub fn real_ln(&mut self, hl: u16) -> bool { self.real_log_with(hl, 0x3086) }

    /// ROM `3017`: `log(x) = (e + log2(m)) * [scale]` where `x = m * 2^e`, `m` in 0.7071..1.4142.
    fn real_log_with(&mut self, hl: u16, scale: u16) -> bool {
        let (s, _f) = self.real_sign(hl);
        if s != 1 { return false; }
        // exponent of x relative to 0x80, then force x's exponent to 0x80 (0.5 <= m < 1)
        let mut e = self.read(hl.wrapping_add(4)).wrapping_sub(0x80);
        self.wr8(hl.wrapping_add(4), 0x80);
        let (_, f) = self.real_compare(hl, 0x3081);               // sqrt(0.5)
        if f.carry {
            let v = self.rd8(hl.wrapping_add(4)).wrapping_add(1);
            self.wr8(hl.wrapping_add(4), v);
            e = e.wrapping_sub(1);
        }
        // s = (m - 1) / (m + 1), via T3
        self.real_copy(MATHS_T3, hl);
        self.real_add(MATHS_T3, ONE);
        self.real_sub(hl, ONE);
        self.real_div(hl, MATHS_T3);
        self.real_series(hl, 0x304F, true);
        self.real_mul(hl, MATHS_T2);                              // times s
        // add the (signed) binary exponent
        let mag = if e & 0x80 != 0 { e.wrapping_neg() } else { e };
        self.int_to_real(mag as u16, MATHS_T2, e);
        self.real_add(hl, MATHS_T2);
        self.real_mul(hl, scale)
    }

    /// `BD85` (ROM `3090`): e^x.  Overflow gives the largest number (carry clear), underflow
    /// zero.
    ///
    /// `x*log2(e) = n + f`; `2^f` is computed as `(P1 + f*P2) / (P1 - f*P2)` with two small
    /// polynomials in `f*f`, doubled and halved: the result is `2^n * 2^f`.
    pub fn real_exp(&mut self, hl: u16) -> bool {
        let (above, _) = self.real_exp_above(hl, 0xE1);
        if !above { return self.real_set_one(hl); }                       // e^tiny = 1
        let (a, _) = self.real_compare(hl, 0x3100);
        if a & 0x80 == 0 { self.real_overflow(hl, 0); return false; }     // x >= 88.72
        let (a, _) = self.real_compare(hl, 0x3105);
        if a & 0x80 != 0 { self.real_zero_result(hl); return true; }      // x <= -88.72
        let Some((nmag, nsign)) = self.real_reduce(hl, 0x30FB, false) else { return false };
        let n = if nsign & 0x80 != 0 { (nmag as u8).wrapping_neg() } else { nmag as u8 };
        self.real_square_keep(hl);                                        // T2 = f, x = f*f
        self.real_copy(MATHS_T1, hl);                                     // T1 = f*f
        self.real_series(MATHS_T1, 0x30C1, false);                        // T1 = P1(f*f)
        self.real_series(hl, 0x30D5, false);                              // x  = P2(f*f)
        self.real_mul(hl, MATHS_T2);                                      // x  = f * P2
        self.real_sub(MATHS_T1, hl);                                      // T1 = P1 - f*P2
        self.real_div(hl, MATHS_T1);                                      // x  = f*P2 / (P1 - f*P2)
        self.real_add(hl, HALF);
        let e = self.rd8(hl.wrapping_add(4)).wrapping_add(1);             // times 2
        self.wr8(hl.wrapping_add(4), e);
        self.real_mul_pow2(hl, n)                                         // times 2^n
    }

    /// `BD79` (ROM `310A`): square root, computed as `x ^ 0.5`.
    pub fn real_sqrt(&mut self, hl: u16) -> bool { self.real_power(hl, HALF) }

    /// ROM `32FD`: `x := 1 / x` (via `T3`).
    fn real_reciprocal(&mut self, x: u16) -> bool {
        self.real_copy(MATHS_T3, x);
        self.real_set_one(x);
        self.real_div(x, MATHS_T3).carry
    }

    /// ROM `3182`: classify the exponent `y` of a power.  Returns `(small_int, a, b, sign)`:
    /// `small_int` (carry) – `y` is an integer in 0..38 (then `a` is its magnitude);
    /// `b` – the sign byte of the base, kept only if `y` is odd (otherwise 0), and
    /// `sign` – the S flag the Z80 leaves: set when the base is negative and `y` is not an
    /// integer (an error, the result would be complex).
    fn power_classify(&mut self, y: u16, b: u8) -> (bool, u8, u8, bool) {
        self.real_copy(MATHS_T3, y);
        let (fix, f) = self.real_fix(MATHS_T3);
        if !f.carry || !f.zero {
            return (false, b, b, b & 0x80 != 0);                 // L_318F
        }
        let low = self.rd8(MATHS_T3);
        let b = if low & 1 != 0 { b } else { 0 };                // keep the sign only for odd y
        if fix.c >= 2 { return (false, 0, b, false); }
        if low < 0x27 { return (true, low, b, false); }
        (false, 0, b, false)
    }

    /// `BD7C` (ROM `310D`): `x ^ y`.  `hl` = x, `de` = y; the result replaces x.
    ///
    /// Small integer exponents use repeated squaring; everything else goes through
    /// `exp(y * ln x)`.  A negative base with a non-integer exponent fails (carry clear, A=1).
    pub fn real_power(&mut self, hl: u16, de: u16) -> bool {
        let (ay, fy) = self.real_sign(de);
        if fy.zero { return self.real_set_one(hl); }                  // x^0 = 1
        let (ax, fx) = self.real_sign(hl);
        if fx.zero {                                                  // L_3140
            if ay & 0x80 == 0 { return true; }
            self.real_overflow(hl, 0);
            return false;
        }
        if ax == 0xFF { self.real_negate(hl); }
        let (small, n, b, sign) = self.power_classify(de, ax);
        if !small {
            if sign { return false; }                                 // L_3148
            if !self.real_ln(hl) { return self.power_finish(hl, b, false); }
            let ok = self.real_mul(hl, de);
            let ok = if ok { self.real_exp(hl) } else { false };
            return self.power_finish(hl, b, ok);
        }
        // L_314B: integer exponent 0..38 by square-and-multiply, most significant bit first
        let (mut a, mut cin) = (n, 1u8);
        loop {
            let cout = a >> 7;
            a = (a << 1) | cin;
            cin = 0;
            if cout != 0 { break; }
        }
        self.real_copy(MATHS_T1, hl);
        loop {
            let bit = a & 0x80 != 0;
            a <<= 1;
            if a == 0 { break; }
            if !self.real_square_keep(hl) { return self.power_overflow(hl, ay, b); }
            if bit && !self.real_mul(hl, MATHS_T1) { return self.power_overflow(hl, ay, b); }
        }
        let mut ok = true;
        // ROM bug, kept: the final sign flip (L_35FB) acts on the real IX points to.  After an
        // exponent of exactly +1 (no multiplication, no division) IX still points to the scratch
        // copy of the exponent, so the flip hits T3 and the result stays positive.
        let mut flip_at = hl;
        if ay & 0x80 != 0 { ok = self.real_reciprocal(hl); }         // negative exponent
        else if n == 1 { flip_at = MATHS_T3; }
        if !ok { return false; }
        if b & 0x80 != 0 { self.real_negate(flip_at); }
        true
    }

    /// ROM `3179`: overflow while powering: a negative exponent underflows to 0, a positive
    /// one gives the largest number.
    fn power_overflow(&mut self, hl: u16, ay: u8, b: u8) -> bool {
        if ay & 0x80 != 0 { self.real_zero_result(hl); true } else { self.real_overflow(hl, b); false }
    }

    /// ROM `3137`: negate the result if the base was negative and the exponent odd.
    fn power_finish(&mut self, hl: u16, b: u8, ok: bool) -> bool {
        if !ok { return false; }
        if b & 0x80 != 0 { self.real_negate(hl); }
        true
    }

    // ===============================================================================
    // Trigonometry                                                      (31A3 .. 32A8)
    // ===============================================================================

    /// `BD76` (ROM `31A3`): store pi at `hl`.
    pub fn real_pi(&mut self, hl: u16) -> u16 { self.real_copy(hl, 0x31A9); hl }

    /// `BD73` (ROM `31AE`): select degrees (non-zero) or radians (0) for the trig functions.
    pub fn real_set_angle_mode(&mut self, degrees: u8) { self.wr8(MATHS_ANGLE_MODE, degrees); }

    /// `BD8B` (ROM `31B2`): cosine.
    pub fn real_cos(&mut self, hl: u16) -> bool {
        let (a, f) = self.real_sign(hl);
        if f.sign { self.real_negate(hl); }                           // cos(-x) = cos(x)
        self.real_sin_cos(hl, a | 1)
    }

    /// `BD88` (ROM `31BC`): sine.
    pub fn real_sin(&mut self, hl: u16) -> bool { self.real_sin_cos(hl, 0) }

    /// ROM `31BD`: `phase` = 0 for sine, non-zero for cosine (which differs by adding half a
    /// turn to the reduced argument).
    ///
    /// The argument is multiplied by 1/(2 pi) (or 1/360), reduced to a fraction `f` in
    /// [-0.5, 0.5]; odd turns flip the sign; the sine is `2f * P((2f)^2)` with a degree-5
    /// polynomial.
    fn real_sin_cos(&mut self, hl: u16, phase: u8) -> bool {
        let degrees = self.rd8(MATHS_ANGLE_MODE) != 0;
        let (k, b) = if degrees { (0x3222, 0xF6) } else { (0x321D, 0xF0) };
        let (above, _) = self.real_exp_above(hl, b);
        if !above {                                                   // L_320D: tiny argument
            if phase != 0 { return self.real_set_one(hl); }
            if !degrees { return true; }
            return self.real_mul(hl, 0x3227);                         // pi/180
        }
        let Some((nmag, _)) = self.real_reduce(hl, k, phase != 0) else { return false };
        if nmag & 1 != 0 { self.real_negate(hl); }
        let (above, _) = self.real_exp_above(hl, 0xE8);
        if !above { self.real_zero_result(hl); return true; }
        let e = self.rd8(hl.wrapping_add(4)).wrapping_add(1);        // 2f
        self.wr8(hl.wrapping_add(4), e);
        self.real_series(hl, 0x31EB, true);
        self.real_mul(hl, MATHS_T2)
    }

    /// `BD8E` (ROM `3231`): tangent = sin / cos.
    pub fn real_tan(&mut self, hl: u16) -> bool {
        self.real_copy(MATHS_T1, hl);
        let mut ok = self.real_cos(MATHS_T1);
        if ok { ok = self.real_sin(hl); }
        if ok { self.real_div(hl, MATHS_T1).carry } else { false }
    }

    /// `BD91` (ROM `3241`): arctangent.
    ///
    /// Works on |x|; values of 1 or more are inverted first and the result subtracted from
    /// pi/2; the 11-term series is in x*x.
    pub fn real_atn(&mut self, hl: u16) -> bool {
        let (sign, f) = self.real_sign(hl);
        if f.sign { self.real_negate(hl); }
        let (above, a) = self.real_exp_above(hl, 0xF0);
        if above {
            let invert = a.wrapping_sub(1) & 0x80 == 0;
            if invert { self.real_reciprocal(hl); }
            self.real_series(hl, 0x3257, true);
            self.real_mul(hl, MATHS_T2);
            if invert { self.real_sub_reverse(hl, 0x3205); }          // pi/2 - x
        }
        if self.rd8(MATHS_ANGLE_MODE) != 0 { self.real_mul(hl, 0x322C); }   // 180/pi
        if sign == 0xFF { self.real_negate(hl); }
        true
    }
}
