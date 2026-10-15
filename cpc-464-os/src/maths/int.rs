//! 16-bit integer arithmetic (ROM `3708`-`37FF`).
//!
//! Integers are 16-bit two's complement in `HL` / `DE`.  Most routines report overflow
//! (or division by zero) with the **carry flag clear**; the Rust versions return a `bool`
//! that is `true` where the Z80 sets carry.

use super::Flags;

/// `BDA3` (ROM `3708`): take the absolute value of `hl` and remember its sign.
///
/// Exit: `HL` = |hl| (as an unsigned magnitude, `0x8000` stays), `B` = original high byte
/// (bit 7 = sign), `E` = 0, `C` = 2.  Used to feed integers to the real-number converters.
pub fn int_abs_split(hl: u16) -> (u16, u8) {
    let b = (hl >> 8) as u8;
    (int_abs(hl), b)
}

/// `BDA6` (ROM `370E`): as [`int_abs_split`] but for a value that is already positive.
/// Exit: `B` = 0, `E` = 0, `C` = 2.
pub fn int_unsigned_split() -> u8 { 0 }

/// ROM `37D1`: absolute value (negate only if `H` has bit 7 set).
fn int_abs(hl: u16) -> u16 {
    if hl & 0x8000 != 0 { int_negate(hl).0 } else { hl }
}

/// `BDA9` (ROM `3715`): apply the sign in `b` (bit 7) to the magnitude `hl`.
///
/// Returns `(result, ok, a)`: `ok` (carry) is clear if the magnitude does not fit
/// (positive above 32767, negative above 32768).  `a` is the Z80 `A` register on exit, which
/// differs from entry only in the special `0x8000` case.
pub fn int_apply_sign(hl: u16, b: u8) -> (u16, bool, u8) {
    let h = (hl >> 8) as u8;
    let l = hl as u8;
    if h & 0x80 == 0 {
        // Positive magnitude: if the sign byte is negative, negate.
        if (h | b) & 0x80 != 0 {
            return int_negate_flags(hl);
        }
        return (hl, true, h | b);
    }
    // Magnitude >= 0x8000: only exactly 0x8000 with a negative sign is representable.
    let a = (h ^ 0x80) | l;
    if a != 0 { return (hl, false, a); }
    // A = B, SCF, ADC A,A  ->  carry = bit 7 of B
    (hl, b & 0x80 != 0, b.wrapping_mul(2).wrapping_add(1))
}

/// `BDAC` (ROM `3728`): signed addition.  `ok` is false on overflow (A = 0xFF then).
pub fn int_add(hl: u16, de: u16) -> (u16, bool) {
    let (r, _) = hl.overflowing_add(de);
    let overflow = ((hl ^ r) & (de ^ r)) & 0x8000 != 0;
    (r, !overflow)
}

/// `BDAF` (ROM `3731`): signed subtraction `HL - DE`.
pub fn int_sub(hl: u16, de: u16) -> (u16, bool) {
    let r = hl.wrapping_sub(de);
    let overflow = ((hl ^ de) & (hl ^ r)) & 0x8000 != 0;
    (r, !overflow)
}

/// `BDB2` (ROM `3730`): reverse subtraction `DE - HL`.
pub fn int_sub_reverse(hl: u16, de: u16) -> (u16, bool) { int_sub(de, hl) }

/// `BDB5` (ROM `3739`): signed multiplication.  Returns `(result, ok)`.
pub fn int_mul(hl: u16, de: u16) -> (u16, bool) {
    let b = ((hl >> 8) as u8) ^ ((de >> 8) as u8);
    let (hl, de) = (int_abs(hl), int_abs(de));
    let (prod, overflow) = int_mul_unsigned(hl, de);
    if overflow { return (prod, false); }
    let (r, ok, _) = int_apply_sign(prod, b);
    (r, ok)
}

/// `BDBE` (ROM `3750`): unsigned multiplication `HL * DE`.
///
/// Returns `(result, overflow)`; **carry is set on overflow** here (the opposite of the
/// signed routines).  The algorithm is the original's: if both operands exceed 255 the
/// product cannot fit; otherwise the byte-sized operand drives a shift-and-add loop.
pub fn int_mul_unsigned(hl: u16, de: u16) -> (u16, bool) {
    let (mut hl, mut de) = (hl, de);
    if hl >> 8 != 0 {
        if de >> 8 != 0 { return (hl, true); }
        std::mem::swap(&mut hl, &mut de);
    }
    // Now H == 0 and the multiplier is L.
    let mut a = hl as u8;
    if a == 0 { return (hl, false); }
    hl = de;
    if de == 0 { return (hl, false); }
    if a < 3 {
        if a == 1 { return (hl, false); }
        let (r, c) = hl.overflowing_add(hl);
        return (r, c);
    }
    // Shift the multiplier left until its leading 1 falls out; the bits that follow drive
    // the double-and-add loop, with a lone marker bit terminating it.
    let mut carry = true;
    loop {
        let c = a & 0x80 != 0;
        a = (a << 1) | carry as u8;
        carry = c;
        if carry { break; }
    }
    loop {
        let (r, c) = hl.overflowing_add(hl);
        hl = r;
        if c { return (hl, true); }
        let c = a & 0x80 != 0;
        a <<= 1;
        if c {
            let (r, c) = hl.overflowing_add(de);
            hl = r;
            if c { return (hl, true); }
        }
        if a == 0x80 { return (hl, false); }
    }
}

/// `BDB8` (ROM `377A`): signed division (truncating).  Returns `None` for division by zero
/// (the ROM returns with carry clear and zero set) and `Some((quotient, ok))` otherwise,
/// where `ok` is false if the quotient does not fit (`-32768 / -1`).
pub fn int_div(hl: u16, de: u16) -> Option<(u16, bool)> {
    let (q, _r, b) = int_div_prepare(hl, de)?;
    let (r, ok, _) = int_apply_sign(q, b);
    Some((r, ok))
}

/// `BDBB` (ROM `3781`): signed modulus; the result has the sign of the dividend.
pub fn int_mod(hl: u16, de: u16) -> Option<(u16, bool)> {
    let c = (hl >> 8) as u8;
    let (_q, rem, _b) = int_div_prepare(hl, de)?;
    let (r, ok, _) = int_apply_sign(rem, c);
    Some((r, ok))
}

/// ROM `3789`: strip signs and divide.  Returns `(quotient, remainder, sign byte)`.
fn int_div_prepare(hl: u16, de: u16) -> Option<(u16, u16, u8)> {
    let b = ((hl >> 8) as u8) ^ ((de >> 8) as u8);
    let (hl, de) = (int_abs(hl), int_abs(de));
    let (q, r) = int_div_unsigned_checked(hl, de)?;
    Some((q, r, b))
}

/// `BDC1` (ROM `378C`): unsigned division.  `None` = division by zero.
pub fn int_div_unsigned_checked(hl: u16, de: u16) -> Option<(u16, u16)> {
    if de == 0 { return None; }
    Some((hl / de, hl % de))
}

/// Unsigned division as the graphics code uses it: both registers unchanged on a zero
/// divisor, `(quotient, remainder)` otherwise.
pub fn int_div_unsigned(hl: u16, de: u16) -> (u16, u16) {
    int_div_unsigned_checked(hl, de).unwrap_or((hl, de))
}

/// `BDC7` (ROM `37D4`): negate.  Returns `(result, ok)`; `ok` is false only for `0x8000`.
pub fn int_negate(hl: u16) -> (u16, bool) {
    let (r, ok, _) = int_negate_flags(hl);
    (r, ok)
}

/// Negate, also returning the final `A` register of the Z80 routine (the new high byte
/// compared with the old one decides the carry).
fn int_negate_flags(hl: u16) -> (u16, bool, u8) {
    let r = hl.wrapping_neg();
    let new_h = (r >> 8) as u8;
    let old_h = (hl >> 8) as u8;
    if new_h != old_h { return (r, true, new_h); }
    // Equal high bytes: zero (A = 0, carry set) or 0x8000 (A = 0x80, carry clear).
    (r, new_h == 0, new_h)
}

/// `BDCA` (ROM `37E0`): sign of `hl`: `-1`, `0` or `1` as `A` = `0xFF`, `0`, `1`.
/// Flags: carry for negative, zero for zero.
///
/// **ROM bug, kept:** after the sign test `A` is 0 rather than `H`, so the "is it zero?" test
/// only looks at `L`: any positive value whose low byte is 0 (256, 512, ...) is reported as 0.
pub fn int_sign(hl: u16) -> (u8, Flags) {
    if hl & 0x8000 != 0 { return (0xFF, Flags { carry: true, zero: false, sign: true }); }
    if hl as u8 == 0 { return (0, Flags { carry: false, zero: true, sign: false }); }
    (1, Flags::default())
}

/// `BDC4` (ROM `37E9`): compare `HL` with `DE`: `A` = `0xFF` (carry) if less, `0` (zero)
/// if equal, `1` if greater.
pub fn int_compare(hl: u16, de: u16) -> (u8, Flags) {
    let (a, b) = (hl as i16, de as i16);
    if a < b { (0xFF, Flags { carry: true, zero: false, sign: true }) }
    else if a == b { (0, Flags { carry: false, zero: true, sign: false }) }
    else { (1, Flags::default()) }
}
