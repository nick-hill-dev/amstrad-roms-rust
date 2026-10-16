//! # The accumulator, the operators and small helpers  (ROM `FC3E .. FFFF`)
//!
//! ## The accumulator
//!
//! Every expression leaves its value at [`ACC`] with its type at [`ACC_TYPE`]:
//!
//! | type | meaning  | value at `B0C2`                                            |
//! |------|----------|------------------------------------------------------------|
//! | `2`  | integer  | 16-bit two's complement                                    |
//! | `3`  | string   | the address of a 3-byte descriptor `(length, address)`      |
//! | `5`  | real     | 5-byte floating point (see [`cpc464_os::maths`])           |
//!
//! The type byte doubles as the size of the value, which is how values are copied
//! (`FF66`: `LDIR` with the type as the count).
//!
//! ## Binary operators
//!
//! When an operator routine runs, the right operand is in the accumulator and the left
//! one has just been popped from BASIC's stack (`HL` = its address, `C` = its type).
//! [`Basic::unify_operands`] (`FE15`) brings both to the same type: if either is real the
//! other is converted *in place* (the left operand's stack slot is only as big as an
//! integer, but the real is written there anyway - the stack has room above its top).
//! Integer `+ - *` that overflow are redone in real arithmetic.
//!
//! *Overflow* and *Division by zero* from real arithmetic are "soft" errors: unless an
//! `ON ERROR GOTO` is active the message is printed and evaluation continues with
//! whatever result the firmware left (see [`crate::errors`]).

use crate::fw::*;
use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// What [`Basic::numeric_acc`] finds in the accumulator (`FF2D`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Num {
    /// An integer (carry set), with its value.
    Int(u16),
    /// A real (carry clear); it is at [`ACC`].
    Real,
}

/// The two operands of a binary operator after [`Basic::unify_operands`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operands {
    /// Both integers: `(right, left)` values (the ROM's `HL`, `DE`).
    Int(u16, u16),
    /// Both reals: the right one is the accumulator, the left one at this address.
    Real(u16),
}

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Setting and reading the accumulator   (FF02 .. FF70)
    // ===============================================================================

    /// ROM `FF0D`: the accumulator becomes the integer `v`.
    pub fn set_acc_int(&mut self, v: u16) {
        self.wr16(ACC, v);
        self.wr(ACC_TYPE, T_INT);
    }

    /// ROM `FF05`: the integer value of `a` taken as a signed byte (-1 for `FF`).
    pub fn set_acc_signed_byte(&mut self, a: u8) {
        self.set_acc_int(a as i8 as i16 as u16);
    }

    /// ROM `FF19`: mark the accumulator as real (the value is already there).
    pub fn set_acc_type_real(&mut self) { self.wr(ACC_TYPE, T_REAL); }

    /// ROM `FF23`: the type of the accumulator.
    pub fn acc_type(&self) -> u8 { self.rd(ACC_TYPE) }

    /// ROM `FF45` (zero flag): is the accumulator a string?
    pub fn acc_is_string(&self) -> bool { self.rd(ACC_TYPE) == T_STR }

    /// ROM `FF3C`: the accumulator must be a string, else *Type mismatch*.
    pub fn require_string(&self) -> Res {
        if self.acc_is_string() { Ok(()) } else { Err(Exit::Error(0x0D)) }
    }

    /// ROM `FF2D`: the accumulator must be a number (else *Type mismatch*).
    pub fn numeric_acc(&self) -> Res<Num> {
        match self.rd(ACC_TYPE) {
            T_STR => Err(Exit::Error(0x0D)),
            t if t < T_STR => Ok(Num::Int(self.rd16(ACC))),
            _ => Ok(Num::Real),
        }
    }

    /// ROM `FF4B`: load the accumulator with the value of type `t` at `src`.
    pub fn set_acc_from(&mut self, src: u16, t: u8) {
        self.wr(ACC_TYPE, t);
        self.copy_value(src, ACC);
    }

    /// ROM `FF62`: copy the accumulator's value to `dest`.
    pub fn copy_acc_to(&mut self, dest: u16) { self.copy_value(ACC, dest); }

    /// ROM `FF66`: copy a value of the accumulator's type (its size) from `src` to `dest`.
    pub fn copy_value(&mut self, src: u16, dest: u16) {
        let n = self.rd(ACC_TYPE) as u16;
        self.ldir(src, dest, n);
    }

    /// ROM `FF53`: push the accumulator on BASIC's stack.  Returns its type (`C`).
    pub fn push_acc(&mut self) -> Res<u8> {
        let t = self.rd(ACC_TYPE);
        let slot = self.stack_alloc(t)?;                  // F5B0
        self.copy_acc_to(slot);
        Ok(t)
    }

    /// ROM `FEF3`: zero the accumulator's value bytes (the type is left alone).
    pub fn zero_acc(&mut self) {
        self.wr16(ACC, 0);
        self.wr16(ACC + 2, 0);
        self.wr16(ACC + 3, 0);
    }

    // ===============================================================================
    // Conversions   (FE60 .. FEF2)
    // ===============================================================================

    /// ROM `FE60`: the accumulator becomes the real value of the *unsigned* `v`.
    pub fn set_acc_unsigned_real(&mut self, v: u16) {
        self.wr(ACC_TYPE, T_REAL);
        self.int_to_real(v, ACC, 0);
    }

    /// ROM `FE6A`: the accumulator becomes the real value of the signed integer `v`.
    pub fn set_acc_signed_real(&mut self, v: u16) {
        self.wr(ACC_TYPE, T_REAL);
        self.int_to_real_at(v, ACC);
    }

    /// ROM `FE72`: convert signed `v` to a real at `dest` (`INT TO REAL` with the sign
    /// split off).
    fn int_to_real_at(&mut self, v: u16, dest: u16) {
        let sign = (v >> 8) as u8;
        let mag = if sign & 0x80 != 0 { self.int_negate(v).0 } else { v };
        self.int_to_real(mag, dest, sign);
    }

    /// ROM `FE63`: convert the integer stored at `addr` into a real *in place* (5 bytes).
    pub fn int_in_place_to_real(&mut self, addr: u16) {
        let v = self.rd16(addr);
        self.int_to_real_at(v, addr);
    }

    /// ROM `FE7C`: the accumulator becomes the unsigned 32-bit value `high:low` (`TIME`).
    pub fn set_acc_u32(&mut self, high: u16, low: u16) {
        self.wr16(ACC, low);
        self.wr16(ACC + 2, high);
        self.wr(ACC_TYPE, T_REAL);
        self.bin_to_real(ACC, 0);
    }

    /// ROM `FE8D`: `CINT` - the accumulator becomes an integer; *Overflow* if it does not
    /// fit, *Type mismatch* for a string.  Returns the value.
    pub fn acc_to_int(&mut self) -> Res<u16> {
        // FE93: FEA5, then the result register is stored whether or not it fitted.
        let (v, ok) = self.acc_int_value()?;
        self.wr16(ACC, v);
        if ok { Ok(v) } else { Err(Exit::Error(6)) }      // FED2
    }

    /// ROM `FEA5`: the accumulator's value as an integer, `(HL, fits)`.  The type byte is
    /// set to integer *before* the conversion is tried; the value bytes are not touched.
    pub fn acc_int_value(&mut self) -> Res<(u16, bool)> {
        let t = self.rd(ACC_TYPE);
        self.wr(ACC_TYPE, T_INT);
        self.value_to_int(t, ACC)
    }

    /// ROM `FEAC`: the value of type `t` at `addr` as an integer: `(HL, fits)`.  `HL` is
    /// whatever the conversion left when it does not fit.  A string is a *Type mismatch*.
    fn value_to_int(&mut self, t: u8, addr: u16) -> Res<(u16, bool)> {
        if t < T_STR { return Ok((self.rd16(addr), true)); }
        if t == T_STR { return Err(Exit::Error(0x0D)); }  // FF40
        let (mag, f) = self.real_to_int(addr);
        if !f.carry { return Ok((mag, false)); }
        Ok(self.int_apply_sign(mag, if f.sign { 0x80 } else { 0 }))
    }

    /// ROM `FEC2`: `UNT` - an integer, or a real `-32768..65535` taken as an unsigned
    /// 16-bit value; the accumulator becomes that integer.
    pub fn acc_to_unsigned(&mut self) -> Res<u16> {
        if let Num::Int(v) = self.numeric_acc()? { return Ok(v); }
        let (mag, f) = self.real_to_int(ACC);
        if !f.carry { return Err(Exit::Error(6)); }
        let v = if f.sign {
            let (v, ok) = self.int_apply_sign(mag, 0x80);
            if !ok { return Err(Exit::Error(6)); }
            v
        } else {
            mag
        };
        self.set_acc_int(v);
        Ok(v)
    }

    /// ROM `FED7`: convert the accumulator to type `t` (for assignments and function
    /// arguments): to integer with *Overflow* check, to real, or check it is a string.
    pub fn convert_acc_to(&mut self, t: u8) -> Res {
        if t == self.rd(ACC_TYPE) { return Ok(()); }
        match t {
            t if t < T_STR => { self.acc_to_int()?; Ok(()) }
            T_STR => self.require_string(),
            _ => self.acc_to_real(),
        }
    }

    /// ROM `FEEC`: `CREAL` - make the accumulator real.
    pub fn acc_to_real(&mut self) -> Res {
        if let Num::Int(v) = self.numeric_acc()? { self.set_acc_signed_real(v); }
        Ok(())
    }

    // ===============================================================================
    // Operand preparation   (FE15 .. FE5F, FE9A)
    // ===============================================================================

    /// ROM `FE15`: bring the left operand (at `left`, type `lt`) and the accumulator to a
    /// common numeric type, converting an integer to real in place if the other is real.
    pub fn unify_operands(&mut self, left: u16, lt: u8) -> Res<Operands> {
        if lt == T_STR { return Err(Exit::Error(0x0D)); }
        let rt = self.rd(ACC_TYPE);
        if rt == T_STR { return Err(Exit::Error(0x0D)); }
        if rt == lt {
            if rt == T_INT { return Ok(Operands::Int(self.rd16(ACC), self.rd16(left))); }
            return Ok(Operands::Real(left));
        }
        if rt > lt {
            // right real, left integer
            self.int_in_place_to_real(left);
        } else {
            // right integer, left real
            self.wr(ACC_TYPE, lt);
            self.int_in_place_to_real(ACC);
        }
        Ok(Operands::Real(left))
    }

    /// ROM `FE4F`: after an integer overflow, convert both integer operands to reals:
    /// the accumulator, and the left operand at the top of BASIC's stack.  Returns the
    /// left operand's address.
    fn promote_operands(&mut self) -> u16 {
        let right = self.rd16(ACC);
        self.set_acc_signed_real(right);
        let left = self.rd16(STACK_PTR);
        self.int_in_place_to_real(left);
        left
    }

    /// ROM `FE9A`: both operands as integers (for `\ MOD AND OR XOR`): `(left, right)`.
    fn int_operands(&mut self, left: u16, lt: u8) -> Res<(u16, u16)> {
        let (l, ok) = self.value_to_int(lt, left)?;
        if !ok { return Err(Exit::Error(6)); }
        let (r, ok) = self.acc_int_value()?;
        if !ok { return Err(Exit::Error(6)); }
        Ok((l, r))
    }

    // ===============================================================================
    // Arithmetic operators   (FCCC .. FD84)
    // ===============================================================================

    /// `+` (ROM `FCCC`).
    pub fn op_add(&mut self, left: u16, lt: u8) -> Res {
        let l = match self.unify_operands(left, lt)? {
            Operands::Int(r, l) => {
                let (v, ok) = self.int_op(INT_ADD, r, l);
                if ok { self.set_acc_int(v); return Ok(()); }
                self.promote_operands()
            }
            Operands::Real(l) => l,
        };
        if !self.real_op(REAL_ADD, ACC, l) { self.overflow_error()?; }
        Ok(())
    }

    /// `-` (ROM `FCE1`).
    pub fn op_subtract(&mut self, left: u16, lt: u8) -> Res {
        let l = match self.unify_operands(left, lt)? {
            Operands::Int(r, l) => {
                let (v, ok) = self.int_op(INT_REVERSE_SUB, r, l);
                if ok { self.set_acc_int(v); return Ok(()); }
                self.promote_operands()
            }
            Operands::Real(l) => l,
        };
        if !self.real_op(REAL_REVERSE_SUB, ACC, l) { self.overflow_error()?; }
        Ok(())
    }

    /// `*` (ROM `FCF5`).
    pub fn op_multiply(&mut self, left: u16, lt: u8) -> Res {
        let l = match self.unify_operands(left, lt)? {
            Operands::Int(r, l) => {
                let (v, ok) = self.int_op(INT_MULTIPLY, r, l);
                if ok { self.set_acc_int(v); return Ok(()); }
                self.promote_operands()
            }
            Operands::Real(l) => l,
        };
        if !self.real_op(REAL_MULTIPLY, ACC, l) { self.overflow_error()?; }
        Ok(())
    }

    /// ROM `FD09`: compare the left operand with the accumulator: `FF` if the
    /// accumulator (right) is the smaller, `0` equal, `1` larger.
    pub fn compare_numbers(&mut self, left: u16, lt: u8) -> Res<u8> {
        Ok(match self.unify_operands(left, lt)? {
            Operands::Int(r, l) => self.int_compare(r, l),
            Operands::Real(l) => self.real_compare(ACC, l),
        })
    }

    /// `/` (ROM `FD12`): always real.  The quotient is computed in the left operand's
    /// stack slot and copied to the accumulator.
    pub fn op_divide(&mut self, left: u16, lt: u8) -> Res {
        let l = if (self.rd(ACC_TYPE) | lt) == T_INT {
            self.promote_operands()
        } else {
            match self.unify_operands(left, lt)? {
                Operands::Real(l) => l,
                Operands::Int(..) => unreachable!("both integers handled above"),
            }
        };
        let f = self.real_divide(l, ACC);
        self.ldir(l, ACC, 5);
        if f.carry { return Ok(()); }
        if f.zero { self.division_by_zero_error() } else { self.overflow_error() }
    }

    /// `\` integer division (ROM `FD37`).  `-32768 \ -1` gives the real 32768.
    pub fn op_int_divide(&mut self, left: u16, lt: u8) -> Res {
        let (l, r) = self.int_operands(left, lt)?;
        match self.int_divide(INT_DIVIDE, l, r) {
            Some((v, true)) => { self.set_acc_int(v); Ok(()) }
            None => Err(Exit::Error(0x0B)),
            Some((_, false)) => { self.set_acc_unsigned_real(0x8000); Ok(()) }
        }
    }

    /// `MOD` (ROM `FD49`).
    pub fn op_mod(&mut self, left: u16, lt: u8) -> Res {
        let (l, r) = self.int_operands(left, lt)?;
        match self.int_divide(INT_MODULUS, l, r) {
            Some((v, true)) => { self.set_acc_int(v); Ok(()) }
            _ => Err(Exit::Error(0x0B)),
        }
    }

    /// `AND` (ROM `FD58`).
    pub fn op_and(&mut self, left: u16, lt: u8) -> Res {
        let (l, r) = self.int_operands(left, lt)?;
        self.set_acc_int(l & r);
        Ok(())
    }
    /// `OR` (ROM `FD63`).
    pub fn op_or(&mut self, left: u16, lt: u8) -> Res {
        let (l, r) = self.int_operands(left, lt)?;
        self.set_acc_int(l | r);
        Ok(())
    }
    /// `XOR` (ROM `FD6D`).
    pub fn op_xor(&mut self, left: u16, lt: u8) -> Res {
        let (l, r) = self.int_operands(left, lt)?;
        self.set_acc_int(l ^ r);
        Ok(())
    }

    /// `NOT` (ROM `FD77`).
    pub fn not_acc(&mut self) -> Res {
        let v = self.acc_to_int()?;
        self.set_acc_int(!v);
        Ok(())
    }

    /// `ABS` (ROM `FD85`).
    pub fn fn_abs(&mut self) -> Res {
        if self.acc_sign()? & 0x80 == 0 { return Ok(()); }
        self.negate_acc()
    }

    /// ROM `FD89`: negate the accumulator (`-32768` becomes the real 32768).
    pub fn negate_acc(&mut self) -> Res {
        match self.numeric_acc()? {
            Num::Int(v) => {
                let (n, ok) = self.int_negate(v);
                self.wr16(ACC, n);
                if !ok { self.set_acc_unsigned_real(n); }
            }
            Num::Real => self.real_negate(ACC),
        }
        Ok(())
    }

    /// ROM `FDA3`: the sign of the accumulator: `FF`, `0` or `1`.
    pub fn acc_sign(&mut self) -> Res<u8> {
        Ok(match self.numeric_acc()? {
            Num::Int(v) => self.int_sign(v),
            Num::Real => self.real_sign(ACC).0,
        })
    }

    /// ROM `FDAF`: `ROUND` of the value of type `t` at `src` to `digits` decimal places
    /// (negative: to tens, hundreds ...).  The result is in the accumulator.
    pub fn round_value(&mut self, src: u16, t: u8, digits: u8) -> Res {
        self.set_acc_from(src, t);
        match self.numeric_acc()? {
            Num::Int(v) => {
                if digits & 0x80 == 0 { return Ok(()); }
                self.set_acc_signed_real(v);
                self.round_real(digits, src);
                self.acc_to_int()?;
                Ok(())
            }
            Num::Real => {
                if digits == 0 {
                    self.real_to_binary_and_back(REAL_TO_BIN);
                } else {
                    self.round_real(digits, src);
                }
                Ok(())
            }
        }
    }

    /// ROM `FDCE`: scale by 10^`digits`, round to an integer, convert back and scale back.
    /// If anything overflows the original value (still at `orig`) is reloaded.
    fn round_real(&mut self, digits: u8, orig: u16) {
        let mut ok = self.real_pow10(ACC, digits);
        let mut sign = 0;
        if ok {
            let (b, _c, f) = self.real_to_binary(REAL_TO_BIN, ACC);
            sign = b;
            ok = f.carry;
        }
        if !ok {
            // FDE4: back to the unrounded value.
            self.copy_value(orig, ACC);
            return;
        }
        self.bin_to_real(ACC, sign);
        self.real_pow10(ACC, 0u8.wrapping_sub(digits));
    }

    /// `FIX` (ROM `FDE8`).
    pub fn fn_fix(&mut self) -> Res {
        if let Num::Int(_) = self.numeric_acc()? { return Ok(()); }
        self.real_to_binary_and_back(REAL_FIX);
        Ok(())
    }

    /// `INT` (ROM `FDED`).
    pub fn fn_int(&mut self) -> Res {
        if let Num::Int(_) = self.numeric_acc()? { return Ok(()); }
        self.real_to_binary_and_back(REAL_INT);
        Ok(())
    }

    /// ROM `FDF4`: convert the real accumulator to a binary integer with `entry`
    /// (`REAL TO BIN`, `REAL FIX` or `REAL INT`); keep it as an integer if it fits,
    /// otherwise convert back to real.  If the conversion fails (the number is too big
    /// to have a fraction) the accumulator is left as it is.
    fn real_to_binary_and_back(&mut self, entry: u16) {
        let (sign, bytes, f) = self.real_to_binary(entry, ACC);
        if !f.carry { return; }
        if bytes < 3 {
            // FE06
            let mag = self.rd16(ACC);
            let (v, ok) = self.int_apply_sign(mag, sign);
            if ok { self.set_acc_int(v); return; }
        }
        self.bin_to_real(ACC, sign);
    }

    /// `SGN` (ROM `FF02`).
    pub fn fn_sgn(&mut self) -> Res {
        let s = self.acc_sign()?;
        self.set_acc_signed_byte(s);
        Ok(())
    }

    // ===============================================================================
    // Character classes and tables   (FF71 .. FFB7)
    // ===============================================================================

    /// ROM `FF93`: look `c` up in a dispatch table `count, default, (char, address)*` and
    /// return the address (the default if not found).
    pub fn table_lookup(&self, table: u16, c: u8) -> u16 {
        let n = self.rom_byte(table);
        for k in 0..n as u16 {
            let e = table + 3 + 3 * k;
            if self.rom_byte(e) == c { return self.rom_word(e + 1); }
        }
        self.rom_word(table + 1)
    }

    /// ROM `FFAA`: is `c` in the zero-terminated list at `list`?
    pub fn in_list(&self, list: u16, c: u8) -> bool {
        let mut a = list;
        loop {
            let v = self.rd(a);
            if v == 0 { return false; }
            a = a.wrapping_add(1);
            if v == c { return true; }
        }
    }
}

/// ROM `FF8A`: lower case letters to upper case.
pub fn to_upper(c: u8) -> u8 { if (b'a'..=b'z').contains(&c) { c - 0x20 } else { c } }

/// ROM `FF71` (carry): a letter (either case)?
pub fn is_letter(c: u8) -> bool { to_upper(c).is_ascii_uppercase() }

/// ROM `FF83` (carry): a decimal digit?
pub fn is_digit(c: u8) -> bool { c.is_ascii_digit() }

/// ROM `FF7F` (carry): a digit or a point?
pub fn is_digit_or_point(c: u8) -> bool { c == b'.' || is_digit(c) }

/// ROM `FF7B` (carry): a character that may continue a name: letter, digit or point.
pub fn is_name_char(c: u8) -> bool { is_letter(c) || is_digit_or_point(c) }
