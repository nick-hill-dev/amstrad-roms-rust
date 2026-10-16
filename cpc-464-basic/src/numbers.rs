//! # Number parsing and formatting  (ROM `ECA3 .. F157`)
//!
//! ## Parsing
//!
//! * [`Basic::parse_signed_number`] (`ECA3`): `VAL`, `INPUT`, `READ`: optional sign,
//!   decimal or `&`/`&H`/`&X` number; text that is not a number gives 0.
//! * [`Basic::parse_number_text`] (`ECBE`): the tokeniser's number reader.
//! * [`Basic::parse_line_number`] (`EE04`): a line number 1..65535.
//!
//! A decimal number is collected as up to 12 significant digits in [`DIGITS`] (one digit
//! per byte, `FF` terminated), converted to a 40-bit binary integer at `AE53..AE57` and
//! then to a real scaled by the decimal exponent (`ED2C`).  Without a point or exponent,
//! and if it fits, it stays an integer.
//!
//! ## Formatting  (`EEA7`)
//!
//! The value is split into a binary mantissa and a decimal exponent (firmware
//! `REAL DECIMAL PREPARE`, or the integer itself), the mantissa is turned into decimal
//! digits right-to-left ending at [`NUMBER_TEXT`] (`F0B7`, through BCD at [`DIGITS`]), and
//! the digit string is then laid out according to a flag byte:
//!
//! | bit | meaning (`PRINT USING` unless noted)                           |
//! |-----|-----------------------------------------------------------------|
//! | 7   | a `USING` field: fixed number of places from [`FORMAT_A`] / [`FORMAT_B`] |
//! | 6   | exponent form `^^^^` (with bit 7); free format: limit to 9 digits |
//! | 5   | `**` fill                                                       |
//! | 4   | sign after the number                                           |
//! | 3   | `+` always print the sign                                       |
//! | 2   | `$$` currency sign                                              |
//! | 1   | `,` thousands separators                                        |
//! | 0   | (internal) the number did not fit: prefix `%`                   |
//!
//! [`FORMAT_A`] is the number of places after the point (+1 if there is a point) and
//! [`FORMAT_B`] the field width before it.  The layout routines are transcribed from the
//! ROM with its registers as local variables: `HL` the start of the text being built
//! (it grows to the left), `C` the number of digits, `B` digits before the point, `E` the
//! decimal exponent, `D` the flags.

use crate::arith::{is_digit, to_upper};
use crate::vars::*;
use crate::{Basic, Res};
use cpc464_os::Hardware;

/// End of the binary mantissa built by the parser (`AE53..AE57`).
const PARSE_BINARY: u16 = 0xAE53;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Parsing   (ECA3 .. EE78)
    // ===============================================================================

    /// ROM `ECA3`: an optional sign and a number at `p`, into the accumulator.
    /// Returns `Some(after)` (carry) on success; text that does not start like a number
    /// gives `Some(p)` with the value 0.  `None`: a malformed number (e.g. `&` without
    /// digits).
    pub fn parse_signed_number(&mut self, p: u16) -> Res<Option<u16>> {
        let (p, sign, had_sign) = self.parse_sign(p);     // ED44
        if had_sign {
            let p = self.skip_white_at_addr(p);
            return self.parse_decimal(p, sign);
        }
        let c = self.rd(p);
        if c == b'&' { return self.parse_based(p); }
        if c == b'.' || is_digit(c) { return self.parse_decimal(p, 0); }
        self.wr(ACC_TYPE, T_INT);
        self.zero_acc();
        Ok(Some(p))
    }

    /// ROM `ECBE`: parse a number at `p` (no sign).  `Ok((after, A))` on success, where
    /// `A` is the base for `&` numbers; `Err(stop)` = how far the attempt got (the
    /// tokeniser copies that much text literally).
    pub fn parse_number_text(&mut self, p: u16) -> Res<core::result::Result<(u16, u8), u16>> {
        let r = if self.rd(p) == b'&' { self.parse_based_full(p)? } else { self.parse_decimal_full(p, 0)? };
        Ok(match r {
            (true, after, a) => Ok((after, a)),
            (false, stop, _) => Err(stop),
        })
    }

    /// Wrapper of [`Self::parse_decimal_full`] for [`Self::parse_signed_number`].
    fn parse_decimal(&mut self, p: u16, sign: u8) -> Res<Option<u16>> {
        let (ok, after, _) = self.parse_decimal_full(p, sign)?;
        Ok(if ok { Some(after) } else { None })
    }
    fn parse_based(&mut self, p: u16) -> Res<Option<u16>> {
        let (ok, after, _) = self.parse_based_full(p)?;
        Ok(if ok { Some(after) } else { None })
    }

    /// ROM `ECCD`: `&`, `&H` or `&X` and digits; the value (an integer) goes into the
    /// accumulator.  A value that overflows prints the soft *Overflow* message and fails.
    /// Returns `(ok, HL, A = base)`.
    fn parse_based_full(&mut self, p: u16) -> Res<(bool, u16, u8)> {
        let (after, value, base, ok, any) = self.based_digits(p);   // EE1C
        self.set_acc_int(value);
        if ok { return Ok((true, after, base)); }
        if any { self.overflow_error()?; }               // CAF3
        Ok((false, after, base))
    }

    /// ROM `ECDC` / `ECF0`: a decimal number with optional point and exponent.
    /// `sign` = `FF` for negative.  Returns `(ok, HL, A)`; on failure `HL` is how far it
    /// got.
    fn parse_decimal_full(&mut self, p: u16, sign: u8) -> Res<(bool, u16, u8)> {
        // ECDC: must start with a digit, or a point followed by a digit.
        {
            let mut q = p;
            let c = self.rd(q);
            q = q.wrapping_add(1);
            if c == b'.' { q = self.skip_white_at_addr(q); }
            let first = if c == b'.' { self.rd(q) } else { c };
            if !is_digit(first) {
                if c != b'.' { return Ok((false, p, c ^ b'.')); }
                return Ok((false, p.wrapping_add(1), 0));
            }
        }
        // ECF0
        self.wr(ACC_TYPE, T_INT);
        let mut st = Digits { hl: p, b: 0, c: 0, de: DIGITS };
        let mut a = self.collect_digits(&mut st);          // ED53
        if a == b'.' {
            st.hl = self.skip_white_at_addr(st.hl).wrapping_add(1);  // EDC9
            self.wr(ACC_TYPE, T_REAL);
            st.c = st.c.wrapping_add(1);
            a = self.collect_digits(&mut st);
            st.c = st.c.wrapping_sub(1);
        }
        self.wr(st.de, 0xFF);
        let exp = self.parse_exponent(&mut st, a)?;      // ED77
        let after = st.hl;
        // ED16: digits -> 40-bit binary
        let bytes = self.digits_to_binary();              // EDCE
        if self.acc_type() < T_STR && bytes < 3 {
            // FE06: an integer if it fits
            let mag = self.rd16(PARSE_BINARY);
            let (v, ok) = self.int_apply_sign(mag, sign);
            if ok { self.set_acc_int(v); return Ok((true, after, 0x0A)); }
        }
        // ED2C: real = binary (AE54..57 with AE53 as guard) * 2^-8, scaled by 10^exp
        let guard = self.rd(PARSE_BINARY);
        self.bin_to_real_scaled(PARSE_BINARY + 1, sign, guard);
        let ok = self.real_pow10(PARSE_BINARY + 1, exp);
        self.wr(ACC_TYPE, T_REAL);
        if ok {
            self.real_copy(ACC, PARSE_BINARY + 1);
            return Ok((true, after, 0x0A));
        }
        self.overflow_error()?;                           // CAF3
        Ok((false, after, 0x0A))
    }

    /// ROM `ED53`: collect decimal digits (spaces between digits are skipped).  Leading
    /// zeros are not stored; at most 12 significant digits are kept (later ones overwrite
    /// the twelfth).  `B` counts significant digits; `C`, if non-zero, counts every digit
    /// (it is 1 + fraction digits when called for the fraction).  Returns the first
    /// non-digit character, upper-cased, with `HL` left *before* any spaces preceding it.
    fn collect_digits(&mut self, st: &mut Digits) -> u8 {
        loop {
            let q = self.skip_white_at_addr(st.hl);
            let c = self.rd(q);
            if !is_digit(c) { return to_upper(c); }
            st.hl = q.wrapping_add(1);
            let d = c - b'0';
            self.wr(st.de, d);
            if d | st.b != 0 {
                let before = st.b;
                st.b = st.b.wrapping_add(1);
                if before < 12 { st.de = st.de.wrapping_add(1); }
            }
            if st.c != 0 { st.c = st.c.wrapping_add(1); }
        }
    }

    /// ROM `ED77`: an optional exponent `E[+|-]digits`, combined with the digit counts into
    /// the power of ten to apply to the 40-bit integer (signed, clamped).
    fn parse_exponent(&mut self, st: &mut Digits, a: u8) -> Res<u8> {
        let mut e: u8 = 0;
        if a == b'E' {
            let save = st.hl;
            let q = self.skip_white_at_addr(st.hl).wrapping_add(1);  // EDC9
            let (q, esign, had) = self.parse_sign(q);     // ED44
            let q = if had { self.skip_white_at_addr(q) } else { q };
            if is_digit(self.rd(q)) {
                self.wr(ACC_TYPE, T_REAL);
                let (after, value, ok) = self.decimal_digits(q);  // EE35
                st.hl = after;
                let mut v = if ok && value < 100 { value as u8 } else { 0x7F };
                if esign != 0 { v = (!v).wrapping_add(1); }
                e = v;
            } else {
                st.hl = save;
            }
        }
        // EDAC: combine with the digit counts.
        let e = e.wrapping_add(0x80);
        let x = if st.b >= 12 { st.b - 12 } else { 0 };
        let (t, borrow) = x.overflowing_sub(st.c);
        Ok(if !borrow {
            let (s, carry) = t.overflowing_add(e);
            let s = if carry { 0xFF } else { s };
            s.wrapping_sub(0x80)
        } else {
            let (s, carry) = t.overflowing_add(e);
            let s = if carry { s } else { 0 };
            s.wrapping_add(0x80).wrapping_add((s < 1) as u8)
        })
    }

    /// ROM `ED44`: skip white space and an optional `-`/`+`: `(after, D, sign found)` with
    /// `D` = `FF` for minus.
    fn parse_sign(&self, p: u16) -> (u16, u8, bool) {
        let q = self.skip_white_at_addr(p);
        match self.rd(q) {
            b'-' => (q.wrapping_add(1), 0xFF, true),
            b'+' => (q.wrapping_add(1), 0, true),
            _ => (q, 0, false),
        }
    }

    /// ROM `EDCE`: the digits at [`DIGITS`] (`FF` terminated) as a little-endian binary
    /// number at `AE53..AE57`.  Returns the number of bytes in use (at least 1).
    fn digits_to_binary(&mut self) -> u8 {
        for i in 0..5 { self.wr(PARSE_BINARY + i, 0); }
        let mut de = DIGITS;
        let mut c: u8 = 1;
        let first = self.rd(de);
        if first == 0xFF { return c; }
        self.wr(PARSE_BINARY, first);
        loop {
            de = de.wrapping_add(1);
            let digit = self.rd(de);
            if digit == 0xFF { return c; }
            // multiply by 10 and add the digit, byte by byte
            let mut carry = digit as u16;
            for i in 0..c as u16 {
                let v = self.rd(PARSE_BINARY + i) as u16 * 10 + carry;
                self.wr(PARSE_BINARY + i, v as u8);
                carry = v >> 8;
            }
            if carry != 0 {
                self.wr(PARSE_BINARY + c as u16, carry as u8);
                c += 1;
            }
        }
    }

    /// ROM `EE04`: a line number (1..65535) at `p` into the accumulator.  `Ok(after)`, or
    /// `Err(stop)` (text up to `stop` was looked at) for no digits, 0 or overflow.
    pub fn parse_line_number(&mut self, p: u16) -> core::result::Result<u16, u16> {
        let (after, value, ok) = self.decimal_digits(p);  // EE35
        self.set_acc_int(value);
        if ok && value != 0 { Ok(after) } else { Err(after) }
    }

    /// ROM `EE1C`: `&`, an optional `H`/`X` (spaces allowed), and digits.
    /// Returns `(after, value, base, ok, any digit)`.
    fn based_digits(&mut self, p: u16) -> (u16, u16, u8, bool, bool) {
        let mut q = self.skip_white_at_addr(p.wrapping_add(1));
        let c = to_upper(self.rd(q));
        let base = match c { b'X' => 2, _ => 16 };
        if c == b'X' || c == b'H' { q = self.skip_white_at_addr(q.wrapping_add(1)); }
        let (after, value, ok, any) = self.digits_in_base(q, base);
        (after, value, base, ok, any)
    }

    /// ROM `EE35`: decimal digits: `(after, value, ok)`; `ok` is false for no digits or
    /// overflow.
    pub fn decimal_digits(&self, p: u16) -> (u16, u16, bool) {
        let (after, value, ok, _) = self.digits_in_base(p, 10);
        (after, value, ok)
    }

    /// ROM `EE37`: digits in `base` (2, 10 or 16).  Overflow beyond 16 bits is noted but
    /// the digits are still consumed.  Returns `(after, value, ok, any digit)`.
    pub(crate) fn digits_in_base(&self, p: u16, base: u8) -> (u16, u16, bool, bool) {
        let digit = |a: u16| -> Option<u8> {
            let c = self.rd(a);
            let v = if is_digit(c) {
                c - b'0'
            } else {
                let u = to_upper(c);
                if u < b'A' { return None; }
                u - 7 - b'0'
            };
            if v < base { Some(v) } else { None }
        };
        let mut q = p;
        let Some(first) = digit(q) else { return (q, 0, false, false) };
        q = q.wrapping_add(1);
        let mut v = first as u16;
        let mut overflow = false;
        while let Some(d) = digit(q) {
            q = q.wrapping_add(1);
            let (m, o) = cpc464_os::maths::int_mul_unsigned(v, base as u16);
            if o { overflow = true; v = m; continue; }
            let (s, o2) = m.overflowing_add(d as u16);
            v = s;
            if o2 { overflow = true; }
        }
        (q, v, !overflow, true)
    }

    /// `DD61` on an address other than the text pointer.
    pub fn skip_white_at_addr(&self, mut p: u16) -> u16 {
        loop {
            let c = self.rd(p);
            if c != b' ' && c != 0x09 && c != 0x0A { return p; }
            p = p.wrapping_add(1);
        }
    }

    // ===============================================================================
    // Formatting entry points   (EE79 .. EEA6)
    // ===============================================================================

    /// ROM `EE79`: print `n` as an unsigned decimal number.
    pub fn print_number(&mut self, n: u16) -> Res {
        self.set_acc_int(n);
        let s = self.format_unsigned_acc()?;
        self.print_string(s)
    }

    /// ROM `EE82`: the accumulator (`UNT`) as unsigned digits, no leading space.
    /// Returns the address of the zero-terminated text.
    pub fn format_unsigned_acc(&mut self) -> Res<u16> {
        self.acc_to_unsigned()?;                          // FCC3
        let b = self.int_unsigned_split(self.rd16(ACC));
        let p = Prep { hl: ACC + 1, b, c: 2, e: 0 };
        let s = self.format_prepared(p, 0);
        Ok(s.wrapping_add(1))
    }

    /// ROM `EE8F`: the accumulator in free format without a leading space.  `hl` is the
    /// caller's `HL`, which the ROM stores at [`FORMAT_A`] on the way.
    pub fn format_acc_trimmed_hl(&mut self, hl: u16) -> Res<u16> {
        let s = self.format_acc(0, hl)?;
        Ok(if self.rd(s) == b' ' { s.wrapping_add(1) } else { s })
    }

    /// [`Self::format_acc_trimmed_hl`] for callers whose `HL` is the accumulator's own
    /// integer value (the de-tokeniser's numbers).
    pub fn format_acc_trimmed(&mut self) -> Res<u16> {
        let hl = self.rd16(ACC);
        self.format_acc_trimmed_hl(hl)
    }

    /// ROM `EE9D`: the accumulator in free format for `PRINT`/`STR$` (flags `40`).
    pub fn format_acc_print(&mut self, hl: u16) -> Res<u16> { self.format_acc(0x40, hl) }

    /// ROM `EE9F`: format the accumulator with `flags` (see the module documentation).
    /// `hl` is stored at [`FORMAT_A`]/[`FORMAT_B`] first - it *is* the field description
    /// for `PRINT USING`, and whatever the caller had in `HL` otherwise.
    pub fn format_acc(&mut self, flags: u8, hl: u16) -> Res<u16> {
        self.wr16(FORMAT_A, hl);
        let p = self.prepare_for_format()?;               // FCB3
        Ok(self.format_prepared(p, flags))
    }

    /// ROM `FCB3`: split the accumulator for formatting: integers become magnitude and
    /// sign (exponent 0); reals go through `REAL DECIMAL PREPARE`.
    fn prepare_for_format(&mut self) -> Res<Prep> {
        Ok(match self.numeric_acc()? {
            crate::arith::Num::Int(v) => {
                let (mag, sign) = self.int_abs_split(v);
                self.wr16(ACC, mag);
                Prep { hl: ACC + 1, b: sign, c: 2, e: 0 }
            }
            crate::arith::Num::Real => {
                let (hl, b, c, _d, e) = self.real_decimal_prepare(ACC);
                Prep { hl, b, c, e }
            }
        })
    }

    // ===============================================================================
    // The formatter   (EEA7 .. F0DC)
    // ===============================================================================

    /// ROM `EEA7`: lay out a prepared number.  Returns the start of the text; it ends at
    /// [`NUMBER_END`] (a zero byte).
    fn format_prepared(&mut self, p: Prep, flags: u8) -> u16 {
        self.wr(NUMBER_TEXT, 0);
        self.wr16(NUMBER_END, NUMBER_TEXT);
        let (hl, c) = self.mantissa_digits(p.hl, p.c);   // F0B7
        let mut r = Fmt { hl, b: 0, c, d: flags, e: p.e };
        self.layout(&mut r);                              // EED4
        self.thousands(&mut r);                           // F03D
        r.e = r.b;
        r.b = p.b;                                        // POP BC: the sign
        if r.e == 0 { self.leading_zero(&mut r); }        // F050
        if r.d & 0x04 != 0 { self.prefix(&mut r, b'$'); } // F05F
        self.sign_char(&mut r);                           // F069
        self.pad_field(&mut r);                           // F07C
        if r.d & 1 != 0 {
            r.hl = r.hl.wrapping_sub(1);
            self.wr(r.hl, b'%');
        }
        r.hl
    }

    /// ROM `F0B7`: decimal digits of the `c`-byte binary integer whose most significant
    /// byte is at `msb`, ending at [`NUMBER_TEXT`].  Returns `(start, number of digits)`.
    fn mantissa_digits(&mut self, msb: u16, c: u8) -> (u16, u8) {
        let n = self.binary_to_bcd(msb, c);               // F0DD
        let mut hl = NUMBER_TEXT;
        let mut count = n.wrapping_add(n);
        if n == 0 { return (hl, count); }
        let mut de = DIGITS;
        let mut last = 0;
        for _ in 0..n {
            let v = self.rd(de);
            hl = hl.wrapping_sub(1);
            self.wr(hl, (v & 0x0F) + b'0');
            hl = hl.wrapping_sub(1);
            last = (v >> 4) + b'0';
            self.wr(hl, last);
            de = de.wrapping_add(1);
        }
        if last == b'0' {
            count = count.wrapping_sub(1);
            hl = hl.wrapping_add(1);
        }
        (hl, count)
    }

    /// ROM `F0DD`: convert a binary integer (`c` bytes, most significant at `msb`) to
    /// little-endian packed BCD at [`DIGITS`], bit by bit (doubling with `DAA`).
    /// Returns the number of BCD bytes (0 for a zero).
    fn binary_to_bcd(&mut self, msb: u16, c: u8) -> u8 {
        let mut src = msb;
        let mut c = c;
        let mut a: u8 = 0;
        loop {
            a |= self.rd(src);
            src = src.wrapping_sub(1);
            if a != 0 { break; }
            c = c.wrapping_sub(1);
            if c == 0 { return 0; }
        }
        // F0EA: shift out the top 1 bit (a sentinel 1 enters at the bottom); that bit is
        // the first BCD digit.
        let mut carry = true;
        loop {
            let out = a & 0x80 != 0;
            a = a << 1 | carry as u8;
            carry = out;
            if carry { break; }
        }
        let mut n: u8 = 1;
        self.wr(DIGITS, 1);                               // F104
        let mut d = a;
        loop {
            // F107: the remaining bits of this byte
            loop {
                let bit = d & 0x80 != 0;
                d <<= 1;
                if d == 0 { break; }                      // only the sentinel was left
                if self.bcd_double(n, bit) {              // F0F8
                    self.wr(DIGITS + n as u16, 1);        // F104
                    n += 1;
                }
            }
            c = c.wrapping_sub(1);
            if c == 0 { return n; }
            // F0F3: next byte
            let byte = self.rd(src);
            src = src.wrapping_sub(1);
            let bit = byte & 0x80 != 0;
            d = byte << 1 | 1;
            if self.bcd_double(n, bit) {
                self.wr(DIGITS + n as u16, 1);
                n += 1;
            }
        }
    }

    /// ROM `F0FA`: the `n`-byte BCD number at [`DIGITS`] times two plus `carry`
    /// (`ADC A,A` / `DAA` per byte).  Returns the carry out of the top byte.
    fn bcd_double(&mut self, n: u8, carry: bool) -> bool {
        let mut cy = carry;
        for i in 0..n as u16 {
            let v = self.rd(DIGITS + i);
            let (a, c) = daa_add(v, v, cy);
            self.wr(DIGITS + i, a);
            cy = c;
        }
        cy
    }

    // ---- layout ------------------------------------------------------------------------

    /// ROM `EED4`: choose the layout.
    fn layout(&mut self, r: &mut Fmt) {
        if r.d & 0x80 == 0 { return self.layout_free(r); }            // EF01
        if r.d & 0x40 != 0 { return self.layout_using_exponent(r); }  // EF27
        let t = r.e.wrapping_add(r.c).wrapping_sub(0x0A);
        if t & 0x80 != 0 {
            // EF88: fixed point
            self.leading_zeros(r);                        // EFB4
            return self.round_to_field(r);                // EF8B
        }
        // too many digits for a USING field: exponent form, flag the overflow
        r.d = 0x01;
        self.layout_exponent(r)
    }

    /// ROM `EEE4`: `d.ddddE+nn`.
    fn layout_exponent(&mut self, r: &mut Fmt) {
        r.b = r.c;
        if r.c != 0 {
            r.e = r.c.wrapping_add(r.e).wrapping_sub(1);
            self.strip_trailing_zeros(r);                 // F00E
            r.b = 1;
            if r.c >= 7 && r.d & 0x40 != 0 {
                // EF20: free format of a long number: 6 places after the point
                self.wr(FORMAT_A, 6);
                return self.layout_round_exponent(r, r.b);
            }
            if r.c != r.b { self.insert_point(r); }       // EFA0
        }
        self.append_exponent(r)                           // EF62
    }

    /// ROM `EF01`: free format (no `USING`).
    fn layout_free(&mut self, r: &mut Fmt) {
        if r.e & 0x80 != 0 { return self.layout_free_fraction(r); }  // EF0A
        if r.e != 0 { return self.layout_exponent(r); }
        r.b = r.c;                                        // EF08: an integer
    }

    /// ROM `EF0A`: free format of a number with a fraction.
    fn layout_free_fraction(&mut self, r: &mut Fmt) {
        r.b = r.e;
        self.strip_trailing_zeros(r);
        if r.b == 0 { r.b = r.c; return; }               // only zeros after the point
        let a = r.b.wrapping_sub(r.e);
        r.e = r.b;
        r.b = a;
        let s = a.wrapping_add(r.c).wrapping_add(r.e);
        if s & 0x80 != 0 { return self.layout_exponent(r); }
        self.leading_zeros(r);                            // EFB4
        self.insert_point(r);                             // EFA0
    }

    /// ROM `EF27`: a `USING` field with `^^^^`.
    fn layout_using_exponent(&mut self, r: &mut Fmt) {
        // Room for the digits: the field width less a sign position (computed as if
        // the number were negative: B = 80).
        let (room, borrow) = self.field_room(0x80, r.d); // F025
        let mut a = room;
        let mut zero = room == 0;
        if borrow { self.flag_overflow(r); a = 0; zero = true; }
        r.b = a;
        if zero {
            let f = self.places_after_point();            // F036
            if f.1 {
                r.b = r.b.wrapping_add(1);
                let places = self.rd(FORMAT_A);
                if places != 0 {
                    r.b = r.b.wrapping_sub(1);
                    self.wr(FORMAT_A, places.wrapping_add(1));
                }
            }
        }
        // EF44: (with no digits at all the ROM jumps on with A = C = 0)
        let before = if r.c != 0 {
            r.e = r.c.wrapping_add(r.e).wrapping_sub(r.b);
            r.b
        } else {
            0
        };
        self.layout_round_exponent(r, before)
    }

    /// ROM `EF4C`: round to `before` digits before the point and the places after it,
    /// adjust the exponent if rounding produced an extra digit, then the exponent.
    fn layout_round_exponent(&mut self, r: &mut Fmt, before: u8) {
        r.b = before;
        self.round_to_field(r);                           // EF8B
        if r.b != before {
            r.e = r.e.wrapping_add(1);
            r.hl = r.hl.wrapping_add(1);
            r.b = r.b.wrapping_sub(1);
            let mut p = r.hl;
            if self.rd(p) == b'.' { p = p.wrapping_add(1); }
            self.wr(p, b'1');
        }
        self.append_exponent(r)
    }

    /// ROM `EF62`: append `E`, the sign and two digits of the exponent `E`.
    fn append_exponent(&mut self, r: &mut Fmt) {
        self.append_char(b'E');
        let sign = if r.e & 0x80 != 0 { r.e = 0u8.wrapping_sub(r.e); b'-' } else { b'+' };
        self.append_char(sign);
        let mut tens = b'/';
        let mut a = r.e;
        loop {
            tens = tens.wrapping_add(1);
            let (v, borrow) = a.overflowing_sub(10);
            a = v;
            if borrow { break; }
        }
        r.e = a;
        r.c = tens;
        self.append_char(tens);
        self.append_char(a.wrapping_add(0x3A));
    }

    /// ROM `EF8B`: keep `B` digits before the point and the `USING` places after it:
    /// round away extra digits, or pad with zeros.  Then the point, if the field has one.
    fn round_to_field(&mut self, r: &mut Fmt) {
        let want = self.places_after_point().0.wrapping_add(r.b);
        if want < r.c {
            self.round_at(r, want);                       // EFC8
        } else {
            let pad = want.wrapping_sub(r.c);
            if pad != 0 { self.pad_zeros(r, pad); }       // EFEF
        }
        if self.rd(FORMAT_A) != 0 { self.insert_point(r); }
    }

    /// ROM `EFB4`: for a number below 1, put zeros before the digits so that `B` (digits
    /// before the point) is 0; otherwise `B` = digits before the point.
    fn leading_zeros(&mut self, r: &mut Fmt) {
        let a = r.e.wrapping_add(r.c);
        r.b = a;
        if a & 0x80 == 0 { return; }
        let n = 0u8.wrapping_sub(a);
        let mut count = if n >= 0x14 { 0x14 } else { n };
        loop {
            r.hl = r.hl.wrapping_sub(1);
            self.wr(r.hl, b'0');
            r.c = r.c.wrapping_add(1);
            count = count.wrapping_sub(1);
            if count == 0 { break; }
        }
        r.b = 0;
    }

    /// ROM `EFC8`: cut the digits after position `n` and round (a `1` is put in front if
    /// all digits rolled over, which adds a digit before the point).
    fn round_at(&mut self, r: &mut Fmt, n: u8) {
        r.c = n;
        let cut = r.hl.wrapping_add(n as u16);
        let next = self.rd(cut);
        self.wr(cut, 0);
        self.wr16(NUMBER_END, cut);
        if next < b'5' { return; }
        // EFE1: increment from the last digit kept (C counts down as the carry moves)
        let mut p = cut;
        loop {
            if r.c == 0 {
                r.hl = r.hl.wrapping_sub(1);
                self.wr(r.hl, b'1');
                r.b = r.b.wrapping_add(1);
                return;
            }
            p = p.wrapping_sub(1);
            r.c -= 1;
            let d = self.rd(p);
            self.wr(p, d.wrapping_add(1));
            if d < b'9' { return; }
            self.wr(p, b'0');
        }
    }

    /// ROM `EFEF`: move the digits `n` places left and append `n` zeros.
    fn pad_zeros(&mut self, r: &mut Fmt, n: u8) {
        let old = r.hl;
        let new = old.wrapping_sub(n as u16);
        for i in 0..r.c as u16 {
            let v = self.rd(old.wrapping_add(i));
            self.wr(new.wrapping_add(i), v);
        }
        for i in 0..n as u16 { self.wr(new.wrapping_add(r.c as u16 + i), b'0'); }
        r.hl = new;
    }

    /// ROM `F00E`: remove trailing zeros (counting `C` down and `B` up, stopping when `B`
    /// reaches 0).
    fn strip_trailing_zeros(&mut self, r: &mut Fmt) {
        let mut p = self.rd16(NUMBER_END);
        loop {
            if self.rd(p.wrapping_sub(1)) != b'0' { break; }
            p = p.wrapping_sub(1);
            r.c = r.c.wrapping_sub(1);
            r.b = r.b.wrapping_add(1);
            if r.b == 0 { break; }
        }
        self.wr(p, 0);
        self.wr16(NUMBER_END, p);
    }

    /// ROM `EFA0`: put the decimal point after `B` digits.
    fn insert_point(&mut self, r: &mut Fmt) {
        let b = r.b;
        self.insert_char(r, b, b'.');
    }

    /// ROM `EFA3`: insert `ch` after the first `pos` characters (the text grows to the
    /// left).
    fn insert_char(&mut self, r: &mut Fmt, pos: u8, ch: u8) {
        let mut p = r.hl.wrapping_add(pos as u16);
        let mut carry = ch;
        for _ in 0..=pos as u16 {
            p = p.wrapping_sub(1);
            let old = self.rd(p);
            self.wr(p, carry);
            carry = old;
        }
        r.hl = p;
    }

    /// ROM `F06F`: append a character at [`NUMBER_END`].
    fn append_char(&mut self, c: u8) {
        let p = self.rd16(NUMBER_END);
        self.wr(p, c);
        self.wr(p.wrapping_add(1), 0);
        self.wr16(NUMBER_END, p.wrapping_add(1));
    }

    /// ROM `F03D`: thousands separators (flag bit 1).
    fn thousands(&mut self, r: &mut Fmt) {
        if r.d & 0x02 == 0 { return; }
        let mut a = r.b;
        loop {
            let (v, borrow) = a.overflowing_sub(3);
            if borrow || v == 0 { return; }
            a = v;
            self.insert_char(r, a, b',');
            r.b = r.b.wrapping_add(1);
        }
    }

    /// ROM `F036`: places after the point less one (the point itself is counted in
    /// [`FORMAT_A`]): `(value, zero flag)`.
    fn places_after_point(&self) -> (u8, bool) {
        let a = self.rd(FORMAT_A);
        if a == 0 { return (0, true); }
        let a = a - 1;
        (a, a == 0)
    }

    /// ROM `F025`: room in the field for digits: [`FORMAT_B`] less one if a sign goes in
    /// front, less one more for `$$`.  `(room, borrow)`.
    fn field_room(&self, sign: u8, d: u8) -> (u8, bool) {
        let (none, trailing, _) = sign_decision(sign, d);
        let mut b: u8 = if none || trailing { 0 } else { 1 };
        if d & 0x04 != 0 { b += 1; }
        self.rd(FORMAT_B).overflowing_sub(b)
    }

    /// ROM `F096`: the number does not fit its field.
    fn flag_overflow(&self, r: &mut Fmt) { r.d |= 1; }

    /// ROM `F050`: a `0` before the point when there are no digits before it - unless a
    /// `USING` field has no room for it.
    fn leading_zero(&mut self, r: &mut Fmt) {
        if r.d & 0x80 != 0 {
            let (room, borrow) = self.field_room(r.b, r.d);
            if borrow || room == 0 { return; }
        }
        self.prefix(r, b'0');
    }

    /// ROM `F065`: put a character in front.
    fn prefix(&mut self, r: &mut Fmt, c: u8) {
        r.e = r.e.wrapping_add(1);
        r.hl = r.hl.wrapping_sub(1);
        self.wr(r.hl, c);
    }

    /// ROM `F069`: the sign: in front, at the end, or none.
    fn sign_char(&mut self, r: &mut Fmt) {
        let (none, trailing, c) = sign_decision(r.b, r.d);
        if none { return; }
        if trailing { self.append_char(c) } else { self.prefix(r, c) }
    }

    /// ROM `F07C`: pad a `USING` field on the left with spaces (or `*` for `**`); flag the
    /// overflow if the number is wider than the field.
    fn pad_field(&mut self, r: &mut Fmt) {
        if r.d & 0x80 == 0 { return; }
        let (n, borrow) = self.rd(FORMAT_B).overflowing_sub(r.e);
        if n == 0 { return; }
        if borrow { return self.flag_overflow(r); }
        let c = if r.d & 0x20 != 0 { b'*' } else { b' ' };
        for _ in 0..n {
            r.hl = r.hl.wrapping_sub(1);
            self.wr(r.hl, c);
        }
    }

    // ===============================================================================
    // Binary and hexadecimal digits   (F114 .. F157)
    // ===============================================================================

    /// ROM `F114`: binary digits of the value of type `t` at `src` (at least `min`
    /// digits).  Returns `(src after the value, start of the digits)`; the digits end at
    /// `AE57`.
    pub fn binary_digits(&mut self, src: u16, t: u8, min: u8) -> Res<(u16, u16)> {
        self.based_text(src, t, min, 1, 0x01)
    }

    /// ROM `F119`: hexadecimal digits, as [`Self::binary_digits`].
    pub fn hex_digits(&mut self, src: u16, t: u8, min: u8) -> Res<(u16, u16)> {
        self.based_text(src, t, min, 4, 0x0F)
    }

    /// ROM `F11C`: digits of an unsigned 16-bit value, `bits` per digit, written right
    /// to left from `AE57`.
    fn based_text(&mut self, src: u16, t: u8, min: u8, bits: u8, mask: u8) -> Res<(u16, u16)> {
        self.set_acc_from(src, t);
        let after = src.wrapping_add(t as u16);
        let mut v = self.acc_to_unsigned()?;              // FEC2
        let mut de: u16 = 0xAE57;
        self.wr(de, 0);
        let mut left = min;
        loop {
            left = left.saturating_sub(1);                // SUB 1 / ADC A,0
            let digit = (v as u8) & mask;
            de = de.wrapping_sub(1);
            self.wr(de, b"0123456789ABCDEF"[digit as usize]);
            v &= !(mask as u16);
            if v == 0 {
                if left == 0 { return Ok((after, de)); }
                continue;
            }
            v >>= bits;
        }
    }
}

/// ROM `F09B`: which sign character the number gets: `(none, at the end, character)`.
/// `sign` bit 7 = negative.  A `USING` field without `+` or a trailing sign prints no
/// sign for a positive number; free format prints a space.
fn sign_decision(sign: u8, d: u8) -> (bool, bool, u8) {
    let c = if sign & 0x80 != 0 {
        b'-'
    } else {
        let a = (d & 0x98) ^ 0x80;
        if a == 0 { return (true, true, 0); }
        if a & 0x08 != 0 { b'+' } else { b' ' }
    };
    (false, d & 0x10 != 0, c)
}

/// `ADC A,r` with `A = r = v` followed by `DAA`: returns `(A, carry)`.
fn daa_add(a: u8, b: u8, carry: bool) -> (u8, bool) {
    let sum = a as u16 + b as u16 + carry as u16;
    let half = (a & 0x0F) + (b & 0x0F) + carry as u8 > 0x0F;
    let mut c = sum > 0xFF;
    let mut v = sum as u8;
    let mut corr = 0u8;
    if half || v & 0x0F > 9 { corr |= 0x06; }
    if c || v > 0x99 { corr |= 0x60; c = true; }
    v = v.wrapping_add(corr);
    (v, c)
}

/// What `FCB3`/`FCC3` hand to the formatter: `HL` = most significant byte of the binary
/// mantissa, `B` = sign (bit 7), `C` = its length in bytes, `E` = decimal exponent.
#[derive(Clone, Copy, Debug)]
struct Prep {
    hl: u16,
    b: u8,
    c: u8,
    e: u8,
}

/// The formatter's registers.
#[derive(Clone, Copy, Debug)]
struct Fmt {
    hl: u16,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
}

/// The digit collector's registers.
struct Digits {
    hl: u16,
    b: u8,
    c: u8,
    de: u16,
}
