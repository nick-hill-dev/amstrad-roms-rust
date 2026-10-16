//! # `PEEK`/`POKE`, `INP`/`OUT`/`WAIT`, `CALL`, RSXs, `ZONE`, `PRINT [USING]`, `WRITE`  (ROM `F158 .. F4C3`)
//!
//! ## `CALL` and RSXs
//!
//! `CALL address[,p1...]` and `|NAME[,p1...]` push up to 32 parameters (16-bit
//! integers or addresses) on the Z80 stack, point `IX` at the last one, put the count
//! in `A` and call the routine (an RSX through `KL FIND COMMAND`, with its ROM).  The
//! Rust port writes the parameters where the Z80 stack would hold them - BASIC's stack
//! pointer is always `BFFE` while a statement runs - and hands the call to
//! [`Hardware::far_call`].  The routine may change the saved text pointer
//! ([`CALL_SAVED_PTR`]); BASIC continues from there.
//!
//! ## `PRINT`
//!
//! Items are expressions (numbers are printed with a leading sign position and a
//! trailing space), `;` (nothing), `,` (to the next print zone, [`ZONE`]), `SPC(n)`
//! and `TAB(n)` (both taken modulo the line width).  An item that does not fit on the
//! rest of the line starts a new one.  A new line is printed unless the statement ends
//! with `;` or `,`.
//!
//! Quirk: the dispatch table of the item separators (`F224`) says it has five entries but
//! only holds four.  The fifth is read from the code that follows it, `CALL CEFB`, so the
//! token `SPEED` (`CD`, the opcode byte) dispatches to the expression evaluator (`CEFB`)
//! without printing.  It has no visible effect: like every item handler the evaluator is
//! entered *on* the token, and `SPEED` is not an operand, so this is a *Syntax error* -
//! exactly what treating `SPEED` as an ordinary item would give.
//!
//! ## `PRINT USING`
//!
//! The format string is scanned for fields: `!` (first character), `&` (whole string),
//! `\  \` (fixed width), and numeric fields (see [`Basic::number_field`]); other
//! characters are printed as they are, `_` quotes the next character.  The items are
//! matched to the fields in order, starting the format again when it runs out; after
//! the last item the literal text up to the next field is printed.

use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::{Hardware, Regs};

/// BASIC's Z80 stack pointer while a statement runs.
const STATEMENT_SP: u16 = 0xBFFE;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Memory and ports   (F158 .. F19F)
    // ===============================================================================

    /// `PEEK(address)` (ROM `F158`): always the RAM (`RST 20`, `RAM LAM`).
    pub fn fn_peek(&mut self) -> Res {
        let a = self.acc_to_unsigned()?;                  // FEC2
        let v = self.ram(a);
        self.set_acc_int(v as u16);
        Ok(())
    }

    /// `POKE address,value` (ROM `F15F`).
    pub fn stmt_poke(&mut self) -> Res {
        let a = self.eval_address()?;
        self.expect(b',')?;
        let v = self.eval_byte()?;
        self.wr(a, v);
        Ok(())
    }

    /// `INP(port)` (ROM `F16D`).
    pub fn fn_inp(&mut self) -> Res {
        let port = self.acc_to_int()?;
        let v = self.cpc.hw.inp(port);
        self.set_acc_int(v as u16);
        Ok(())
    }

    /// `OUT port,value` (ROM `F177`).
    pub fn stmt_out(&mut self) -> Res {
        let (port, v) = self.port_and_byte()?;            // F194
        self.cpc.hw.out(port, v);
        Ok(())
    }

    /// `WAIT port,mask[,inversion]` (ROM `F17D`): wait until `(IN port XOR inversion)
    /// AND mask` is not zero.
    pub fn stmt_wait(&mut self) -> Res {
        let (port, mask) = self.port_and_byte()?;
        let mut inv = 0;
        if !crate::is_end(self.cur()) {
            self.expect(b',')?;
            inv = self.eval_byte()?;
        }
        while (self.cpc.hw.inp(port) ^ inv) & mask == 0 {}
        Ok(())
    }

    /// ROM `F194`: `port,byte`.
    fn port_and_byte(&mut self) -> Res<(u16, u8)> {
        let port = self.eval_address()?;
        self.expect(b',')?;
        Ok((port, self.eval_byte()?))
    }

    // ===============================================================================
    // RSX, CALL, ZONE   (F1A0 .. F1FC)
    // ===============================================================================

    /// ROM `F1A0`: `|NAME[,params]` - an external command (*Unknown command* if no ROM
    /// or RSX has it).
    pub fn call_rsx(&mut self) -> Res {
        let p = self.ptr.wrapping_add(1);
        if self.rd(p) != 0 { return Err(Exit::Error(0x1C)); }
        let name = p.wrapping_add(1);
        let crate::fw::FoundCommand::Extension(addr, rom) = self.kl_find_command(name) else {
            return Err(Exit::Error(0x1C));
        };
        let mut q = name;
        loop {
            let c = self.rd(q);
            q = q.wrapping_add(1);
            if c & 0x80 != 0 { break; }
        }
        self.ptr = q;
        self.call_with_parameters(addr, rom)
    }

    /// `CALL address[,params]` (ROM `F1BA`).
    pub fn stmt_call(&mut self) -> Res {
        let addr = self.eval_address()?;
        self.call_with_parameters(addr, 0xFF)
    }

    /// ROM `F1BF`: evaluate up to 32 parameters, call `addr` (ROM select `rom`) and
    /// continue at the text position saved in [`CALL_SAVED_PTR`].
    fn call_with_parameters(&mut self, addr: u16, rom: u8) -> Res {
        self.wr16(CALL_ADDRESS, addr);
        self.wr(CALL_ROM, rom);
        self.wr16(CALL_SAVED_SP, STATEMENT_SP);
        let mut sp = STATEMENT_SP;
        let mut left: u8 = 0x20;
        let mut last = 0;
        while self.comma() {
            last = self.eval_address()?;
            sp = sp.wrapping_sub(2);
            self.cpc.wr16(sp, last);
            left -= 1;
            if left == 0 { break; }
        }
        self.check_end()?;
        self.wr16(CALL_SAVED_PTR, self.ptr);
        let mut r = Regs { hl: self.ptr, de: last, bc: (left as u16) << 8 | rom as u16, ix: sp, ..Regs::default() };
        r.set_a(0x20 - left);
        let addr = self.rd16(CALL_ADDRESS);
        let rom = self.rd(CALL_ROM);
        self.cpc.hw.far_call(addr, rom, &mut r);
        self.ptr = self.rd16(CALL_SAVED_PTR);
        Ok(())
    }

    /// ROM `F1F2`: `ZONE 13`.
    pub fn reset_zone(&mut self) { self.wr(ZONE, 13); }

    /// `ZONE n` (ROM `F1F6`).
    pub fn stmt_zone(&mut self) -> Res {
        let n = self.eval_nonzero_byte()?;
        self.wr(ZONE, n);
        Ok(())
    }

    // ===============================================================================
    // PRINT   (F1FD .. F2C3)
    // ===============================================================================

    /// `PRINT [#s,]items` (ROM `F1FD`).
    pub fn stmt_print(&mut self) -> Res {
        let old = self.output_stream_prefix()?;           // C1C6
        self.print_items()?;
        self.set_output_stream(old);
        Ok(())
    }

    /// ROM `F208`.
    fn print_items(&mut self) -> Res {
        if crate::is_end(self.cur()) { return self.new_line(); }
        loop {
            match self.cur() {
                0xED => return self.print_using(),        // F2C4
                b',' => self.print_comma()?,              // F25C
                0xE5 => self.print_spc()?,                // F277
                0xEA => self.print_tab()?,                // F280
                b';' => { self.next(); }                  // DD3F
                0xCD => self.eval()?,                     // the phantom fifth entry (CEFB): Syntax error
                _ => self.print_expression()?,            // F233
            }
            if crate::is_end(self.cur()) { return Ok(()); }
        }
    }

    /// ROM `F233`: an expression; a number is printed with a trailing space.  New line
    /// first if it does not fit; new line after it if the statement ends here.
    fn print_expression(&mut self) -> Res {
        self.eval()?;
        let at_end = crate::is_end(self.cur());
        if !self.acc_is_string() {
            let digits = self.format_acc_print(self.ptr)?;    // EE9D
            let text = self.ptr;
            let end = self.buffer_string(digits)?;        // F7DC
            self.ptr = text;
            self.wr(end, b' ');
            let desc = self.rd16(ACC);
            let len = self.rd(desc).wrapping_add(1);
            self.wr(desc, len);
        }
        let len = self.rd(self.rd16(ACC));
        if !self.fits_on_line(len) { self.new_line()?; }  // C2B9
        self.print_acc_string()?;                         // F828
        if at_end { self.new_line()?; }
        Ok(())
    }

    /// ROM `F25C`: `,` - to the next print zone (a new line if the zone after that
    /// would not fit).
    fn print_comma(&mut self) -> Res {
        self.next();
        let zone = self.rd(ZONE);
        let mut a = self.column().wrapping_sub(1);
        loop {
            let (r, borrow) = a.overflowing_sub(zone);
            a = r;
            if borrow { break; }
        }
        let spaces = (!a).wrapping_add(1);
        if !self.fits_on_line(spaces.wrapping_add(zone)) { return self.new_line(); }
        self.print_spaces(spaces)
    }

    /// `SPC(n)` (ROM `F277`).
    fn print_spc(&mut self) -> Res {
        let n = self.print_function_argument()?;          // F2A0
        let n = self.modulo_width(n);                     // F2AF
        self.print_spaces(n as u8)
    }

    /// `TAB(n)` (ROM `F280`): to column `n` (on the next line if already past it).
    fn print_tab(&mut self) -> Res {
        let n = self.print_function_argument()?.wrapping_sub(1);
        let e = self.modulo_width(n) as u8;
        let col = self.column();
        let (a, carry) = (!col).wrapping_add(1).overflowing_add(e.wrapping_add(1));
        if carry { return self.print_spaces(a); }
        self.new_line()?;
        self.print_spaces(e)
    }

    /// ROM `F295`: `n` spaces.
    fn print_spaces(&mut self, n: u8) -> Res {
        for _ in 0..n { self.print_char(b' ')?; }
        Ok(())
    }

    /// ROM `F2A0`: `(integer)` after `SPC` / `TAB`.
    fn print_function_argument(&mut self) -> Res<u16> {
        self.next();
        self.expect(b'(')?;
        let n = self.eval_int()?;
        self.expect(b')')?;
        Ok(n)
    }

    /// ROM `F2AF`: a negative count is 0; then modulo the line width (if there is one).
    fn modulo_width(&mut self, n: u16) -> u16 {
        let n = if n & 0x8000 != 0 { 0 } else { n };
        let Some(w) = self.line_width() else { return n };   // C29F
        let (_, rem) = self.int_div_unsigned(n, w as u16);
        rem
    }

    // ===============================================================================
    // PRINT USING   (F2C4 .. F3B9)
    // ===============================================================================

    /// ROM `F2C4`: `USING format;items`.  *Improper argument* for an empty format or one
    /// without any field.
    fn print_using(&mut self) -> Res {
        self.next();
        self.eval_string_keep()?;                         // CEA5
        self.expect(b';')?;
        let format = self.rd16(ACC);
        if self.rd(format) == 0 { return Err(Exit::Error(5)); }
        self.eval()?;
        self.wr(USING_DONE, 0);
        let (mut b, mut de) = (self.rd(format), self.rd16(format + 1));
        if !self.using_pass(&mut b, &mut de)? { return Err(Exit::Error(5)); }
        let newline = loop {
            // F2ED
            let c = self.cur();
            if crate::is_end(c) { break true; }
            if c != b';' && c != b',' { return Err(Exit::Error(2)); }
            if crate::is_end(self.next()) { break false; }
            self.eval()?;
            if b == 0 || !self.using_pass(&mut b, &mut de)? {
                // F2DE: start the format again.
                b = self.rd(format);
                de = self.rd16(format + 1);
                if !self.using_pass(&mut b, &mut de)? { return Err(Exit::Error(5)); }
            }
        };
        // F30F: the literal text up to the next field.
        self.wr(USING_DONE, 0xFF);
        if b != 0 { self.using_pass(&mut b, &mut de)?; }
        if newline { self.new_line()?; }
        self.release_string(format);                      // FBE8
        Ok(())
    }

    /// ROM `F324`: print the format from `de` (`b` characters left) up to and including
    /// the next field (which formats the accumulator unless [`USING_DONE`] is set).
    /// `false` if the format ended without a field.
    fn using_pass(&mut self, b: &mut u8, de: &mut u16) -> Res<bool> {
        loop {
            let c = self.rd(*de);
            if c == b'_' {
                if *b >= 2 {
                    *de = de.wrapping_add(1);
                    *b -= 1;
                }
            } else if self.string_field(b, de)? || self.numeric_field(b, de)? {
                return Ok(true);
            }
            let c = self.rd(*de);
            self.print_char(c)?;
            *de = de.wrapping_add(1);
            *b -= 1;
            if *b == 0 { return Ok(false); }
        }
    }

    /// ROM `F350`: `!`, `&` or `\  \` at `de`: print the string accumulator in it.
    fn string_field(&mut self, b: &mut u8, de: &mut u16) -> Res<bool> {
        let width = match self.rd(*de) {
            b'!' => 1,
            b'&' => 0,
            b'\\' => {
                let (mut d, mut n, mut w) = (*de, *b, 2u8);
                loop {
                    d = d.wrapping_add(1);
                    n -= 1;
                    if n == 0 { return Ok(false); }
                    let c = self.rd(d);
                    if c == b'\\' { break; }
                    w = w.wrapping_add(1);
                    if c != b' ' { return Ok(false); }
                }
                *de = d;
                *b = n;
                w
            }
            _ => return Ok(false),
        };
        // F378
        *de = de.wrapping_add(1);
        *b -= 1;
        if self.rd(USING_DONE) != 0 { return Ok(true); }
        self.require_string()?;
        if width != 0 {
            let desc = self.rd16(ACC);
            self.substring(desc, width, 0)?;              // F971
        }
        self.print_acc_string()?;
        if width != 0 {
            let len = self.rd(self.rd16(ACC));
            self.print_spaces(width.wrapping_sub(len))?;
        }
        Ok(true)
    }

    /// ROM `F3A3`: a numeric field at `de`: print the accumulator formatted by it.
    fn numeric_field(&mut self, b: &mut u8, de: &mut u16) -> Res<bool> {
        let Some((nde, nb, flags, field)) = self.number_field(*de, *b)? else { return Ok(false) };
        *de = nde;
        *b = nb;
        if self.rd(USING_DONE) != 0 { return Ok(true); }
        let text = self.format_acc(flags, field)?;        // EE9F
        self.print_string(text)?;
        Ok(true)
    }

    /// ROM `F3BA` for `DEC$`: the format of `len` characters at `fmt` must be a numeric
    /// field: `(characters left over, flags, field)`.
    pub fn parse_number_field(&mut self, fmt: u16, len: u8) -> Res<Option<(u8, u8, u16)>> {
        Ok(self.number_field(fmt, len)?.map(|(_, b, c, f)| (b, c, f)))
    }

    /// ROM `F3BA`: parse a numeric field at `de` (`b` characters left):
    ///
    /// ```text
    ///   [+] [$$|**|**$] [#...] [,] [.#...] [^^^^] [+|-]
    /// ```
    ///
    /// Returns `(de, b after it, flags, H = digit positions before the point, L =
    /// after)`; the flags are `80` plus `08` leading `+`, `04` `$$`, `20` `**`, `24`
    /// `**$`, `02` commas, `40` exponent, `10` trailing `-`, `18` trailing `+`.  More
    /// than 20 positions is *Improper argument*.
    pub fn number_field(&mut self, mut de: u16, mut b: u8) -> Res<Option<(u16, u8, u8, u16)>> {
        let mut c: u8 = 0x80;
        let mut h: u8 = 0;
        let mut l: u8 = 0;
        if self.rd(de) == b'+' {
            de = de.wrapping_add(1);
            b -= 1;
            if b == 0 { return Ok(None); }
            h += 1;
            c = 0x88;
        }
        let a = self.rd(de);
        match a {
            b'.' => {
                if b == 1 || self.rd(de.wrapping_add(1)) != b'#' { return Ok(None); }
            }
            b'#' => {}
            _ => {
                de = de.wrapping_add(1);
                b -= 1;
                if b == 0 || self.rd(de) != a { return Ok(None); }
                h += 2;
                if a == b'$' {
                    l = 0x04;
                    de = de.wrapping_add(1);
                    b -= 1;
                } else if a == b'*' {
                    l = 0x20;
                    de = de.wrapping_add(1);
                    b -= 1;
                    if b != 0 && self.rd(de) == b'$' {
                        h += 1;
                        l = 0x24;
                        de = de.wrapping_add(1);
                        b -= 1;
                    }
                } else {
                    return Ok(None);
                }
                c |= l;
                l = 0;
            }
        }
        // F41B: the digit positions.
        if b != 0 {
            let mut point = false;
            loop {
                let ch = self.rd(de);
                if !point && ch == b'#' { h += 1; }
                else if !point && ch == b',' { c |= 0x02; h += 1; }
                else if ch == b'.' && !point { point = true; l += 1; }
                else if point && ch == b'#' { l += 1; }
                else { break; }
                de = de.wrapping_add(1);
                b -= 1;
                if b == 0 { break; }
            }
            if b != 0 {
                // F442: an exponent, then a trailing sign.
                if self.rd(de) == b'^' && (1..4).all(|k| self.rd(de.wrapping_add(k)) == b'^') && b >= 4 {
                    b -= 4;
                    de = de.wrapping_add(4);
                    c |= 0x40;
                }
                if b != 0 && c & 0x08 == 0 {
                    let s = self.rd(de);
                    if s == b'-' || s == b'+' {
                        c |= if s == b'-' { 0x10 } else { 0x18 };
                        de = de.wrapping_add(1);
                        b -= 1;
                    }
                }
            }
        }
        if h.wrapping_add(l) >= 0x15 { return Err(Exit::Error(5)); }
        Ok(Some((de, b, c, (h as u16) << 8 | l as u16)))
    }

    // ===============================================================================
    // WRITE   (F47B .. F4C3)
    // ===============================================================================

    /// `WRITE [#s,]items` (ROM `F47B`): items separated by commas, strings quoted.
    pub fn stmt_write(&mut self) -> Res {
        let old = self.output_stream_prefix()?;
        if !crate::is_end(self.cur()) {
            loop {
                self.eval()?;
                if self.acc_is_string() {
                    self.print_char(b'"')?;
                    self.print_acc_string()?;
                    self.print_char(b'"')?;
                } else {
                    let text = self.ptr;
                    let digits = self.format_acc_trimmed_hl(text)?;   // EE8F
                    self.buffer_string(digits)?;
                    self.ptr = text;
                    self.print_acc_string()?;
                }
                let c = self.cur();
                if crate::is_end(c) { break; }
                if c != b';' && c != b',' { return Err(Exit::Error(2)); }
                self.next();
                self.print_char(b',')?;
            }
        }
        self.new_line()?;
        self.set_output_stream(old);
        Ok(())
    }
}
