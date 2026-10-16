//! # Strings  (ROM `F7CB .. FCB2`)
//!
//! ## Descriptors
//!
//! A string value is a three-byte *descriptor*: length, address.  A string variable
//! holds a descriptor; a string in the accumulator is the *address of* a descriptor
//! ([`ACC`] holds it, [`ACC_TYPE`] is 3).  The text itself may live
//!
//! * in the program (a literal: `a$="x"` points the variable straight into the program
//!   text),
//! * in the string space below `HIMEM` (built strings), or
//! * anywhere else (the input buffer, ROM) - such strings are copied before a variable
//!   keeps them.
//!
//! ## Temporaries
//!
//! The results of string expressions are *temporary* descriptors on a small stack
//! (`B09C .. B0B9`, ten entries, [`DESCRIPTOR_SP`] the next free one).  A new string is
//! built in [`TEMP_DESCRIPTOR`] (`B0BA`) and pushed with [`Basic::push_temp_string`]; more
//! than ten pending temporaries is *String expression too complex*.  Using a string
//! "releases" it ([`Basic::release_string`]): if it is the newest temporary it is
//! popped, and if its text is the lowest string of the string space that space is given
//! back at once.  The whole stack is emptied between statements.
//!
//! ## String space and garbage
//!
//! Strings are allocated downwards from [`STRINGS_TOP`]; [`STRINGS_BOTTOM`] is the byte
//! below the lowest one.  When the space runs out the garbage collector
//! ([`Basic::collect_garbage`]) moves every string still referenced (by a variable, an
//! array element, a `DEF FN` parameter or a temporary) to the top, highest address first,
//! fixing each descriptor.  It rescans all strings for every string it moves.

use crate::memory::StringVisitor;
use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// The first temporary descriptor.
const DESCRIPTORS: u16 = 0xB09C;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Strings in the text   (F7CB .. F827)
    // ===============================================================================

    /// ROM `F7CB`: a string literal at the text pointer (on the opening quote).  The
    /// result points into the program text.  An unterminated literal ends at the end of
    /// the line, without its trailing white space.
    pub fn string_literal(&mut self) -> Res {
        let start = self.ptr.wrapping_add(1);
        let mut p = start;
        let mut len = 0u8;
        loop {
            let c = self.rd(p);
            if c == b'"' {
                self.ptr = p;
                self.next();                              // JP DD3F
                break;
            }
            if c == 0 {
                len = self.trim_trailing_white(p, len);   // F80F
                self.ptr = p;
                break;
            }
            len = len.wrapping_add(1);
            p = p.wrapping_add(1);
        }
        self.text_string(start, len)                      // F800
    }

    /// ROM `F7DC`: the zero-terminated text at `p` as a string (in place).  Returns the
    /// address of the terminator.
    pub fn buffer_string(&mut self, p: u16) -> Res<u16> {
        let mut e = p;
        let mut len = 0u8;
        while self.rd(e) != 0 {
            e = e.wrapping_add(1);
            len = len.wrapping_add(1);
        }
        self.text_string(p, len)?;
        Ok(e)
    }

    /// ROM `F7E6`: an unquoted `INPUT` item at `p`, ending at a zero, at `end`, or at a
    /// comma, without trailing white space (in place).  Returns where it ended.
    pub fn item_string(&mut self, p: u16, end: u8) -> Res<u16> {
        let mut e = p;
        let mut len = 0u8;
        loop {
            let c = self.rd(e);
            if c == 0 || c == end || c == b',' { break; }
            e = e.wrapping_add(1);
            len = len.wrapping_add(1);
        }
        let len = self.trim_trailing_white(e, len);
        self.text_string(p, len)?;
        Ok(e)
    }

    /// ROM `F800`: the `len` characters at `start` become a temporary string.
    fn text_string(&mut self, start: u16, len: u8) -> Res {
        self.set_temp_descriptor(len, start);
        self.push_temp_string()                           // FBBA
    }

    /// ROM `F80F`: drop spaces, TABs, CRs and LFs from the end of the `len` characters
    /// that end before `end`.
    fn trim_trailing_white(&self, end: u16, mut len: u8) -> u8 {
        let mut p = end;
        while len != 0 {
            p = p.wrapping_sub(1);
            if !matches!(self.rd(p), b' ' | 0x09 | 0x0D | 0x0A) { break; }
            len -= 1;
        }
        len
    }

    // ===============================================================================
    // Printing, case conversion, + and comparison   (F828 .. F8B9)
    // ===============================================================================

    /// ROM `F828`: print the string in the accumulator (characters sent as they are).
    pub fn print_acc_string(&mut self) -> Res {
        let (len, mut addr) = self.pop_string()?;
        for _ in 0..len {
            let c = self.rd(addr);
            addr = addr.wrapping_add(1);
            self.output_raw(c)?;                          // C36E
        }
        Ok(())
    }

    /// `LOWER$(s)` (ROM `F834`).
    pub fn fn_lower(&mut self) -> Res { self.convert_case(false) }

    /// `UPPER$(s)` (ROM `F842`).
    pub fn fn_upper(&mut self) -> Res { self.convert_case(true) }

    /// ROM `F845`: a copy of the string with each letter converted.  (The new string is
    /// allocated before the argument is released.)
    fn convert_case(&mut self, upper: bool) -> Res {
        let desc = self.rd16(ACC);
        let len = self.rd(desc);
        let mut dst = self.alloc_temp_string(len)?;       // FC19
        let (_, mut src) = self.pop_string()?;
        for _ in 0..len {
            let c = self.rd(src);
            src = src.wrapping_add(1);
            let c = if upper { crate::arith::to_upper(c) } else if c.is_ascii_uppercase() { c + 0x20 } else { c };
            self.wr(dst, c);
            dst = dst.wrapping_add(1);
        }
        self.push_temp_string()
    }

    /// ROM `F863`: `left + accumulator` (`left` = the left operand's descriptor).
    /// *String too long* beyond 255 characters.
    pub fn concatenate(&mut self, left: u16) -> Res {
        let right = self.rd16(ACC);
        let (total, overflow) = self.rd(left).overflowing_add(self.rd(right));
        if overflow { return Err(Exit::Error(0x0F)); }
        let mut dst = self.alloc_temp_string(total)?;
        let (rlen, raddr) = self.pop_string()?;
        let (llen, laddr) = self.release_string(left);
        self.ldir(laddr, dst, llen as u16);
        dst = dst.wrapping_add(llen as u16);
        self.ldir(raddr, dst, rlen as u16);
        self.push_temp_string()
    }

    /// ROM `F897`: compare `left` (a descriptor) with the string in the accumulator:
    /// `1` if left is smaller, `FF` if larger, `0` if equal (a prefix is smaller).
    pub fn compare_strings(&mut self, left: u16) -> Res<u8> {
        let (mut c, mut r) = self.pop_string()?;
        let (mut b, mut l) = self.release_string(left);
        loop {
            if b == 0 && c == 0 { return Ok(0); }
            if c == 0 { return Ok(0xFF); }
            if b == 0 { return Ok(1); }
            b -= 1;
            c -= 1;
            let x = self.rd(l);
            let y = self.rd(r);
            l = l.wrapping_add(1);
            r = r.wrapping_add(1);
            if x != y { return Ok(if x < y { 1 } else { 0xFF }); }
        }
    }

    // ===============================================================================
    // BIN$, HEX$, DEC$, STR$   (F8BA .. F93B)
    // ===============================================================================

    /// `BIN$(n[,digits])` (ROM `F8BA`).
    pub fn fn_bin(&mut self) -> Res {
        let (src, t, min) = self.based_arguments()?;      // F8CE
        let text = self.ptr;
        let (_, digits) = self.binary_digits(src, t, min)?;   // F114
        self.cstring_result(digits)?;                     // F922
        self.ptr = text;
        Ok(())
    }

    /// `HEX$(n[,digits])` (ROM `F8C4`).
    pub fn fn_hex(&mut self) -> Res {
        let (src, t, min) = self.based_arguments()?;
        let text = self.ptr;
        let (_, digits) = self.hex_digits(src, t, min)?;  // F119
        self.cstring_result(digits)?;
        self.ptr = text;
        Ok(())
    }

    /// ROM `F8CE`: `n[,digits])`, digits 0..16: `(address of n, its type, digits)`.
    fn based_arguments(&mut self) -> Res<(u16, u8, u8)> {
        self.eval()?;
        let t = self.push_acc()?;
        let min = if self.comma() { self.eval_byte()? } else { 0 };
        if min >= 0x11 { return Err(Exit::Error(5)); }
        self.expect(b')')?;
        let src = self.pop_stack(t);
        Ok((src, t, min))
    }

    /// `DEC$(n,format)` (ROM `F8EA`): `n` formatted like `PRINT USING` with a single
    /// numeric field (*Improper argument* for anything else).
    ///
    /// Quirk: the function dispatcher has already taken the `(` (`D08F`) for every
    /// function, including this group that read their own arguments and `)`, but `DEC$`
    /// checks for a `(` again (`F8EA`).  So BASIC 1.0 wants an unbalanced
    /// `DEC$((n,"##.#")`: two opening brackets and one closing.  `DEC$(n,"##.#")` and
    /// the balanced `DEC$((n,"##.#"))` are both *Syntax error*s.
    pub fn fn_dec(&mut self) -> Res {
        self.expect(b'(')?;
        self.eval()?;
        self.expect(b',')?;
        let t = self.push_acc()?;
        let (len, fmt) = self.eval_string()?;
        self.expect(b')')?;
        let text = self.ptr;
        let src = self.pop_stack(t);
        self.set_acc_from(src, t);                        // FF4B
        if len == 0 { return Err(Exit::Error(5)); }
        let Some((rest, flags, field)) = self.parse_number_field(fmt, len)? else { return Err(Exit::Error(5)) };   // F3BA
        if rest != 0 { return Err(Exit::Error(5)); }
        let digits = self.format_acc(flags, field)?;      // EE9F
        self.cstring_result(digits)?;
        self.ptr = text;
        Ok(())
    }

    /// `STR$(n)` (ROM `F91E`).
    pub fn fn_str(&mut self) -> Res {
        let text = self.ptr;
        let digits = self.format_acc_print(text)?;        // EE9D
        self.cstring_result(digits)?;
        self.ptr = text;
        Ok(())
    }

    /// ROM `F922`: the zero-terminated text at `p` copied into a new temporary string.
    fn cstring_result(&mut self, p: u16) -> Res {
        let mut len: u16 = 0;
        while self.rd(p.wrapping_add(len)) != 0 { len += 1; }
        let dst = self.alloc_temp_string(len as u8)?;
        if len as u8 != 0 { self.ldir(p, dst, len); }
        self.push_temp_string()
    }

    // ===============================================================================
    // LEFT$, RIGHT$, MID$   (F93C .. FA09)
    // ===============================================================================

    /// `LEFT$(s,n)` (ROM `F93C`).
    pub fn fn_left(&mut self) -> Res {
        let (desc, n) = self.string_and_byte()?;          // F9E9
        self.expect(b')')?;
        self.substring(desc, n, 0)                        // F971
    }

    /// `RIGHT$(s,n)` (ROM `F943`).
    pub fn fn_right(&mut self) -> Res {
        let (desc, n) = self.string_and_byte()?;
        let start = self.rd(desc).wrapping_sub(n);
        self.expect(b')')?;
        self.substring(desc, n, start)
    }

    /// `MID$(s,start[,n])` as a function (ROM `F94B`, through the operand table).
    pub fn fn_mid(&mut self) -> Res {
        self.expect(b'(')?;
        let (desc, start) = self.string_and_byte()?;
        if start == 0 { return Err(Exit::Error(5)); }
        let start = start - 1;
        let n = self.optional_length()?;                  // F9FB
        let len = self.rd(desc);
        let count = match len.checked_sub(start) {
            None => 0,
            Some(left) => if left < n { left } else { n },
        };
        self.expect(b')')?;
        self.substring(desc, count, start)
    }

    /// ROM `F971`: `count` characters from offset `start` of the string `desc` (all of it
    /// if `count` exceeds its length).
    pub fn substring(&mut self, desc: u16, count: u8, mut start: u8) -> Res {
        let len = self.rd(desc);
        let n = if len >= count { count } else { start = 0; len };
        let dst = self.alloc_temp_string(n)?;
        let (_, src) = self.release_string(desc);
        if n != 0 { self.ldir(src.wrapping_add(start as u16), dst, n as u16); }
        self.push_temp_string()
    }

    /// `MID$(var$,start[,n]) = s` (ROM `F993`): overwrite part of a string variable in
    /// place (its text is first copied into the string space if it is a literal in the
    /// program).  The length never changes.
    pub fn stmt_mid(&mut self) -> Res {
        self.expect(b'(')?;
        let (var, _) = self.find_variable()?;             // D686
        self.require_string()?;
        if self.rd(var) != 0 {
            let addr = self.rd16(var + 1);
            self.copy_string_in(var + 2, addr)?;          // FB21 / FB2E
        }
        self.expect(b',')?;
        let start = self.eval_nonzero_byte()?;
        let mut n = self.optional_length()?;
        self.expect(b')')?;
        self.expect(0xEF)?;
        let (rlen, raddr) = self.eval_string()?;
        if n == 0 { return Ok(()); }
        let len = self.rd(var);
        let Some(left) = len.checked_sub(start) else { return Err(Exit::Error(5)) };
        let left = left.wrapping_add(1);
        if left < n { n = left; }
        let dst = self.rd16(var + 1).wrapping_add(start as u16 - 1);
        let count = if n >= rlen { rlen } else { n };
        if count != 0 { self.ldir(raddr, dst, count as u16); }
        Ok(())
    }

    /// ROM `F9E9`: `string,byte`: `(the string's descriptor, the byte)`.
    fn string_and_byte(&mut self) -> Res<(u16, u8)> {
        self.eval_string_keep()?;                         // CEA5
        self.expect(b',')?;
        let desc = self.rd16(ACC);
        let n = self.eval_byte()?;
        Ok((desc, n))
    }

    /// ROM `F9FB`: `[,n]` before the `)`; 255 if omitted.
    fn optional_length(&mut self) -> Res<u8> {
        if self.cur() == b')' { return Ok(0xFF); }
        self.expect(b',')?;
        self.eval_byte()
    }

    // ===============================================================================
    // LEN, ASC, CHR$, INKEY$, STRING$, SPACE$, VAL   (FA0A .. FA9F)
    // ===============================================================================

    /// `LEN(s)` (ROM `FA0A`).
    pub fn fn_len(&mut self) -> Res {
        let (len, _) = self.pop_string()?;
        self.set_acc_int(len as u16);
        Ok(())
    }

    /// `ASC(s)` (ROM `FA10`): *Improper argument* for an empty string.
    pub fn fn_asc(&mut self) -> Res {
        let c = self.first_char()?;
        self.set_acc_int(c as u16);
        Ok(())
    }

    /// `CHR$(n)` (ROM `FA16`).
    pub fn fn_chr(&mut self) -> Res {
        let c = self.acc_byte()?;                         // FA92
        self.one_char_string(c)
    }

    /// ROM `FA19`.
    fn one_char_string(&mut self, c: u8) -> Res {
        let dst = self.alloc_temp_string(1)?;
        self.wr(dst, c);
        self.push_temp_string()
    }

    /// `INKEY$` (ROM `FA24`): the next key typed, or the empty string.
    pub fn fn_inkey_string(&mut self) -> Res {
        match self.read_key() {                           // C439
            Some(c) => self.one_char_string(c),
            None => {
                self.wr(TEMP_DESCRIPTOR, 0);
                self.push_temp_string()
            }
        }
    }

    /// `STRING$(n,char)` (ROM `FA36`): `char` a code or a string (its first character).
    pub fn fn_string(&mut self) -> Res {
        let n = self.eval_byte()?;
        self.expect(b',')?;
        self.eval()?;
        self.expect(b')')?;
        let c = if self.acc_is_string() { self.first_char()? } else { self.acc_byte()? };
        self.repeated(n, c)
    }

    /// `SPACE$(n)` (ROM `FA57`).
    pub fn fn_space(&mut self) -> Res {
        let n = self.acc_byte()?;
        self.repeated(n, b' ')
    }

    /// ROM `FA5E`.
    fn repeated(&mut self, n: u8, c: u8) -> Res {
        let dst = self.alloc_temp_string(n)?;
        self.fill(dst, n as u16, c);
        self.push_temp_string()
    }

    /// ROM `FA70`: the first character of the string in the accumulator (*Improper
    /// argument* if empty).
    fn first_char(&mut self) -> Res<u8> {
        let (len, addr) = self.pop_string()?;
        if len == 0 { return Err(Exit::Error(5)); }
        Ok(self.rd(addr))
    }

    /// `VAL(s)` (ROM `FA77`): the number at the start of the string (0 if none); *Type
    /// mismatch* for a malformed number.  The byte after the string is temporarily
    /// replaced by a zero terminator.
    pub fn fn_val(&mut self) -> Res {
        let (len, addr) = self.pop_string()?;
        if len == 0 {
            self.set_acc_int(0);
            return Ok(());
        }
        let end = addr.wrapping_add(len as u16);
        let saved = self.rd(end);
        self.wr(end, 0);
        let r = self.parse_signed_number(addr)?;          // ECA3
        self.wr(end, saved);
        if r.is_some() { Ok(()) } else { Err(Exit::Error(0x0D)) }
    }

    /// ROM `FA92`: the accumulator as a byte 0..255 (*Improper argument* otherwise).
    fn acc_byte(&mut self) -> Res<u8> {
        let v = self.acc_to_int()?;
        if v >> 8 != 0 { return Err(Exit::Error(5)); }
        Ok(v as u8)
    }

    // ===============================================================================
    // INSTR   (FAA1 .. FB1A)
    // ===============================================================================

    /// `INSTR([start,]s,pattern)` (ROM `FAA1`): the position of `pattern` in `s` from
    /// `start` (1..255), 0 if not found.  An empty pattern is found at `start` (if that
    /// is within `s`).
    pub fn fn_instr(&mut self) -> Res {
        self.eval()?;
        let mut start = 1;
        if !self.acc_is_string() {
            start = self.acc_byte()?;
            if start == 0 { return Err(Exit::Error(5)); }
            self.expect(b',')?;
            self.eval_string_keep()?;
        }
        self.expect(b',')?;
        let s = self.rd16(ACC);
        let (plen, paddr) = self.eval_string()?;
        self.expect(b')')?;
        let text = self.ptr;
        let r = self.search(s, start, plen, paddr);       // FAD4
        self.set_acc_int(r as u16);
        self.ptr = text;
        Ok(())
    }

    /// ROM `FAD4`.
    fn search(&mut self, s: u16, start: u8, plen: u8, paddr: u16) -> u8 {
        let (slen, saddr) = self.release_string(s);
        if slen < start { return 0; }
        let mut at = saddr.wrapping_add(start as u16 - 1);
        let mut left = slen - (start - 1);
        if plen == 0 { return start; }
        loop {
            // FAF5: try a match at `at`.
            let (mut d, mut h, mut c, mut b) = (at, paddr, plen, left);
            loop {
                if self.rd(d) != self.rd(h) { break; }
                h = h.wrapping_add(1);
                c -= 1;
                if c == 0 { return slen - left + 1; }
                d = d.wrapping_add(1);
                b -= 1;
                if b == 0 { return 0; }
            }
            at = at.wrapping_add(1);
            left -= 1;
            if left == 0 { return 0; }
        }
    }

    // ===============================================================================
    // Making strings safe to keep   (FB1B .. FB58)
    // ===============================================================================

    /// ROM `FB2E`: string visitor: a string outside the string space (in the program, the
    /// input buffer ...) is copied into it, and its descriptor (`de` - 2) updated.  Used
    /// by `CHAIN` (the program is about to be replaced) and `MID$ =`.
    pub fn copy_string_in(&mut self, de: u16, addr: u16) -> Res {
        let inside = addr > self.rd16(STRINGS_BOTTOM) && addr <= self.rd16(STRINGS_TOP);
        if inside { return Ok(()); }
        let desc = de.wrapping_sub(2);
        self.copy_to_temp(desc)?;                         // FB8F
        self.ldir(TEMP_DESCRIPTOR, desc, 3);              // FBA6
        Ok(())
    }

    /// ROM `FB1B`: copy every string variable's text that is outside the string space into
    /// it.
    pub fn copy_all_strings_in(&mut self) -> Res { self.for_each_string(StringVisitor::CopyIn) }

    /// ROM `FB49`: after `FN` returned a string: unless it is one of the temporaries, copy
    /// it into a new temporary.
    pub fn keep_string_result(&mut self) -> Res {
        let desc = self.rd16(ACC);
        if desc < TEMP_DESCRIPTOR { return Ok(()); }
        self.copy_to_temp(desc)?;
        self.push_temp_string()
    }

    /// ROM `FB59`: the string in the accumulator is about to be stored in a variable:
    /// returns the descriptor to store.  Literals in the program are kept where they are,
    /// as is a fresh result that is the only temporary; anything else (another variable's
    /// string, the input buffer ...) is copied into the string space first.
    pub fn string_for_assignment(&mut self) -> Res<u16> {
        let desc = self.rd16(ACC);
        let len = self.rd(desc);
        if len != 0 {
            let addr = self.rd16(desc + 1);
            let keep = addr > self.rd16(PROGRAM_BASE)
                && addr <= self.rd16(STRINGS_TOP)
                && (addr <= self.rd16(PROGRAM_END) || desc == DESCRIPTORS);
            if !keep {
                self.release_descriptor(desc);            // FBFF
                self.copy_to_temp(desc)?;                 // FB8F
                return Ok(TEMP_DESCRIPTOR);
            }
        }
        self.release_descriptor(desc);
        Ok(desc)
    }

    /// ROM `FB8F`: copy the text of the descriptor at `desc` into a new string; its
    /// descriptor is left in [`TEMP_DESCRIPTOR`].
    fn copy_to_temp(&mut self, desc: u16) -> Res {
        let len = self.rd(desc);
        let dst = self.alloc_temp_string(len)?;           // FC19
        let src = self.rd16(desc + 1);
        if len != 0 { self.ldir(src, dst, len as u16); }
        Ok(())
    }

    // ===============================================================================
    // The temporaries   (FBB3 .. FC2C)
    // ===============================================================================

    /// ROM `FBB3`: no temporaries.
    pub fn reset_descriptor_stack(&mut self) { self.wr16(DESCRIPTOR_SP, DESCRIPTORS); }

    /// ROM `FBBA`: push [`TEMP_DESCRIPTOR`] as a new temporary and make it the
    /// accumulator's (string) value.
    pub fn push_temp_string(&mut self) -> Res {
        self.wr(ACC_TYPE, T_STR);
        let sp = self.rd16(DESCRIPTOR_SP);
        self.wr16(ACC, sp);
        if sp == TEMP_DESCRIPTOR { return Err(Exit::Error(0x10)); }
        self.ldir(TEMP_DESCRIPTOR, sp, 3);
        self.wr16(DESCRIPTOR_SP, sp + 3);
        Ok(())
    }

    /// ROM `FBDA`: the string in the accumulator (*Type mismatch* if it is a number),
    /// released: `(length, address)`.
    pub fn pop_string(&mut self) -> Res<(u8, u16)> {
        self.require_string()?;                           // FF3C
        let desc = self.rd16(ACC);
        Ok(self.release_string(desc))
    }

    /// ROM `FBE8`: release the string `desc`: if it is the newest temporary it is popped,
    /// and its text freed if it is the lowest string in the string space.  Returns
    /// `(length, address)`.
    pub fn release_string(&mut self, desc: u16) -> (u8, u16) {
        let (len, addr, was_top) = self.release_descriptor(desc);
        if was_top && addr.wrapping_sub(1) == self.rd16(STRINGS_BOTTOM) {
            let b = self.rd16(STRINGS_BOTTOM).wrapping_add(len as u16);
            self.wr16(STRINGS_BOTTOM, b);
        }
        (len, addr)
    }

    /// ROM `FBFF`: pop `desc` if it is the newest temporary: `(length, address, popped)`.
    fn release_descriptor(&mut self, desc: u16) -> (u8, u16, bool) {
        let len = self.rd(desc);
        let addr = self.rd16(desc + 1);
        let top = self.rd16(DESCRIPTOR_SP).wrapping_sub(3);
        let popped = top == desc;
        if popped { self.wr16(DESCRIPTOR_SP, top); }
        (len, addr, popped)
    }

    /// ROM `FC19`: allocate `len` bytes of string space for a new string; its descriptor
    /// is set up in [`TEMP_DESCRIPTOR`].  Returns the address.
    pub fn alloc_temp_string(&mut self, len: u8) -> Res<u16> {
        let a = self.alloc_string_space(len)?;            // F5D1
        self.set_temp_descriptor(len, a);
        Ok(a)
    }

    /// Set [`TEMP_DESCRIPTOR`] to `(len, addr)`.
    fn set_temp_descriptor(&mut self, len: u8, addr: u16) {
        self.wr(TEMP_DESCRIPTOR, len);
        self.wr16(TEMP_DESCRIPTOR + 1, addr);
    }

    // ===============================================================================
    // FRE and the garbage collector   (FC2D .. FCB2)
    // ===============================================================================

    /// `FRE(x)` (ROM `FC2D`): the free memory; `FRE("")` collects garbage first.
    pub fn fn_fre(&mut self) -> Res {
        if self.acc_is_string() {
            self.pop_string()?;
            self.collect_garbage();
        }
        let free = self.free_memory();                    // F628
        self.set_acc_unsigned_real(free);
        Ok(())
    }

    /// ROM `FC3E`: compact the string space (see the module documentation).
    pub fn collect_garbage(&mut self) {
        let mut point = self.rd16(STRINGS_TOP);
        loop {
            self.wr16(STRINGS_BOTTOM, point);
            self.wr16(GC_DESCRIPTOR, 0);
            let limit = self.rd16(ARRAYS_END);
            self.wr16(GC_LIMIT, limit);
            // FC7B: the temporaries, then every variable.
            let mut d = DESCRIPTORS;
            while d != self.rd16(DESCRIPTOR_SP) {
                if self.rd(d) != 0 {
                    let addr = self.rd16(d + 1);
                    self.gc_find_highest(d + 2, addr);
                }
                d += 3;
            }
            // (this visitor cannot fail)
            let _ = self.for_each_string(StringVisitor::FindHighest);
            let best = self.rd16(GC_DESCRIPTOR);
            if best == 0 { return; }
            let addr = self.rd16(best - 1);
            let len = self.rd(best - 2) as u16;
            let bottom = self.rd16(STRINGS_BOTTOM);
            if len != 0 { self.lddr(addr.wrapping_add(len).wrapping_sub(1), bottom, len); }
            let new = bottom.wrapping_sub(len).wrapping_add(1);
            self.wr16(best - 1, new);
            point = new.wrapping_sub(1);
        }
    }

    /// ROM `FC9C`: string visitor of the collector: remember the highest string still
    /// below the compacted part.
    pub fn gc_find_highest(&mut self, de: u16, addr: u16) {
        if self.rd16(STRINGS_BOTTOM) < addr { return; }
        if self.rd16(GC_LIMIT) >= addr { return; }
        self.wr16(GC_DESCRIPTOR, de);
        self.wr16(GC_LIMIT, addr);
    }
}
