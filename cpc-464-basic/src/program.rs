//! # The program: lines, line references, `LIST`, `DELETE`, `RENUM`, `RUN`, files  (ROM `E0F7 .. E162`, `E676 .. EC9E`)
//!
//! ## Layout
//!
//! The program starts with a zero byte at [`PROGRAM_BASE`], then the lines in order:
//!
//! ```text
//!   length(2)  number(2)  tokens...  00          (length counts all of it)
//! ```
//!
//! and a zero length ends it ([`PROGRAM_END`] is just after that).
//!
//! ## Line references
//!
//! `GOTO 100` is tokenised as `1E` + the line number.  The first time it is executed it
//! is rewritten as `1D` + the address of the byte before line 100 (the previous line's
//! end marker), so later executions jump directly; [`LINE_REFS_ARE_ADDRESSES`] notes that
//! the program contains such references.  Before anything changes the program (or
//! `LIST`s it) they are turned back into numbers ([`Basic::line_refs_to_numbers`]).
//! `RENUM` uses the same mechanism: refs become addresses, the lines are renumbered, and
//! the addresses now stand for the new numbers.
//!
//! ## Walking the text
//!
//! [`Basic::scan_item`] (`E943`) steps over one *item* of tokenised text (a keyword, a
//! number constant with its bytes, a variable reference with its name, a string ...);
//! everything that searches the program for statements is built on it.  A statement ends
//! at `00`/`01`, and for these searches also at `THEN` and `ELSE`.

use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// What [`Basic::store_numbered_line`] did with a typed line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stored {
    /// It started with a line number: the line was stored (or deleted).
    Line,
    /// It was empty.
    Nothing,
    /// No line number: a direct command, starting at this address.
    Direct(u16),
}

/// The per-statement routines [`Basic::walk_program`] calls (`BC` in the ROM).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Walk {
    /// `E69D`: line addresses back to line numbers.
    RefsToNumbers,
    /// `E996`: forget the cached variable offsets.
    ForgetOffsets,
    /// `E864`: line numbers to addresses (`RENUM`).
    NumbersToRefs,
    /// `E888`: report the references to lines that do not exist (`RENUM`).
    ReportUndefined,
}

const TOK_ELSE: u8 = 0x97;
const TOK_THEN: u8 = 0xEB;
const TOK_IF: u8 = 0xA1;
const TOK_DELETE: u8 = 0x92;
const TOK_MERGE: u8 = 0xAB;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // LIST   (E0F7 .. E162)
    // ===============================================================================

    /// `LIST [range][,#stream]` (ROM `E0F7`).  Ends in direct mode (`Ready`).
    pub fn stmt_list(&mut self) -> Res {
        let (from, to) = self.line_range()?;              // CEB0
        self.output_stream_prefix()?;                     // C1C6 (not restored: Ready does)
        self.check_end()?;
        self.set_direct_mode();
        self.list_lines(from, to)?;                       // E10D
        Err(Exit::Ready)
    }

    /// ROM `E10D`: list lines `from..=to` on the output stream.  ESC pauses / breaks.
    pub fn list_lines(&mut self, from: u16, to: u16) -> Res {
        let (mut line, _) = self.find_line(from);         // E7A3
        loop {
            let length = self.rd16(line);
            if length == 0 { return Ok(()); }
            self.poll_escape()?;                          // C43C
            let next = line.wrapping_add(length);
            if to < self.rd16(line.wrapping_add(2)) { return Ok(()); }
            let mut p = self.detokenise_line(line)?;      // E163
            loop {
                self.list_char(self.rd(p))?;              // E145
                p = p.wrapping_add(1);
                if self.rd(p) == 0 { break; }
            }
            self.new_line()?;
            line = next;
        }
    }

    /// ROM `E145`: one character of a listing: on the screen a control character is
    /// shown (prefixed with `01`); elsewhere `LF` becomes `LF CR`.
    fn list_char(&mut self, c: u8) -> Res {
        if self.output_is_screen() {
            if c < 0x20 { self.output_raw(0x01)?; }
            return self.output_raw(c);
        }
        self.output_raw(c)?;
        if c == 0x0A { self.output_raw(0x0D)?; }
        Ok(())
    }

    // ===============================================================================
    // Storing lines   (E676 .. E766)
    // ===============================================================================

    /// ROM `E676`: no program.
    pub fn empty_program(&mut self) {
        self.wr(LINE_REFS_ARE_ADDRESSES, 0);
        let base = self.rd16(PROGRAM_BASE);
        self.fill(base, 3, 0);
        self.wr16(PROGRAM_END, base.wrapping_add(3));
    }

    /// ROM `E687`: turn all line addresses in the program back into line numbers.
    pub fn line_refs_to_numbers(&mut self) -> Res {
        if self.rd(LINE_REFS_ARE_ADDRESSES) == 0 { return Ok(()); }
        self.walk_program(Walk::RefsToNumbers)?;          // E8FF with E69D
        self.wr(LINE_REFS_ARE_ADDRESSES, 0);
        Ok(())
    }

    /// ROM `E6BC`: a typed line at `text`: store it if it starts with a line number.
    ///
    /// A number that is not a valid line number (0, or too big) counts as "no number";
    /// the caller sees it as a direct command, except that a file being loaded as ASCII
    /// reports it differently (`load_ascii`, `EBEF`).
    pub fn store_numbered_line(&mut self, text: u16) -> Res<Stored> {
        self.ptr = text;
        if self.skip_white() == 0 { return Ok(Stored::Nothing); }
        let start = self.ptr;
        let Ok(after) = self.parse_line_number(start) else { return Ok(Stored::Direct(start)) };   // EE04
        let n = self.rd16(ACC);
        let mut p = after;
        if self.rd(p) == b' ' { p = p.wrapping_add(1); }
        self.store_line(p, n)?;                           // E6D2
        Ok(Stored::Line)
    }

    /// ROM `E6D2`: store the text at `text` as line `n` (replacing it); an empty text
    /// deletes the line (*Line does not exist* if there is none).
    pub fn store_line(&mut self, text: u16, n: u16) -> Res {
        self.line_refs_to_numbers()?;                     // E687
        let (tokens, len) = self.tokenise_counted(text)?; // DEBB
        self.ptr = tokens;
        if self.skip_white() == 0 {
            let at = self.find_line_or_error(n)?;         // E79A
            let old = self.rd16(at);
            self.delete_line(at, old);                    // E70B
            return Ok(());
        }
        let length = len.wrapping_add(4);
        let (at, found) = self.find_line(n);              // E7A3
        if found {
            let old = self.rd16(at);
            self.delete_line(at, old);
        }
        self.open_gap_in_vars(length, at)?;               // F5F8
        self.shift_after_program(length);                 // F52C
        self.wr16(at, length);
        self.wr16(at.wrapping_add(2), n);
        self.ldir(tokens, at.wrapping_add(4), len);
        Ok(())
    }

    /// ROM `E70B`: remove `length` bytes at `at` (a line, or a range of lines).
    fn delete_line(&mut self, at: u16, length: u16) {
        let next = at.wrapping_add(length);
        let count = self.rd16(ARRAYS_END).wrapping_sub(next);
        if count != 0 { self.ldir(next, at, count); }
        self.shift_after_program(0u16.wrapping_sub(length));
    }

    /// `DELETE range` (ROM `E728`).  Ends in `Ready`.
    pub fn stmt_delete(&mut self) -> Res {
        self.lines_in_range()?;                           // E737
        self.check_end()?;
        self.delete_range()?;                             // E75A
        self.after_program_change();
        Err(Exit::Ready)
    }

    /// ROM `E737`: a line range; its first line and byte length go to [`RANGE_START`] /
    /// [`RANGE_LENGTH`].  *Improper argument* if it contains no lines.
    fn lines_in_range(&mut self) -> Res {
        let (from, to) = self.line_range()?;
        let end = self.line_after(to);                    // E7C1
        let (start, _) = self.find_line(from);
        self.wr16(RANGE_START, start);
        let (len, borrow) = end.overflowing_sub(start);
        self.wr16(RANGE_LENGTH, len);
        if borrow || len == 0 { return Err(Exit::Error(5)); }
        Ok(())
    }

    /// ROM `E75A`: delete the lines found by [`Self::lines_in_range`].
    fn delete_range(&mut self) -> Res {
        self.line_refs_to_numbers()?;
        let len = self.rd16(RANGE_LENGTH);
        let start = self.rd16(RANGE_START);
        self.delete_line(start, len);
        Ok(())
    }

    // ===============================================================================
    // Finding lines   (E767 .. E7DE)
    // ===============================================================================

    /// ROM `E767`: the line reference at the text pointer (`1E` number or `1D` address):
    /// the address of the byte before the line.  A number is looked up (forwards from the
    /// current line if it is further on) and the reference rewritten as an address.
    pub fn line_operand(&mut self) -> Res<u16> {
        let t = self.cur();
        let v = self.rd16(self.ptr.wrapping_add(1));
        let after = self.ptr.wrapping_add(3);
        self.ptr = after;
        if t == 0x1D { return Ok(v); }
        if t != 0x1E { return Err(Exit::Error(2)); }      // E8EA
        let mut found = None;
        if let Some(cur) = self.current_line_number() {
            if cur < v {
                self.skip_to_end_of_line();               // E8F3
                let from = self.ptr.wrapping_add(1);
                let (at, ok) = self.find_line_from(from, v);   // E7A7
                if ok { found = Some(at); }
                self.ptr = after;
            }
        }
        let line = match found { Some(a) => a, None => self.find_line_or_error(v)? };
        let addr = line.wrapping_sub(1);
        self.wr(LINE_REFS_ARE_ADDRESSES, 0x1D);
        self.wr(after.wrapping_sub(3), 0x1D);
        self.wr16(after.wrapping_sub(2), addr);
        Ok(addr)
    }

    /// ROM `E79A`: the address of line `n`; *Line does not exist* if there is none.
    pub fn find_line_or_error(&mut self, n: u16) -> Res<u16> {
        let (at, found) = self.find_line(n);
        if found { Ok(at) } else { Err(Exit::Error(8)) }
    }

    /// ROM `E7A3`: the first line numbered `n` or more: `(address, it is line n)`.  (The
    /// end of the program if there is none.)
    pub fn find_line(&self, n: u16) -> (u16, bool) {
        self.find_line_from(self.rd16(PROGRAM_BASE).wrapping_add(1), n)
    }

    /// ROM `E7A7`: as [`Self::find_line`], searching from the line at `p`.
    fn find_line_from(&self, mut p: u16, n: u16) -> (u16, bool) {
        loop {
            let length = self.rd16(p);
            if length == 0 { return (p, false); }
            let number = self.rd16(p.wrapping_add(2));
            if n < number { return (p, false); }
            if n == number { return (p, true); }
            p = p.wrapping_add(length);
        }
    }

    /// ROM `E7C1`: the first line numbered above `n` (or the end of the program).
    fn line_after(&self, n: u16) -> u16 {
        let mut p = self.rd16(PROGRAM_BASE).wrapping_add(1);
        loop {
            let length = self.rd16(p);
            if length == 0 { return p; }
            if n < self.rd16(p.wrapping_add(2)) { return p; }
            p = p.wrapping_add(length);
        }
    }

    // ===============================================================================
    // RENUM   (E7DF .. E89E)
    // ===============================================================================

    /// `RENUM [new][,[old][,step]]` (ROM `E7DF`): renumber from line `old` (default the
    /// first) as `new, new+step...` (defaults 10, 10).  *Improper argument* if that would
    /// put lines out of order or past 65535.  References to lines that do not exist are
    /// reported (`Undefined line`) and left alone.  Ends in `Ready`.
    pub fn stmt_renum(&mut self) -> Res {
        let c = self.cur();
        let mut new = 10;
        if !crate::is_end(c) && c != b',' { new = self.read_line_number_constant()?; }
        let mut old = 0;
        if self.comma() && self.cur() != b',' { old = self.read_line_number_constant()?; }
        let mut step = 10;
        if self.comma() { step = self.read_line_number_constant()?; }
        self.check_end()?;
        let (at_new, _) = self.find_line(new);
        let (at_old, _) = self.find_line(old);
        if at_new < at_old { return Err(Exit::Error(5)); }
        // E81F: will the numbers fit?
        let mut p = at_old;
        let mut number = new;
        loop {
            let length = self.rd16(p);
            if length == 0 { break; }
            p = p.wrapping_add(length);
            if self.rd16(p) == 0 { break; }
            let (n, carry) = number.overflowing_add(step);
            if carry { return Err(Exit::Error(5)); }
            number = n;
        }
        self.walk_program(Walk::NumbersToRefs)?;          // E864
        let mut p = at_old;
        let mut number = new;
        loop {
            let length = self.rd16(p);
            if length == 0 { break; }
            self.wr16(p.wrapping_add(2), number);
            p = p.wrapping_add(length);
            number = number.wrapping_add(step);
        }
        self.walk_program(Walk::ReportUndefined)?;        // E888
        Err(Exit::Ready)
    }

    // ===============================================================================
    // Skipping text   (E89F .. E988)
    // ===============================================================================

    /// ROM `E89F`: after a false `IF`: find its `ELSE` (nested `IF`s need their own).
    /// `true` with the text pointer after the `ELSE`; `false` at the end of the line.
    pub fn skip_to_else(&mut self) -> bool {
        let mut depth: u8 = 1;
        self.ptr = self.ptr.wrapping_sub(1);
        let mut c = self.scan_item();
        loop {
            if c == 0 { return false; }
            if c == 0x01 {
                c = self.scan_item();
                if c != TOK_ELSE { continue; }
                depth -= 1;
                if depth == 0 {
                    self.next();
                    return true;
                }
            } else if c == TOK_IF {
                depth = depth.wrapping_add(1);
            }
            c = self.scan_item();
        }
    }

    /// ROM `E8C1`: step over `(...)` / `[...]` after a variable name (nested brackets
    /// counted); *Syntax error* if the statement ends first.
    pub fn skip_subscripts(&mut self) -> Res {
        let c = self.cur();
        if c != b'[' && c != b'(' { return Ok(()); }
        let mut depth: u8 = 1;
        loop {
            match self.scan_item() {
                b'[' | b'(' => depth = depth.wrapping_add(1),
                b']' | b')' => {
                    depth -= 1;
                    if depth == 0 {
                        self.ptr = self.ptr.wrapping_add(1);
                        return Ok(());
                    }
                }
                c if c < 2 => return Err(Exit::Error(2)),
                _ => {}
            }
        }
    }

    /// ROM `E8EF`: move to the end of the statement (`00` or `01`, returned).  `DATA`,
    /// `DEF`, `GOSUB` ... use it.
    pub fn skip_to_end_of_statement(&mut self) -> u8 { self.skip_until(1) }

    /// ROM `E8F3`: move to the end of the line (`REM`, `'`, `ELSE`).
    pub fn skip_to_end_of_line(&mut self) -> u8 { self.skip_until(0) }

    /// ROM `E8F5`.
    fn skip_until(&mut self, stop: u8) -> u8 {
        self.ptr = self.ptr.wrapping_sub(1);
        loop {
            let c = self.scan_item();
            if c == 0 || c == stop { return c; }
        }
    }

    /// ROM `E8FF`: call `walk` for every statement of the program (with the text pointer
    /// on the byte before it), each line becoming the current line meanwhile.
    fn walk_program(&mut self, walk: Walk) -> Res {
        let saved_line = self.current_line();
        let saved_ptr = self.ptr;
        let mut p = self.rd16(PROGRAM_BASE);
        while self.rd16(p.wrapping_add(1)) != 0 {
            self.set_current_line(p.wrapping_add(3));
            self.ptr = p.wrapping_add(4);
            loop {
                self.walk_statement(walk)?;
                self.ptr = self.ptr.wrapping_sub(1);
                if self.skip_statement() == 0 { break; }  // E935
            }
            p = self.ptr;
        }
        self.set_current_line(saved_line);
        self.ptr = saved_ptr;
        Ok(())
    }

    /// The four statement routines of [`Self::walk_program`]: each scans the items of
    /// one statement (to its `00`/`01`).
    fn walk_statement(&mut self, walk: Walk) -> Res {
        loop {
            let before = self.ptr;
            let c = self.scan_item();
            if c < 2 { return Ok(()); }
            let end = self.ptr;
            match walk {
                Walk::RefsToNumbers if c == 0x1D => {
                    // E69D: the address of the byte before the line -> its number.
                    let addr = self.rd16(end.wrapping_sub(1));
                    let n = self.rd16(addr.wrapping_add(3));
                    self.wr(end.wrapping_sub(2), 0x1E);
                    self.wr16(end.wrapping_sub(1), n);
                }
                Walk::NumbersToRefs if c == 0x1E => {
                    // E864
                    let n = self.rd16(end.wrapping_sub(1));
                    let (at, found) = self.find_line(n);
                    if found {
                        self.wr16(end.wrapping_sub(1), at.wrapping_sub(1));
                        self.wr(end.wrapping_sub(2), 0x1D);
                        self.wr(LINE_REFS_ARE_ADDRESSES, 0x1D);
                    }
                }
                Walk::ReportUndefined if c == 0x1E => {
                    // E888
                    let n = self.rd16(end.wrapping_sub(1));
                    self.undefined_line_message(n)?;      // CB18
                }
                Walk::ForgetOffsets if c < 0x0E && c != 0x07 && c != 0x08 => {
                    // E996: a variable reference: offset 0 (and token 0D stays 0D).
                    self.ptr = before;
                    let t = self.next();
                    let at = self.ptr;
                    if t >= 0x0D { self.wr(at, 0x0D); }
                    self.wr16(at.wrapping_add(1), 0);
                    self.ptr = end;
                }
                _ => {}
            }
        }
    }

    /// ROM `E923`: from the text pointer, the start of the next statement (a statement
    /// ends at `00`, `01`, `THEN` or `ELSE`).  `line` (the current line address, `DE`) is
    /// updated when a line is crossed; at the end of the program error `err` is raised.
    /// The text pointer is left on the byte before the next statement.
    pub fn find_next_statement(&mut self, line: u16, err: u8) -> Res<u16> {
        if self.skip_statement() != 0 { return Ok(line); }
        let at = self.ptr;
        if self.rd16(at.wrapping_add(1)) == 0 { return Err(Exit::Error(err)); }
        self.ptr = at.wrapping_add(4);
        Ok(at.wrapping_add(3))
    }

    /// ROM `E935`: skip items to the end of the statement: `00`, `01`, `ELSE` or `THEN`
    /// (returned).
    fn skip_statement(&mut self) -> u8 {
        loop {
            let c = self.scan_item();
            if c < 2 || c == TOK_ELSE || c == TOK_THEN { return c; }
        }
    }

    /// ROM `E943`: step to the next item and over its data; returns its first byte.  The
    /// text pointer is left on the item's last byte.
    pub fn scan_item(&mut self) -> u8 {
        let c = self.next();
        if crate::is_end(c) { return c; }
        match c {
            0x07 | 0x08 => {}
            0x02..=0x0D => {
                // a variable: offset, then the name up to the byte with bit 7
                self.ptr = self.ptr.wrapping_add(2);
                self.skip_name();
            }
            0x0E..=0x17 => {}
            0x18 | 0x19 => self.ptr = self.ptr.wrapping_add(if c == 0x19 { 1 } else { 0 }),
            0x1A..=0x1E => self.ptr = self.ptr.wrapping_add(2),
            0x1F => self.ptr = self.ptr.wrapping_add(5),
            b'"' => {
                loop {
                    self.ptr = self.ptr.wrapping_add(1);
                    let d = self.cur();
                    if d == b'"' { break; }
                    if d == 0 {
                        self.ptr = self.ptr.wrapping_sub(1);
                        break;
                    }
                }
            }
            b'|' => self.skip_name(),
            0xFF => self.ptr = self.ptr.wrapping_add(1),
            _ => {}
        }
        c
    }

    /// ROM `E970`: move to the next byte with bit 7 set.
    fn skip_name(&mut self) {
        loop {
            self.ptr = self.ptr.wrapping_add(1);
            if self.cur() & 0x80 != 0 { return; }
        }
    }

    /// ROM `E989`: zero the cached offsets of every variable reference in the program.
    pub fn forget_variable_offsets(&mut self) {
        // (The walk only fails when one of its routines raises an error; this one
        // cannot.)
        let _ = self.walk_program(Walk::ForgetOffsets);
    }

    // ===============================================================================
    // RUN, LOAD   (E9BD .. EA3B)
    // ===============================================================================

    /// `RUN [line]` / `RUN "file"` (ROM `E9BD`).
    pub fn stmt_run(&mut self) -> Res {
        let c = self.cur();
        let mut start = self.rd16(PROGRAM_BASE);
        if !crate::is_end(c) {
            if c == 0x1E || c == 0x1D {
                start = self.line_operand()?;             // E767
            } else {
                if !self.load_file()? {                   // EA0D
                    // A binary file: the firmware loads it with the routine at EA30
                    // and enters it.
                    return self.mc_boot_program(LOAD_BINARY);
                }
                self.load_program()?;                     // EBA8
                let base = self.rd16(PROGRAM_BASE);
                return Err(Exit::ContinueLine(base.wrapping_add(1)));
            }
        }
        self.abandon_files();                             // D2AD
        self.clear_variables_and_strings();               // C18C
        self.after_program_change();                      // C17A
        self.reset_angle_mode();                          // C15E
        Err(Exit::ContinueLine(start.wrapping_add(1)))
    }

    /// `LOAD "file"[,address]` (ROM `E9F6`): a BASIC program (then `Ready`), or a binary
    /// file (only outside BASIC's memory, see [`Basic::check_load_range`]).
    pub fn stmt_load(&mut self) -> Res {
        if self.load_file()? {
            self.load_program()?;
            return Err(Exit::Ready);
        }
        let start = self.rd16(LOAD_ADDRESS);
        let length = self.rd16(FILE_LENGTH);
        self.check_load_range(start, length)?;            // F501
        self.load_binary()                                // EA30
    }

    /// ROM `EA0D`: open the file named at the text pointer.  A BASIC file (`true`): the
    /// program is erased, ready for loading.  A binary file (`false`): the optional
    /// `,address` replaces the header's load address ([`LOAD_ADDRESS`]).
    fn load_file(&mut self) -> Res<bool> {
        let header = self.open_program_file()?;           // EB8F
        if (header.file_type & 0x0E) ^ 0x02 != 0 {
            self.check_end()?;
            self.clear_variables_and_strings();
            self.reset_program_state();
            return Ok(true);
        }
        let mut addr = header.load_address;
        if self.comma() { addr = self.eval_address()?; }
        self.wr16(LOAD_ADDRESS, addr);
        self.check_end()?;
        Ok(false)
    }

    /// ROM `EA30`: read the whole (binary) file to [`LOAD_ADDRESS`]; ESC is a *Break*.
    fn load_binary(&mut self) -> Res {
        let addr = self.rd16(LOAD_ADDRESS);
        match self.cas_in_direct(addr) {
            Ok(_) => { self.cas_in_close(); Ok(()) }
            Err(e) if e.zero => self.do_break().map(|n| match n {}),
            Err(_) => Ok(()),
        }
    }

    // ===============================================================================
    // CHAIN, MERGE   (EA3C .. EB8E)
    // ===============================================================================

    /// `CHAIN [MERGE] "file"[,line][,DELETE range]` (ROM `EA3C`): load (or merge) a
    /// program keeping the variables, and run it from `line` (default the start).
    pub fn stmt_chain(&mut self) -> Res {
        let merge = self.cur() == TOK_MERGE;
        if merge { self.next(); }
        self.wr(MERGE_FLAG, if merge { 0xFF } else { 0 });
        self.open_program_file()?;                        // EB8F
        let mut line = 0;
        if self.comma() && self.cur() != b',' { line = self.eval_address()?; }
        let mut delete = false;
        if self.comma() {
            self.expect(TOK_DELETE)?;
            self.lines_in_range()?;                       // E737
            delete = true;
        }
        self.check_end()?;
        self.copy_all_strings_in()?;                      // FB1B
        self.collect_garbage();                           // FC3E
        self.forget_variable_offsets();                   // E989
        self.clear_fn_list();                             // D5D2
        let (vars_size, size) = self.park_arrays();       // F549
        if delete { self.delete_range()?; }               // E75A
        if self.rd(MERGE_FLAG) == 0 {
            self.reset_program_state();                   // C16B
            self.load_program()?;                         // EBA8
        } else {
            self.merge_program()?;                        // EB9D
        }
        self.unpark_arrays(vars_size, size);              // F571
        let base = self.rd16(PROGRAM_BASE);
        self.ptr = if line == 0 { base } else { self.find_line_or_error(line)?.wrapping_sub(1) };
        Ok(())
    }

    /// `MERGE "file"` (ROM `EAA6`).
    pub fn stmt_merge(&mut self) -> Res {
        self.open_program_file()?;
        self.check_end()?;
        self.clear_variables_and_strings();
        self.merge_program()?;
        Err(Exit::Ready)
    }

    /// ROM `EAB5`: merge a tokenised BASIC file into the program.  The program is moved
    /// up against the string space; the lines of both are then merged back down in
    /// order, a file line replacing a program line of the same number.  *Memory full*
    /// if they come within 32 bytes of each other.
    fn merge_tokenised(&mut self) -> Res {
        self.after_program_change();
        self.line_refs_to_numbers()?;
        let end = self.rd16(PROGRAM_END);
        let base = self.rd16(PROGRAM_BASE).wrapping_add(1);
        self.wr16(PROGRAM_END, base);
        let size = end.wrapping_sub(base);
        let bottom = self.rd16(STRINGS_BOTTOM);
        if size != 0 { self.lddr(end.wrapping_sub(1), bottom, size); }
        let mut old = bottom.wrapping_sub(size).wrapping_add(1);   // the moved program
        loop {
            // EAD5
            if old < self.rd16(PROGRAM_END).wrapping_add(0x20) { return self.load_failed(7); }
            let length = self.file_word()?;               // EB84
            if length == 0 {
                // EB1A: copy the rest of the old program, then the end marker.
                while self.rd16(old) != 0 { old = self.copy_down_line(old); }
                let e = self.rd16(PROGRAM_END);
                self.wr16(e, 0);
                self.wr16(PROGRAM_END, e.wrapping_add(2));
                self.no_variables();
                return Ok(());
            }
            let number = self.file_word()?;
            // EAEE: old lines numbered below the file's line go first.
            loop {
                if self.rd16(old) == 0 { break; }
                let n = self.rd16(old.wrapping_add(2));
                if n == number {
                    old = old.wrapping_add(self.rd16(old));   // replaced (EB0E)
                    break;
                }
                if n > number { break; }
                old = self.copy_down_line(old);           // EB48
            }
            self.read_file_line_into_program(length, number)?;   // EB5E
        }
    }

    /// ROM `EB3A`: a failure while loading: the program is erased, error `e`.
    fn load_failed(&mut self, e: u8) -> Res {
        self.abandon_files();
        self.clear_variables_and_strings();
        self.reset_program_state();
        Err(Exit::Error(e))
    }

    /// ROM `EB48`: copy the old line at `old` to the end of the new program; returns the
    /// next old line.
    fn copy_down_line(&mut self, old: u16) -> u16 {
        let length = self.rd16(old);
        let end = self.rd16(PROGRAM_END);
        self.ldir(old, end, length);
        self.wr16(PROGRAM_END, end.wrapping_add(length));
        old.wrapping_add(length)
    }

    /// ROM `EB5E`: append the file's line (`length`, `number` already read) to the new
    /// program.
    fn read_file_line_into_program(&mut self, length: u16, number: u16) -> Res {
        let mut p = self.rd16(PROGRAM_END);
        self.wr16(p, length);
        self.wr16(p.wrapping_add(2), number);
        p = p.wrapping_add(4);
        let mut left = length.wrapping_sub(3);
        loop {
            left = left.wrapping_sub(1);
            if left == 0 { break; }
            match self.cas_in_char() {
                Ok(c) => { self.wr(p, c); p = p.wrapping_add(1); }
                Err(_) => return self.load_failed(0x18),
            }
        }
        self.wr16(PROGRAM_END, p);
        Ok(())
    }

    /// ROM `EB84`: a word from the file (*EOF met* if it ends).
    fn file_word(&mut self) -> Res<u16> {
        let lo = match self.cas_in_char() { Ok(c) => c, Err(_) => return self.load_failed(0x18).map(|_| 0) };
        let hi = match self.cas_in_char() { Ok(c) => c, Err(_) => return self.load_failed(0x18).map(|_| 0) };
        Ok(lo as u16 | (hi as u16) << 8)
    }

    // ===============================================================================
    // Program files   (EB8F .. EC06)
    // ===============================================================================

    /// ROM `EB8F`: close any files, open the input file named at the text pointer and
    /// record its type and length ([`FILE_TYPE`], [`FILE_LENGTH`]).
    fn open_program_file(&mut self) -> Res<cpc464_os::cassette::CasOpen> {
        self.abandon_files();
        let h = self.open_input_file()?;                  // D26A
        self.wr(FILE_TYPE, h.file_type);
        self.wr16(FILE_LENGTH, h.logical_length);
        Ok(h)
    }

    /// ROM `EB9D`: merge the open file: a tokenised program (type 0), or an ASCII file;
    /// anything else is a *File type error*.
    fn merge_program(&mut self) -> Res {
        match self.rd(FILE_TYPE) {
            0 => self.merge_tokenised(),
            0x16 => self.load_program(),
            _ => Err(Exit::Error(0x19)),
        }
    }

    /// ROM `EBA8`: load the open file as the program: ASCII (lines typed in one by one),
    /// or tokenised (types 0 and 1 - 1 is protected); else *File type error*.
    fn load_program(&mut self) -> Res {
        let t = self.rd(FILE_TYPE);
        if t == 0x16 { return self.load_ascii(); }
        if t & 0xFE != 0 { return Err(Exit::Error(0x19)); }
        // EBB8
        self.after_program_change();
        let start = self.rd16(PROGRAM_BASE).wrapping_add(1);
        let length = self.rd16(FILE_LENGTH);
        let top = self.rd16(STRINGS_BOTTOM).wrapping_add(0xFF80);
        let (room, borrow) = top.overflowing_sub(start);
        if borrow || room < length { return self.load_failed(7); }
        self.wr16(PROGRAM_END, length.wrapping_add(start));
        self.wr(PROTECTED, if t & 1 != 0 { 0xFF } else { 0 });
        if let Err(e) = self.cas_in_direct(start) {
            if e.zero { return self.load_failed(0x18); }
        }
        self.no_variables();
        self.close_in()
    }

    /// ROM `EBEF`: load an ASCII file: each line is stored as if typed.  A line without
    /// a line number is *Direct command found*; a bad line number (0, too big) gives
    /// *Overflow*... as far as the flags left by the number parser say.
    fn load_ascii(&mut self) -> Res {
        self.after_program_change();
        self.set_direct_mode();
        loop {
            if !self.read_file_line()? { return self.close_in(); }   // CA4C
            match self.store_numbered_line(INPUT_BUFFER)? {           // E6BC
                Stored::Line | Stored::Nothing => {}
                Stored::Direct(p) => {
                    let (_, _, _, any_digit) = self.digits_in_base(p, 10);
                    return Err(Exit::Error(if any_digit { 6 } else { 0x15 }));
                }
            }
        }
    }

    // ===============================================================================
    // SAVE   (EC09 .. EC9E)
    // ===============================================================================

    /// `SAVE "file"[,A|,B,start,length[,entry]|,P]` (ROM `EC09`): the program
    /// tokenised, as ASCII (a `LIST` to the file), as binary memory, or protected.
    pub fn stmt_save(&mut self) -> Res {
        self.abandon_files();
        self.open_output_file()?;                         // D256
        let mut file_type = 0;
        if self.comma() {
            self.expect(0x0D)?;                           // the option is tokenised as a variable
            self.ptr = self.ptr.wrapping_add(2);
            let c = self.cur() & 0xDF;
            self.ptr = self.ptr.wrapping_add(1);
            if c & 0x80 == 0 { return Err(Exit::Error(2)); }   // more than one letter
            match c {
                0xC1 => return self.save_ascii(),         // A: EC87
                0xC2 => return self.save_binary(),        // B: EC5C
                0xD0 => file_type = 1,                    // P: EC3D
                _ => return Err(Exit::Error(2)),
            }
        }
        // EC3F
        self.check_end()?;
        let text = self.ptr;
        self.line_refs_to_numbers()?;
        self.forget_variable_offsets();
        let start = self.rd16(PROGRAM_BASE).wrapping_add(1);
        let length = self.rd16(PROGRAM_END).wrapping_sub(start);
        self.save_direct(start, length, 0, file_type)?;
        self.ptr = text;
        Ok(())
    }

    /// `SAVE ...,B,start,length[,entry]` (ROM `EC5C`).
    fn save_binary(&mut self) -> Res {
        self.expect(b',')?;
        let start = self.eval_address()?;
        self.expect(b',')?;
        let length = self.eval_address()?;
        let entry = if self.comma() { self.eval_address()? } else { 0 };
        self.check_end()?;
        let text = self.ptr;
        self.save_direct(start, length, entry, 2)?;
        self.ptr = text;
        Ok(())
    }

    /// ROM `EC7F`: write the block; then close the file (ESC is a *Break*).
    fn save_direct(&mut self, start: u16, length: u16, entry: u16, file_type: u8) -> Res {
        if self.cas_out_direct(start, length, entry, file_type).is_err() {
            return self.do_break().map(|n| match n {});
        }
        self.close_out()                                  // EC9E / D2A1
    }

    /// `SAVE ...,A` (ROM `EC87`): `LIST` the program to the file.
    fn save_ascii(&mut self) -> Res {
        self.check_end()?;
        let text = self.ptr;
        let old = self.set_output_stream(9);
        self.list_lines(1, 0xFFFF)?;
        self.set_output_stream(old);
        self.close_out()?;
        self.ptr = text;
        Ok(())
    }
}

/// ROM `EA30`, the loader `RUN "binary"` hands to `MC BOOT PROGRAM`.
const LOAD_BINARY: u16 = 0xEA30;
