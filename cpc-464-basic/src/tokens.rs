//! # Tokeniser and de-tokeniser  (ROM `DEBB .. E675`)
//!
//! ## Tokenised text
//!
//! | bytes                     | meaning                                                  |
//! |---------------------------|----------------------------------------------------------|
//! | `00`                      | end of line                                              |
//! | `01`                      | `:` (statement separator)                                |
//! | `02`/`03`/`04` off(2) name| variable with suffix `%` / `$` / `!`                     |
//! | `0B`/`0C`/`0D` off(2) name| variable without suffix (integer / string / real by `DEF`) |
//! | `0E..17`                  | the integers 0..9                                        |
//! | `19` n                    | 8-bit integer                                            |
//! | `1A` nn                   | 16-bit decimal integer                                   |
//! | `1B` nn / `1C` nn         | 16-bit binary (`&X`) / hexadecimal (`&`) integer         |
//! | `1D` addr                 | line reference converted to the line's address           |
//! | `1E` nn                   | line number reference                                    |
//! | `1F` 5 bytes              | real                                                     |
//! | `22` ... `22`             | string literal, as typed                                 |
//! | `7C 00` name              | RSX call `|NAME`                                         |
//! | `80..DC`                  | statement keywords                                       |
//! | `E3..ED`                  | other keywords (`ERL`, `FN`, `SPC`, `STEP`, `SWAP`, `TAB`, `THEN`, `TO`, `USING`) |
//! | `EE..FE`                  | operators (`>`, `=`, `>=`, `<`, `<>`, `<=`, `+ - * / ^ \`, `AND MOD OR XOR NOT`) |
//! | `FF` n                    | function `n`                                             |
//!
//! A name is stored with bit 7 set on its last character; the two "offset" bytes after
//! a variable token cache the variable's position once it has been looked up (they are
//! zero in freshly tokenised text and cleared again by [`Basic::forget_variable_offsets`]).
//!
//! ## Keyword tables
//!
//! `E354` has 26 pointers, one per initial letter.  Each list holds the keywords starting
//! with that letter *without* the letter, the last character with bit 7 set, followed by
//! the token; a zero ends the list.  In a keyword a TAB means "any spaces here" (`GO TO`,
//! `GO SUB`) and a space means "spaces required" (`ON BREAK`).  The lists are stored in
//! memory in the order `Z, Y, ... A`, which the de-tokeniser relies on.  The symbols
//! (`^ \ >= => > = <> <= =< < / : * - + '`) are a separate list at `E64B`.
//!
//! ## Tokeniser state ([`TOKENISER_STATE`])
//!
//! * `0` - normal;
//! * `1` - after a keyword that takes line numbers (`GOTO`, `GOSUB`, `THEN`, `ELSE`,
//!   `RESTORE`, `RUN`, `LIST`, `DELETE`, `EDIT`, `AUTO`, `RENUM`, `RESUME`, `ERL`):
//!   numbers become line-number tokens `1E`;
//! * `FF` - after a variable name.
//!
//! **Quirks (kept):** the state is only reset by a keyword, `:`, `#` or a symbol following
//! a variable, so digits that follow a variable *after a space* (state `FF`) are stored as
//! plain characters, not as a number; and "line number 0" is not a valid line number, so
//! `GOTO 0` stores the digit as text.  `|` names are stored in upper case.

use crate::arith::{is_digit_or_point, is_letter, is_name_char, to_upper};
use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// Keyword list pointers, one per letter `A..Z`.
pub const KEYWORD_INDEX: u16 = 0xE354;
/// The first keyword list in memory (letter `Z`).
pub const KEYWORD_LISTS: u16 = 0xE388;
/// The symbol list.
pub const SYMBOL_LIST: u16 = 0xE64B;
/// Keywords after which the rest of the statement is copied verbatim: `DATA`, `DEFINT`,
/// `DEFSTR`, `DEFREAL`.
const VERBATIM_KEYWORDS: u16 = 0xDF30;
/// Keywords followed by line numbers.
const LINE_NUMBER_KEYWORDS: u16 = 0xDFDC;
/// Maximum length of a tokenised line (bytes).
pub const TOKENISED_MAX: u16 = 300;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Tokeniser   (DEBB .. E0F6)
    // ===============================================================================

    /// ROM `DEBB`: tokenise the zero-terminated text at `text` into the buffer at
    /// [`MEMORY_BOTTOM`].  The result is followed by three zero bytes (end of line, and a
    /// zero "next line length" so that it can be executed directly).  Returns the address
    /// of the tokenised text; *Line too long* past 300 bytes.
    pub fn tokenise(&mut self, text: u16) -> Res<u16> {
        Ok(self.tokenise_counted(text)?.0)
    }

    /// As [`Self::tokenise`], also returning `BC` = bytes used + 1.
    pub fn tokenise_counted(&mut self, text: u16) -> Res<(u16, u16)> {
        let out = self.rd16(MEMORY_BOTTOM);
        let mut t = Tok { src: text, out, left: TOKENISED_MAX };
        self.wr(TOKENISER_STATE, 0);
        loop {
            self.tokenise_item(&mut t)?;                  // DEE1
            if self.rd(t.src) == 0 { break; }
        }
        let used = 0x012D - t.left;
        self.wr(t.out, 0);
        self.wr(t.out.wrapping_add(1), 0);
        self.wr(t.out.wrapping_add(2), 0);
        Ok((out, used))
    }

    /// ROM `DF25`: store one byte of tokenised text.
    fn tok_store(&mut self, t: &mut Tok, b: u8) -> Res {
        self.wr(t.out, b);
        t.out = t.out.wrapping_add(1);
        t.left = t.left.wrapping_sub(1);
        if t.left == 0 { return Err(Exit::Error(0x17)); } // Line too long
        Ok(())
    }

    /// ROM `DEE1`: tokenise one item.
    fn tokenise_item(&mut self, t: &mut Tok) -> Res {
        self.call_hook(HOOK_TOKENISE);
        let c = self.rd(t.src);
        if c == 0 { return Ok(()); }
        if is_letter(c) { return self.tokenise_word(t); }          // DF09
        if is_digit_or_point(c) { return self.tokenise_number(t); } // DFFF
        if c == b'&' { return self.tokenise_based_number(t); }      // E05A
        t.src = t.src.wrapping_add(1);
        if c >= 0x80 { return Ok(()); }                  // dropped
        if c != b' ' { return self.tokenise_symbol(t, c); }        // E080
        if self.rd(EXT_FLAG) != 0 { return Ok(()); }
        self.tok_store(t, b' ')
    }

    /// ROM `DF09`: a word: keyword, function, or variable name.
    fn tokenise_word(&mut self, t: &mut Tok) -> Res {
        let Some(tok) = self.tokenise_keyword_or_name(t)? else { return Ok(()) };
        if tok == 0xC5 {                                 // REM: the rest of the line as it is
            return self.tok_copy_rest_of_line(t, tok);
        }
        if self.in_list(VERBATIM_KEYWORDS, tok) {        // DATA, DEF...: rest of statement
            return self.tok_copy_statement(t, tok);
        }
        if tok == 0x97 { self.tok_store(t, 0x01)?; }     // ELSE is stored as ": ELSE"
        self.tok_store(t, tok)
    }

    /// ROM `DF35`: store `tok`, then copy up to the next `:` (outside strings) as typed.
    fn tok_copy_statement(&mut self, t: &mut Tok, tok: u8) -> Res {
        self.tok_store(t, tok)?;
        loop {
            let c = self.rd(t.src);
            if c == 0 { return Ok(()); }
            if c == b':' {
                self.wr(TOKENISER_STATE, 0);
                return Ok(());
            }
            t.src = t.src.wrapping_add(1);
            if c == b'"' { self.tok_copy_string(t)?; continue; }
            self.tok_store(t, c)?;
        }
    }

    /// ROM `DF4E`: look for a keyword at the source.  A keyword (token `80+`) is returned
    /// for the caller to store; a function (`FF n`) or a variable name is stored here and
    /// `None` returned.
    ///
    /// A keyword ending in a letter or digit is not taken if the text goes on with a name
    /// character (so `TOTAL` is a variable, not `TO TAL`) - except `FN`, which may be glued
    /// to the function name.
    fn tokenise_keyword_or_name(&mut self, t: &mut Tok) -> Res<Option<u8>> {
        self.call_hook(HOOK_KEYWORD);
        let start = t.src;
        let first = to_upper(self.rd(start));
        let list = self.keyword_list(first);              // E2DD
        if let Some((after, tok_addr, last)) = self.match_keyword(list, start.wrapping_add(1)) {
            let tok = self.rom_byte(tok_addr);
            let glued = is_name_char(last & 0x7F) && tok != 0xE4 && is_name_char(self.rd(after));
            if !glued {
                t.src = after;
                if tok & 0x80 != 0 {
                    // DFC8: a keyword; does it take line numbers?
                    let state = if self.in_list(LINE_NUMBER_KEYWORDS, tok) { 1 } else { 0 };
                    self.wr(TOKENISER_STATE, state);
                    return Ok(Some(tok));
                }
                // A function: FF n.
                self.tok_store(t, 0xFF)?;
                self.tok_store(t, tok)?;
                self.wr(TOKENISER_STATE, 0);
                return Ok(None);
            }
        }
        // DF89: a variable name.
        let mut end = start;
        while is_name_char(self.rd(end)) { end = end.wrapping_add(1); }
        let token = match suffix_type(self.rd(end)) {
            Some(ty) => { end = end.wrapping_add(1); if ty == T_REAL { 0x04 } else { ty } }
            None => 0x0D,
        };
        self.tok_store(t, token)?;
        self.tok_store(t, 0)?;
        self.tok_store(t, 0)?;
        let mut p = start;
        while is_name_char(self.rd(p)) {
            let c = self.rd(p);
            self.tok_store(t, c)?;
            p = p.wrapping_add(1);
        }
        self.tok_mark_last(t);                            // E0DF
        t.src = end;
        self.wr(TOKENISER_STATE, 0xFF);
        Ok(None)
    }

    /// ROM `DFFF`: a number (or a `.`).  After a line-number keyword it must be a line
    /// number (`1E nn`); otherwise the shortest token for its value.  Text that does not
    /// parse is copied as it is.
    fn tokenise_number(&mut self, t: &mut Tok) -> Res {
        let state = self.rd(TOKENISER_STATE);
        if state != 0 {
            // The S flag tested here is still the one from loading the state: after a
            // variable (state FF) every digit is stored as a plain character.
            let c = self.rd(t.src);
            if state & 0x80 != 0 || c == b'.' {
                t.src = t.src.wrapping_add(1);
                return self.tok_store(t, c);
            }
            return match self.parse_line_number(t.src) {  // EE04
                Ok(after) => { t.src = after; self.tok_store_acc(t, 0x1E) }
                Err(stop) => self.tok_copy_literal(t, stop),
            };
        }
        match self.parse_number_text(t.src)? {            // ECBE
            Err(stop) => self.tok_copy_literal(t, stop),
            Ok((after, _)) => {
                t.src = after;
                if self.acc_type() >= T_STR { return self.tok_store_acc(t, 0x1F); }
                let v = self.rd16(ACC);
                if v >> 8 != 0 { return self.tok_store_acc(t, 0x1A); }
                let v = v as u8;
                if v < 10 { return self.tok_store(t, v + 0x0E); }
                self.tok_store(t, 0x19)?;
                self.tok_store(t, v)
            }
        }
    }

    /// ROM `E04A`: copy the source up to `stop` as it is.
    fn tok_copy_literal(&mut self, t: &mut Tok, stop: u16) -> Res {
        loop {
            let c = self.rd(t.src);
            t.src = t.src.wrapping_add(1);
            self.tok_store(t, c)?;
            if t.src == stop { return Ok(()); }
        }
    }

    /// ROM `E05A`: `&`, `&H` or `&X` followed by digits.
    fn tokenise_based_number(&mut self, t: &mut Tok) -> Res {
        match self.parse_number_text(t.src)? {
            Err(stop) => self.tok_copy_literal(t, stop),
            Ok((after, base)) => {
                t.src = after;
                self.tok_store_acc(t, if base == 2 { 0x1B } else { 0x1C })
            }
        }
    }

    /// ROM `E069`: store `token` and the accumulator's value bytes.
    fn tok_store_acc(&mut self, t: &mut Tok, token: u8) -> Res {
        self.tok_store(t, token)?;
        let n = self.acc_type() as u16;
        for i in 0..n {
            let b = self.rd(ACC + i);
            self.tok_store(t, b)?;
        }
        Ok(())
    }

    /// ROM `E080`: a symbol: a string, `|NAME`, `?` (= `PRINT`), an operator or other
    /// punctuation.  Control characters become spaces.  `'` is stored as `: '` and takes
    /// the rest of the line verbatim.
    fn tokenise_symbol(&mut self, t: &mut Tok, c: u8) -> Res {
        if c == b'"' {
            self.tok_store(t, c)?;
            return self.tok_copy_string_body(t);
        }
        if c == b'|' { return self.tokenise_rsx(t); }
        let (tok, state) = if c == b'?' {
            (0xBF, 0)                                     // PRINT; the state becomes 0
        } else {
            t.src = t.src.wrapping_sub(1);
            let tok = match self.match_keyword(SYMBOL_LIST, t.src) {
                Some((after, tok_addr, _)) => { t.src = after; self.rom_byte(tok_addr) }
                None => {
                    let mut ch = self.rd(t.src);
                    if ch < 0x20 { ch = b' '; }
                    t.src = t.src.wrapping_add(1);
                    ch
                }
            };
            (tok, self.state_after_symbol(tok))           // E0B3
        };
        self.wr(TOKENISER_STATE, state);
        if tok == 0xC0 {
            // E0E6: ' as ": '"
            self.tok_store(t, 0x01)?;
            return self.tok_copy_rest_of_line(t, 0xC0);
        }
        self.tok_store(t, tok)
    }

    /// ROM `E0B3`: the tokeniser state after a symbol: `:` and `#` reset it, a symbol
    /// after a variable resets it, otherwise it is unchanged.
    fn state_after_symbol(&self, tok: u8) -> u8 {
        if tok == 0x01 || tok == b'#' { return 0; }
        let s = self.rd(TOKENISER_STATE);
        if s == 0xFF { 0 } else { s }
    }

    /// ROM `E0BF`: having stored the opening `"`, copy a string literal up to and
    /// including its closing quote (or the end of the line).
    fn tok_copy_string_body(&mut self, t: &mut Tok) -> Res {
        loop {
            let c = self.rd(t.src);
            if c == 0 { return Ok(()); }
            t.src = t.src.wrapping_add(1);
            self.tok_store(t, c)?;
            if c == b'"' { return Ok(()); }
        }
    }

    /// ROM `E0BF` entered with the opening quote not yet stored.
    fn tok_copy_string(&mut self, t: &mut Tok) -> Res {
        self.tok_store(t, b'"')?;
        self.tok_copy_string_body(t)
    }

    /// ROM `E0CD`: `|NAME` is stored as `7C 00 NAME` (last letter with bit 7).
    fn tokenise_rsx(&mut self, t: &mut Tok) -> Res {
        self.tok_store(t, b'|')?;
        self.wr(TOKENISER_STATE, 0);
        let mut c = 0u8;
        loop {
            self.tok_store(t, c)?;
            // FF7B upper-cases the character it tests, and that is what gets stored.
            c = to_upper(self.rd(t.src));
            t.src = t.src.wrapping_add(1);
            if !is_name_char(c) { break; }
        }
        t.src = t.src.wrapping_sub(1);
        self.tok_mark_last(t);
        Ok(())
    }

    /// ROM `E0DF`: set bit 7 of the last byte stored.
    fn tok_mark_last(&mut self, t: &mut Tok) {
        let a = t.out.wrapping_sub(1);
        let v = self.rd(a) | 0x80;
        self.wr(a, v);
    }

    /// ROM `E0ED`: store `tok`, then the rest of the line as it is (`REM`, `'`).
    fn tok_copy_rest_of_line(&mut self, t: &mut Tok, tok: u8) -> Res {
        self.tok_store(t, tok)?;
        loop {
            let c = self.rd(t.src);
            if c == 0 { return Ok(()); }
            t.src = t.src.wrapping_add(1);
            self.tok_store(t, c)?;
        }
    }

    // ===============================================================================
    // Keyword tables   (E2DD .. E353)
    // ===============================================================================

    /// ROM `E2DD`: the keyword list for initial letter `letter` (`A..Z`).
    pub fn keyword_list(&self, letter: u8) -> u16 {
        self.rom_word(KEYWORD_INDEX + 2 * (letter.wrapping_sub(b'A')) as u16)
    }

    /// ROM `E327`: try every keyword of `list` against the text at `text`.  On a match
    /// returns `(text after the keyword, address of its token, its last character)`.
    /// Letters match in either case; a TAB or space in the keyword skips white space.
    pub fn match_keyword(&self, list: u16, text: u16) -> Option<(u16, u16, u8)> {
        let mut k = list;
        loop {
            if self.rom_byte(k) == 0 { return None; }
            let mut p = text;
            loop {
                let kc = self.rom_byte(k);
                k += 1;
                if kc == 0x09 || kc == b' ' {
                    p = self.skip_white_at(p);
                    continue;
                }
                let tc = to_upper(self.rd(p));
                p = p.wrapping_add(1);
                let d = tc ^ kc;
                if d == 0 { continue; }
                if d & 0x7F == 0 { return Some((p, k, kc)); }  // last character matched
                // mismatch: skip the rest of this keyword and its token
                k -= 1;
                while self.rom_byte(k) & 0x80 == 0 { k += 1; }
                k += 2;
                break;
            }
        }
    }

    /// `DD61` on an arbitrary address.
    fn skip_white_at(&self, mut p: u16) -> u16 {
        loop {
            let c = self.rd(p);
            if c != b' ' && c != 0x09 && c != 0x0A { return p; }
            p = p.wrapping_add(1);
        }
    }

    /// ROM `E2ED`: find the text of keyword `tok`: `(initial letter or 0 for a symbol,
    /// address of the rest of the keyword)`.  Not found: the extension hook `AC19`, then
    /// *Syntax error*.
    pub fn keyword_text(&mut self, tok: u8) -> Res<(u8, u16)> {
        let mut list = KEYWORD_LISTS;
        for b in (1..=26u8).rev() {
            if let Some(at) = self.find_in_list(list, tok) { return Ok((b + 0x40, at)); }
            // skip to the end of this list
            let mut a = list;
            while self.rom_byte(a) != 0 {
                while self.rom_byte(a) & 0x80 == 0 { a += 1; }
                a += 2;
            }
            list = a + 1;
        }
        if let Some(at) = self.find_in_list(SYMBOL_LIST, tok) { return Ok((0, at)); }
        self.call_hook(HOOK_NO_KEYWORD);                  // E30B
        Err(Exit::Error(2))
    }

    /// ROM `E313`: the keyword with token `tok` in one list.
    fn find_in_list(&self, list: u16, tok: u8) -> Option<u16> {
        let mut a = list;
        loop {
            if self.rom_byte(a) == 0 { return None; }
            let start = a;
            while self.rom_byte(a) & 0x80 == 0 { a += 1; }
            a += 1;
            let t = self.rom_byte(a);
            a += 1;
            if t == tok { return Some(start); }
        }
    }

    // ===============================================================================
    // De-tokeniser   (E163 .. E2DC)
    // ===============================================================================

    /// ROM `E163`: turn the program line at `line` (its length word) back into text in
    /// the input buffer: the line number, a space, the statements.  At most 255
    /// characters are produced (further characters overwrite the last one).
    pub fn detokenise_line(&mut self, line: u16) -> Res<u16> {
        let mut d = Detok { out: INPUT_BUFFER, room: 0, space: 0 };
        let number = self.rd16(line.wrapping_add(2));
        let mut p = line.wrapping_add(4);
        self.set_acc_int(number);
        let mut s = self.format_unsigned_acc()?;          // EE82
        loop {
            let c = self.rd(s);
            s = s.wrapping_add(1);
            if c == 0 { break; }
            self.detok_store(&mut d, c);
        }
        self.detok_store(&mut d, b' ');
        while self.rd(p) != 0 {
            p = self.detokenise_item(&mut d, p)?;         // E196
        }
        self.wr(d.out, 0);
        Ok(INPUT_BUFFER)
    }

    /// ROM `E1FE`: store a character of the listing.
    fn detok_store(&mut self, d: &mut Detok, c: u8) {
        self.wr(d.out, c);
        d.out = d.out.wrapping_add(1);
        d.room = d.room.wrapping_sub(1);
        if d.room == 0 {
            d.out = d.out.wrapping_sub(1);
            d.room = 1;
        }
    }

    /// ROM `E21A`: a separating space if the previous item ended in a letter or digit.
    /// (The flag is consumed either way.)
    fn detok_space(&mut self, d: &mut Detok) {
        d.space = d.space.wrapping_sub(1);
        if d.space == 0 { self.detok_store(d, b' '); }
    }

    /// ROM `E196`: one item of tokenised text at `p`; returns the address after it.
    fn detokenise_item(&mut self, d: &mut Detok, mut p: u16) -> Res<u16> {
        self.call_hook(HOOK_DETOKENISE);
        let c = self.rd(p);
        if c >= 0x80 { return self.detok_keyword(d, p, c); }       // E220
        if c < 0x02 {
            // E1BD: ':' - but not before ' or ELSE, which were stored with one
            let n = self.rd(p.wrapping_add(1));
            if n == 0xC0 || n == 0x97 { return self.detok_keyword(d, p.wrapping_add(1), n); }
            return Ok(self.detok_literal(d, p, b':'));
        }
        if c < 0x05 || (0x0B..0x0E).contains(&c) { return Ok(self.detok_variable(d, p, c)); }
        if c < 0x0B { return Ok(self.detok_literal(d, p, c)); }
        if c < 0x20 {
            // E1DE: a number
            self.detok_space(d);
            p = self.detok_number(d, p)?;                 // E253
            d.space = 1;
            return Ok(p);
        }
        if c == b'|' { return Ok(self.detok_rsx(d, p)); }
        if suffix_type(c).is_some() { self.detok_space(d); }
        Ok(self.detok_literal(d, p, c))
    }

    /// ROM `E1CA`: a character as it is; a string literal is copied to its closing quote.
    fn detok_literal(&mut self, d: &mut Detok, mut p: u16, c: u8) -> u16 {
        d.space = 0;
        if c == b'"' {
            loop {
                let ch = self.rd(p);
                self.detok_store(d, ch);
                p = p.wrapping_add(1);
                let n = self.rd(p);
                if n == 0 { return p; }
                if n == b'"' { break; }
            }
            let q = self.rd(p);
            self.detok_store(d, q);
            return p.wrapping_add(1);
        }
        self.detok_store(d, c);
        p.wrapping_add(1)
    }

    /// ROM `E1E7`: a variable: its name, and its suffix if it had one.
    fn detok_variable(&mut self, d: &mut Detok, p: u16, tok: u8) -> u16 {
        self.detok_space(d);
        let p = self.detok_name(d, p.wrapping_add(3));    // E20F
        d.space = 1;
        if tok >= 0x0B { return p; }
        d.space = 0;
        self.detok_store(d, (tok ^ 0x27) & 0xFD);         // % $ !
        p
    }

    /// ROM `E205`: `|NAME`.
    fn detok_rsx(&mut self, d: &mut Detok, p: u16) -> u16 {
        d.space = 1;
        self.detok_store(d, b'|');
        let z = self.rd(p.wrapping_add(1));
        let p = p.wrapping_add(2);
        if z != 0 { return p; }
        self.detok_name(d, p)
    }

    /// ROM `E20F`: copy a name up to and including the character with bit 7 set.
    fn detok_name(&mut self, d: &mut Detok, mut p: u16) -> u16 {
        loop {
            let raw = self.rd(p);
            let c = raw & 0x7F;
            self.detok_store(d, c);
            p = p.wrapping_add(1);
            if raw != c { return p; }
        }
    }

    /// ROM `E220`: a keyword (or `FF n` function).
    fn detok_keyword(&mut self, d: &mut Detok, mut p: u16, mut tok: u8) -> Res<u16> {
        p = p.wrapping_add(1);
        if tok == 0xFF {
            tok = self.rd(p);
            p = p.wrapping_add(1);
        }
        let (letter, mut k) = self.keyword_text(tok)?;    // E2ED
        if letter != 0 {
            self.detok_space(d);
            self.detok_store(d, letter);
        }
        let mut last;
        loop {
            let raw = self.rom_byte(k);
            last = raw & 0x7F;
            if last != 0x09 { self.detok_store(d, last); }
            k += 1;
            if raw != last { break; }
        }
        d.space = if is_name_char(last) { 1 } else { 0 };
        if tok == 0xE4 { d.space = 0; }                   // FN: no space before the name
        Ok(p)
    }

    /// ROM `E253`: a number token at `p`; returns the address after it.
    fn detok_number(&mut self, d: &mut Detok, p: u16) -> Res<u16> {
        let tok = self.rd(p);
        let mut q = p.wrapping_add(1);
        let text = match tok {
            0x1B | 0x1C => {
                let (after, digits) = if tok == 0x1B { self.binary_digits(q, 2, 0)? } else { self.hex_digits(q, 2, 0)? };
                q = after;
                self.detok_store(d, b'&');
                if tok == 0x1B { self.detok_store(d, b'X'); }
                digits
            }
            0x1D | 0x1E => {
                let mut v = self.rd16(q);
                q = q.wrapping_add(2);
                if tok == 0x1D { v = self.rd16(v.wrapping_add(3)); }
                self.set_acc_int(v);
                self.format_unsigned_acc()?               // EE82
            }
            0x1F => {
                self.set_acc_from(q, T_REAL);             // FF4B: leaves DE = ACC + 5
                q = q.wrapping_add(5);
                self.format_acc_trimmed_hl(ACC + 5)?      // EE8F (HL = that DE)
            }
            _ => {
                let v = match tok {
                    0x19 => { let v = self.rd(q) as u16; q = q.wrapping_add(1); v }
                    0x1A => { let v = self.rd16(q); q = q.wrapping_add(2); v }
                    _ => (tok - 0x0E) as u16,
                };
                self.set_acc_int(v);
                self.format_acc_trimmed()?
            }
        };
        // E2D2: copy the digits (at least one character)
        let mut s = text;
        loop {
            let c = self.rd(s);
            s = s.wrapping_add(1);
            self.detok_store(d, c);
            if self.rd(s) == 0 { break; }
        }
        Ok(q)
    }
}

/// Tokeniser registers: source (`HL`), output (`DE`), room left (`BC`).
struct Tok {
    src: u16,
    out: u16,
    left: u16,
}

/// De-tokeniser registers: output (`BC`), room counter (`D`), "space wanted" (`E`).
struct Detok {
    out: u16,
    room: u8,
    space: u8,
}

/// ROM `DFEA`: the type selected by a suffix character `%` (2), `$` (3), `!` (5).
pub fn suffix_type(c: u8) -> Option<u8> {
    match c {
        b'%' => Some(T_INT),
        b'$' => Some(T_STR),
        b'!' => Some(T_REAL),
        _ => None,
    }
}
