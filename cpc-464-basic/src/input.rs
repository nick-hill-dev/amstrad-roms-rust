//! # `LINE INPUT`, `INPUT`, `READ`, `DATA`, `RESTORE`  (ROM `DAF8 .. DD36`)
//!
//! ## From the keyboard
//!
//! The line is read with the firmware's line editor into the input buffer (`ACA4`),
//! echoing on the screen stream given by `#n` (the input stream number doubles as the
//! window).  `INPUT` then checks the whole line against the variable list *before*
//! assigning anything: the right number of items, each parsing as its variable's type.
//! If not, `?Redo from start` and the prompt again.
//!
//! A prompt string may follow the stream: `;` after it adds `? `, `,` does not; with no
//! prompt `? ` is printed.  A `;` straight after `INPUT` suppresses the new line after
//! the typed line.
//!
//! ## From the cassette file (`#9`)
//!
//! Items are read one at a time: leading white space is skipped; a number ends at white
//! space, a comma or `CR`; a string at a comma or `CR`, or is quoted.  `CR LF` and
//! `LF CR` count as one line end.  The end of the file is *EOF met*.
//!
//! ## DATA
//!
//! `READ` keeps its place in [`DATA_PTR`] (and the `DATA` line in [`DATA_LINE`], for the
//! error message).  `DATA` statements are copied verbatim by the tokeniser, so their
//! items are parsed like typed input; unquoted strings end at a comma, `:` or the end of
//! the line.
//!
//! Quirk: the `:` that ends a `DATA` statement is tokenised (as `01`), but an item must be
//! followed by a comma or the end of the line, so `READ`ing the last item of
//! `DATA 1,2:PRINT` is a *Syntax error* (in the `DATA` line).  An unquoted string item
//! there would even run on to the end of the line.

use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// "?Redo from start\n".
const MSG_REDO: u16 = 0xDB77;
/// The `DATA` token.
const TOK_DATA: u8 = 0x8C;

/// The item-end tests the file reader is given (`DCC6`, `DCCA`, `DCCD`, `DC63`).
#[derive(Clone, Copy, PartialEq, Eq)]
enum ItemEnd {
    /// `DCC6`: white space, a comma or `CR` (a number).
    Number,
    /// `DCCA`: a comma or `CR` (an unquoted string).
    Unquoted,
    /// `DCCD`: `CR` (a whole line).
    Line,
    /// `DC63`: the closing quote.
    Quote,
}

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // LINE INPUT   (DAF8 .. DB2A)
    // ===============================================================================

    /// `LINE INPUT [#s,][;]["prompt";]var$` (ROM `DAF8`): a whole line into a string
    /// variable.
    pub fn stmt_line(&mut self) -> Res {
        self.expect(0xA3)?;                               // INPUT
        let old_in = self.input_stream_prefix()?;         // C1CB
        self.input_prompt()?;                             // DB89
        let (var, _) = self.find_variable()?;             // D686
        self.require_string()?;
        let text = self.ptr;
        self.read_whole_line()?;                          // DB1A
        self.buffer_string(INPUT_BUFFER)?;                // F7DC
        self.assign_same_type(var)?;                      // D66F
        self.ptr = text;
        self.set_input_stream(old_in);
        Ok(())
    }

    /// ROM `DB1A`: a line from the input stream into the input buffer.
    fn read_whole_line(&mut self) -> Res {
        if !self.input_is_keyboard() { return self.read_file_item(ItemEnd::Line).map(|_| ()); }   // DC66
        let echo = self.rd(IN_STREAM);
        let old_out = self.set_output_stream(echo);
        self.read_keyboard_line()?;                       // DBAD
        self.set_output_stream(old_out);
        Ok(())
    }

    // ===============================================================================
    // INPUT   (DB2B .. DBD2)
    // ===============================================================================

    /// `INPUT [#s,][;]["prompt"{;|,}]var[,var...]` (ROM `DB2B`).
    pub fn stmt_input(&mut self) -> Res {
        let old_in = self.input_stream_prefix()?;
        let mut data = self.input_line()?;                // DB47
        loop {
            let (var, _) = self.find_variable()?;
            let text = self.ptr;
            self.ptr = data;
            self.read_item(var, 0)?;                      // DBBC
            data = self.ptr;
            self.ptr = text;
            if !self.comma() { break; }
        }
        self.set_input_stream(old_in);
        Ok(())
    }

    /// ROM `DB47`: the prompt; from the keyboard, read lines until one fits the variable
    /// list.  Returns where the items start (the input buffer).
    fn input_line(&mut self) -> Res<u16> {
        if !self.input_is_keyboard() {
            self.input_prompt()?;
            // DE is not used for file input: the items are read as they are needed.
            return Ok(INPUT_BUFFER);
        }
        let echo = self.rd(IN_STREAM);
        let old_out = self.set_output_stream(echo);
        let prompt_at = self.ptr;
        loop {
            self.ptr = prompt_at;
            if !self.input_prompt()? {
                self.print_char(b'?')?;
                self.print_char(b' ')?;
            }
            let vars = self.ptr;
            self.read_keyboard_line()?;                   // DBAD
            self.ptr = vars;
            if self.input_fits(INPUT_BUFFER)? { break; }  // DBD3
            self.print_string(MSG_REDO)?;
        }
        self.set_output_stream(old_out);
        Ok(INPUT_BUFFER)
    }

    /// ROM `DB89`: `[;]["prompt"{;|,}]`.  Records the character after the stream (a `;`
    /// means "no new line after the input") in [`INPUT_SEPARATOR`].  The prompt is
    /// printed for keyboard input.  Returns `true` (carry) when a prompt ended with a
    /// comma, i.e. no `? `.
    fn input_prompt(&mut self) -> Res<bool> {
        let c = self.cur();
        self.wr(INPUT_SEPARATOR, c);
        let c = if c == b';' { self.next() } else { c };
        if c != b'"' { return Ok(false); }
        self.string_literal()?;                           // F7CB
        if self.input_is_keyboard() { self.print_acc_string()?; } else { self.pop_string()?; }
        if self.comma() { return Ok(true); }
        self.expect(b';')?;
        Ok(false)
    }

    /// ROM `DBAD`: read a line from the keyboard (ESC is a *Break*), then a new line
    /// unless the statement said `;`.
    fn read_keyboard_line(&mut self) -> Res {
        if !self.read_input_line()? { return self.do_break().map(|n| match n {}); }   // CA3B / CB6B
        if self.rd(INPUT_SEPARATOR) != b';' { self.new_line()?; }
        Ok(())
    }

    /// ROM `DBBC`: read the next item at the data pointer (the text pointer here) as the
    /// value of the variable at `var` and assign it (*Type mismatch* if it does not
    /// parse).  `end` ends an unquoted string (`:` for `DATA`).  Afterwards the pointer
    /// is past the character that ended the item; returns `true` if that was a comma or
    /// the end of the text.
    fn read_item(&mut self, var: u16, end: u8) -> Res<bool> {
        let Some(t) = self.read_value(end)? else { return Err(Exit::Error(0x0D)) };   // DC02
        self.assign(var, t)?;                             // D666
        let c = self.cur();
        self.ptr = self.ptr.wrapping_add(1);
        Ok(c == 0 || c == b',')
    }

    /// ROM `DBD3`: does the typed line at `data` fit the variable list at the text
    /// pointer?  (The names are only scanned; the values are parsed and thrown away.)
    fn input_fits(&mut self, data: u16) -> Res<bool> {
        let vars = self.ptr;
        let mut d = data;
        let fits = loop {
            self.skip_variable()?;                        // D6D6
            let v = self.ptr;
            self.ptr = d;
            let Some(t) = self.read_value(0)? else { break false };
            if t == T_STR { self.pop_string()?; }
            d = self.ptr;
            self.ptr = v;
            let more = self.comma();
            let v = self.ptr;
            self.ptr = d;
            let c = self.skip_white();                    // DD61
            if !more { break c == 0; }
            if c != b',' { break false; }
            d = self.ptr.wrapping_add(1);
            self.ptr = v;
        };
        self.ptr = vars;
        Ok(fits)
    }

    /// ROM `DC02`: read a value for a variable of the type in [`ACC_TYPE`] at the text
    /// pointer (from the keyboard buffer or a `DATA` statement), or the next item of the
    /// input file.  Strings always succeed; a number that does not parse gives `None`.
    /// Returns the variable's type.
    fn read_value(&mut self, end: u8) -> Res<Option<u8>> {
        let t = self.acc_type();
        let ok = if t == T_STR {
            if self.input_is_keyboard() {
                if self.skip_white() == b'"' {
                    self.string_literal()?;               // F7CB
                } else {
                    let p = self.ptr;
                    self.ptr = self.item_string(p, end)?; // F7E6
                }
            } else {
                self.read_file_string()?;                 // DC47
                self.ptr = self.buffer_string(INPUT_BUFFER)?;   // F7DC
            }
            true
        } else {
            if !self.input_is_keyboard() {
                self.read_file_item(ItemEnd::Number)?;    // DC38
                self.ptr = INPUT_BUFFER;
            }
            match self.parse_signed_number(self.ptr)? {   // ECA3
                Some(after) => { self.ptr = after; true }
                None => false,
            }
        };
        if ok { self.skip_white(); }
        Ok(if ok { Some(t) } else { None })
    }

    // ===============================================================================
    // Reading the input file   (DC21 .. DCD8)
    // ===============================================================================

    /// ROM `DC47`: a string item from the file into the input buffer (quoted or not).
    fn read_file_string(&mut self) -> Res {
        let Some(c) = self.skip_file_white()? else { return Err(Exit::Error(0x18)) };   // DC9D
        if c != b'"' {
            self.read_item_chars(c, ItemEnd::Unquoted)?;
            return Ok(());
        }
        match self.read_file_char()? {                   // DCA8
            Some(c) => self.read_item_chars(c, ItemEnd::Quote),
            None => { self.wr(INPUT_BUFFER, 0); Ok(()) }
        }
    }

    /// ROM `DC38` / `DC66`: a number item (`Number`) or a whole line (`Line`) from the
    /// file into the input buffer.  *EOF met* if the file has ended.
    fn read_file_item(&mut self, kind: ItemEnd) -> Res<()> {
        let first = if kind == ItemEnd::Line { self.read_file_char()? } else { self.skip_file_white()? };
        let Some(c) = first else { return Err(Exit::Error(0x18)) };
        self.read_item_chars(c, kind)
    }

    /// ROM `DC6E`: store characters, starting with `c`, in the input buffer until `kind`'s
    /// end test is met (at most 255; the end of the file also ends it), then deal with
    /// what follows: after a closing quote or white space, skip white space and a comma
    /// (anything else is put back).
    fn read_item_chars(&mut self, mut c: u8, kind: ItemEnd) -> Res {
        let mut p = INPUT_BUFFER;
        let mut room: u8 = 0xFF;
        let ended = loop {
            if item_ends(kind, c) { break true; }
            self.wr(p, c);
            p = p.wrapping_add(1);
            room -= 1;
            if room == 0 { break false; }
            match self.read_file_char()? {
                Some(n) => c = n,
                None => break false,
            }
        };
        self.wr(p, 0);
        if !ended || c == 0x0D { return Ok(()); }
        if c != b'"' && !is_white(c) { return Ok(()); }
        // DC92
        let Some(c) = self.skip_file_white()? else { return Ok(()) };
        if c != b',' && c != 0x0D { self.file_unread(); }  // C414
        Ok(())
    }

    /// ROM `DC9D`: the next character of the file that is not white space.
    fn skip_file_white(&mut self) -> Res<Option<u8>> {
        loop {
            match self.read_file_char()? {
                None => return Ok(None),
                Some(c) if is_white(c) => {}
                Some(c) => return Ok(Some(c)),
            }
        }
    }

    /// ROM `DCA8`: the next character of the input stream; `CR LF` and `LF CR` are read as
    /// one `CR` / `LF`.
    fn read_file_char(&mut self) -> Res<Option<u8>> {
        let Some(c) = self.read_input_char()? else { return Ok(None) };   // C424
        let pair = match c { 0x0D => 0x0A, 0x0A => 0x0D, _ => return Ok(Some(c)) };
        if let Some(n) = self.read_input_char()? {
            if n != pair { self.file_unread(); }
        }
        Ok(Some(c))
    }

    // ===============================================================================
    // RESTORE, READ   (DCD9 .. DD36)
    // ===============================================================================

    /// `DATA` (ROM `E8EF`): skipped when executed.
    pub fn stmt_data(&mut self) -> Res {
        self.skip_to_end_of_statement();
        Ok(())
    }

    /// `RESTORE [line]` (ROM `DCD9`).
    pub fn stmt_restore(&mut self) -> Res {
        if crate::is_end(self.cur()) {
            self.restore_data_start();
            return Ok(());
        }
        let n = self.read_line_number_constant()?;        // CEE1
        let at = self.find_line_or_error(n)?;             // E79A
        self.wr16(DATA_PTR, at.wrapping_sub(1));
        Ok(())
    }

    /// ROM `DCE5`: `READ` starts at the beginning of the program.
    pub fn restore_data_start(&mut self) {
        let base = self.rd16(PROGRAM_BASE);
        self.wr16(DATA_PTR, base);
    }

    /// `READ var[,var...]` (ROM `DCEB`).  An item that is not followed by a comma or the
    /// end of the `DATA` statement is a *Syntax error* reported in the `DATA` line.
    pub fn stmt_read(&mut self) -> Res {
        let mut data = self.rd16(DATA_PTR);
        loop {
            data = self.next_data_item(data)?;            // DD17
            let (var, _) = self.find_variable()?;
            let text = self.ptr;
            self.ptr = data.wrapping_add(1);
            let fine = self.read_item(var, b':')?;        // DBBC
            data = self.ptr.wrapping_sub(1);
            if !fine {
                let line = self.rd16(DATA_LINE);
                self.set_current_line(line);
                return Err(Exit::Error(2));
            }
            self.ptr = text;
            if !self.comma() { break; }
        }
        self.wr16(DATA_PTR, data);
        Ok(())
    }

    /// ROM `DD17`: from `p`, the next item: `p` itself if it is on a comma, else the next
    /// `DATA` statement (its token).  *DATA exhausted* at the end of the program.
    fn next_data_item(&mut self, p: u16) -> Res<u16> {
        if self.rd(p) == b',' { return Ok(p); }
        let text = self.ptr;
        self.ptr = p;
        loop {
            let c = self.skip_to_end_of_statement();      // E8EF
            if c == 0 {
                let at = self.ptr;
                if self.rd16(at.wrapping_add(1)) == 0 { return Err(Exit::Error(4)); }
                self.wr16(DATA_LINE, at.wrapping_add(3));
                self.ptr = at.wrapping_add(4);
            }
            if self.next() == TOK_DATA { break; }
        }
        let at = self.ptr;
        self.ptr = text;
        Ok(at)
    }
}

/// The end tests of the file reader.
fn item_ends(kind: ItemEnd, c: u8) -> bool {
    match kind {
        ItemEnd::Number => is_white(c) || c == b',' || c == 0x0D,
        ItemEnd::Unquoted => c == b',' || c == 0x0D,
        ItemEnd::Line => c == 0x0D,
        ItemEnd::Quote => c == b'"',
    }
}

/// ROM `DCD0`: space, TAB or LF.
fn is_white(c: u8) -> bool { c == b' ' || c == 0x09 || c == 0x0A }
