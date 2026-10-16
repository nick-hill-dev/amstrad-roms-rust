//! # Streams, character output and the screen commands  (ROM `C19D .. C48B`)
//!
//! BASIC has ten streams: `#0..#7` are text windows of the screen, `#8` the printer and
//! `#9` the cassette file.  [`OUT_STREAM`] / [`IN_STREAM`] hold the current ones; for
//! the screen streams the firmware's selected text stream is switched as well.
//!
//! All output goes through [`Basic::print_char`] (`C356`), which turns the newline
//! character `0A` into the right line ending for the stream (`CR LF` for the screen, the
//! printer and files) and keeps the column counters of the printer and the file that
//! `POS` and `WIDTH` use.  The screen keeps its own cursor.
//!
//! ## Commands taking `#stream`
//!
//! `PEN`, `PAPER`, `CLS`, `LOCATE`, `WINDOW`, `TAG`, `TAGOFF` accept an optional
//! `#n,` prefix that must name a screen stream.  The ROM implements this with a curious
//! idiom (`C1D0`): the helper pops its caller's return address, selects the stream, calls
//! the *rest of the caller* as a subroutine and then restores the stream.  Here that is
//! [`Basic::with_screen_stream`] taking the rest of the statement as a closure.
//!
//! ## ESC
//!
//! While BASIC runs, the break key is armed with the event routine at `C45E`
//! ([`Basic::arm_break`]).  The first ESC pauses (sound held, cursor shown); a second ESC
//! breaks, any other key continues.  Output loops that may take long (the printer, `LIST`)
//! poll for ESC themselves with [`Basic::poll_escape`].

use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// ROM select byte passed with the break routine: the upper ROM (BASIC) enabled.
pub const ROM_SELECT_UPPER: u8 = 0xFD;
/// Address of the break event routine in the ROM.
pub const BREAK_ROUTINE: u16 = 0xC45E;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Stream selection   (C19D .. C1FF)
    // ===============================================================================

    /// ROM `C19D`: input and output both stream 0.
    pub fn select_stream_0(&mut self) {
        self.set_input_stream(0);
        self.set_output_stream(0);
    }

    /// ROM `C1A2`: make `n` the output stream (selecting the text window for a screen
    /// stream).  Returns the previous output stream.
    pub fn set_output_stream(&mut self, n: u8) -> u8 {
        if n < 8 { self.txt_str_select(n); }
        let old = self.rd(OUT_STREAM);
        self.wr(OUT_STREAM, n);
        old
    }

    /// ROM `C1AF`: make `n` the input stream; returns the previous one.
    pub fn set_input_stream(&mut self, n: u8) -> u8 {
        let old = self.rd(IN_STREAM);
        self.wr(IN_STREAM, n);
        old
    }

    /// ROM `C1BA` (carry): is the output stream a screen window?
    pub fn output_is_screen(&self) -> bool { self.rd(OUT_STREAM) < 8 }

    /// ROM `C1C0` (carry): is the input stream the keyboard (not the cassette file)?
    pub fn input_is_keyboard(&self) -> bool { self.rd(IN_STREAM) < 9 }

    /// ROM `C1C6`: parse an optional `#n,` and make it the output stream.  Returns the
    /// previous output stream (to be restored by the caller).
    pub fn output_stream_prefix(&mut self) -> Res<u8> {
        let n = self.stream_prefix()?;
        Ok(self.set_output_stream(n))
    }

    /// ROM `C1CB`: parse an optional `#n,` and make it the input stream.
    pub fn input_stream_prefix(&mut self) -> Res<u8> {
        let n = self.stream_prefix()?;
        Ok(self.set_input_stream(n))
    }

    /// ROM `C1D0`: parse an optional `#n,` naming a *screen* stream (else *Improper
    /// argument*), select it, run `rest` and select the previous output stream again.
    pub fn with_screen_stream(&mut self, rest: impl FnOnce(&mut Self) -> Res) -> Res {
        let n = self.stream_prefix()?;
        if n >= 8 { return Err(Exit::Error(5)); }
        let old = self.set_output_stream(n);
        rest(self)?;
        self.set_output_stream(old);
        Ok(())
    }

    /// ROM `C1E3`: an optional `#n` at the text pointer, `n` < 10, followed either by a
    /// comma or by the end of the statement.  0 if there is no `#`.
    pub fn stream_prefix(&mut self) -> Res<u8> {
        if self.cur() != b'#' { return Ok(0); }
        let n = self.hash_stream()?;
        if !self.comma() { self.check_end()?; }
        Ok(n)
    }

    /// ROM `C1F5`: `#n` (the `#` is required), `n` < 10.
    pub fn hash_stream(&mut self) -> Res<u8> {
        self.expect(b'#')?;
        self.eval_below(10)
    }

    /// ROM `C1FB`: evaluate an integer in `0 .. limit`, else *Improper argument*.
    pub fn eval_below(&mut self, limit: u8) -> Res<u8> {
        let v = self.eval_byte()?;                        // CE67
        if v < limit { Ok(v) } else { Err(Exit::Error(5)) }
    }

    // ===============================================================================
    // Screen commands   (C20A .. C336)
    // ===============================================================================

    /// `PAPER [#s,]ink` (ROM `C20A`).
    pub fn stmt_paper(&mut self) -> Res {
        self.with_screen_stream(|b| { let v = b.eval_below(16)?; b.txt_set_paper(v); Ok(()) })
    }

    /// `PEN [#s,]ink` (ROM `C212`).
    pub fn stmt_pen(&mut self) -> Res {
        self.with_screen_stream(|b| { let v = b.eval_below(16)?; b.txt_set_pen(v); Ok(()) })
    }

    /// `BORDER colour[,colour]` (ROM `C221`).
    pub fn stmt_border(&mut self) -> Res {
        let (c1, c2) = self.colour_pair()?;
        self.scr_set_border(c1, c2);
        Ok(())
    }

    /// `INK ink,colour[,colour]` (ROM `C22A`).
    pub fn stmt_ink(&mut self) -> Res {
        let ink = self.eval_below(16)?;
        self.expect(b',')?;
        let (c1, c2) = self.colour_pair()?;
        self.scr_set_ink(ink, c1, c2);
        Ok(())
    }

    /// ROM `C23C`: one or two colours `< 32`; a single colour is used twice.
    fn colour_pair(&mut self) -> Res<(u8, u8)> {
        let first = self.eval_below(32)?;
        if !self.comma() { return Ok((first, first)); }
        Ok((first, self.eval_below(32)?))
    }

    /// `MODE n` (ROM `C24F`).
    pub fn stmt_mode(&mut self) -> Res {
        let m = self.eval_below(3)?;
        self.scr_set_mode(m);
        Ok(())
    }

    /// `CLS [#s]` (ROM `C25A`): character 12 to the stream.
    pub fn stmt_cls(&mut self) -> Res {
        self.with_screen_stream(|b| b.output_raw(0x0C))
    }

    /// `VPOS(#s)` (ROM `C262`): the cursor row in the window of a screen stream.
    pub fn fn_vpos(&mut self) -> Res {
        self.stream_function(|b| {
            if !b.output_is_screen() { return Err(Exit::Error(5)); }
            let (c, r) = b.txt_get_cursor();
            Ok(b.txt_validate(c, r).1)
        })
    }

    /// `POS(#s)` (ROM `C276`): the column of any stream.
    pub fn fn_pos(&mut self) -> Res {
        self.stream_function(|b| Ok(b.column()))
    }

    /// ROM `C279`: the common part of `POS` and `VPOS`: `#s)`, select stream `s` while
    /// `f` computes the result, which becomes the integer value of the function.
    fn stream_function(&mut self, f: impl FnOnce(&mut Self) -> Res<u8>) -> Res {
        let s = self.hash_stream()?;
        let old = self.set_output_stream(s);
        self.expect(b')')?;
        let v = f(self)?;
        self.set_acc_int(v as u16);                       // FF0A
        self.set_output_stream(old);
        Ok(())
    }

    /// ROM `C290`: the current column of the output stream (1-based for the screen).
    pub fn column(&mut self) -> u8 {
        match self.rd(OUT_STREAM) {
            8 => self.rd(PRINTER_COLUMN),
            s if s > 8 => self.rd(FILE_COLUMN),
            _ => self.cursor_column(),
        }
    }

    /// ROM `C29F`: the line width of the output stream: the window width for the screen,
    /// `WIDTH` for the printer (`None` for `WIDTH 255`), `None` for a file.
    pub fn line_width(&mut self) -> Option<u8> {
        match self.rd(OUT_STREAM) {
            8 => { let w = self.rd(PRINTER_WIDTH); if w < 0xFF { Some(w) } else { None } }
            s if s > 8 => None,
            _ => {
                let (l, _, r, _) = self.txt_get_window();
                Some(r.wrapping_sub(l).wrapping_add(1))
            }
        }
    }

    /// ROM `C2B9`/`C2BF`: will `len` more characters fit on the current line?  (Always
    /// true at the start of a line or for a stream without a width.)
    pub fn fits_on_line(&mut self, len: u8) -> bool {
        let Some(width) = self.line_width() else { return true };
        let col = self.column();
        let before = col.wrapping_sub(1);
        if before == 0 { return true; }
        let (end, overflow) = before.overflowing_add(len);
        if overflow { return false; }
        end.wrapping_sub(1) < width
    }

    /// `LOCATE [#s,]column,row` (ROM `C2D2`).
    pub fn stmt_locate(&mut self) -> Res {
        self.with_screen_stream(|b| {
            let (col, row) = b.byte_pair_minus_1()?;
            b.txt_set_cursor(col.wrapping_add(1), row.wrapping_add(1));
            Ok(())
        })
    }

    /// `WINDOW [#s,]left,right,top,bottom` (ROM `C2E1`), or `WINDOW SWAP a[,b]`.
    pub fn stmt_window(&mut self) -> Res {
        if self.cur() == 0xE7 { return self.window_swap(); } // SWAP
        self.with_screen_stream(|b| {
            let (left, right) = b.byte_pair_minus_1()?;
            b.expect(b',')?;
            let (top, bottom) = b.byte_pair_minus_1()?;
            b.txt_win_enable(left, right, top, bottom);
            Ok(())
        })
    }

    /// ROM `C2FD`: `WINDOW SWAP a[,b]` - exchange screen streams `a` and `b` (default 0).
    fn window_swap(&mut self) -> Res {
        self.next();
        let a = self.eval_below(8)?;
        let b = if self.comma() { self.eval_below(8)? } else { 0 };
        self.txt_swap_streams(b, a);
        Ok(())
    }

    /// `TAG [#s]` (ROM `C319`): text at the graphics cursor.
    pub fn stmt_tag(&mut self) -> Res {
        self.with_screen_stream(|b| { b.txt_set_graphic(0xFF); Ok(()) })
    }

    /// `TAGOFF [#s]` (ROM `C320`).
    pub fn stmt_tagoff(&mut self) -> Res {
        self.with_screen_stream(|b| { b.txt_set_graphic(0); Ok(()) })
    }

    /// ROM `C327`: two expressions `1..255` separated by a comma, each minus one.
    fn byte_pair_minus_1(&mut self) -> Res<(u8, u8)> {
        let a = self.eval_nonzero_byte()?.wrapping_sub(1);  // C32F
        self.expect(b',')?;
        let b = self.eval_nonzero_byte()?.wrapping_sub(1);
        Ok((a, b))
    }

    // ===============================================================================
    // Character output   (C337 .. C416)
    // ===============================================================================

    /// ROM `C337`: the cold-start banner: `WIDTH 132`, streams 0, print the string.
    pub fn print_banner(&mut self, text: u16) -> Res {
        self.wr(PRINTER_WIDTH, 0x84);
        self.select_stream_0();
        self.print_string(text)
    }

    /// ROM `C341`: print the zero-terminated string at `text` (in ROM or RAM).
    pub fn print_string(&mut self, text: u16) -> Res {
        let mut a = text;
        loop {
            let c = self.rd(a);
            a = a.wrapping_add(1);
            if c == 0 { return Ok(()); }
            self.print_char(c)?;
        }
    }

    /// ROM `C34E`: new line on the output stream.
    pub fn new_line(&mut self) -> Res { self.print_char(0x0A) }

    /// ROM `C356`: print a character on the output stream; `0A` means "new line".
    pub fn print_char(&mut self, c: u8) -> Res {
        if c != 0x0A { return self.output_raw(c); }
        match self.rd(OUT_STREAM) {
            8 => self.printer_newline(),
            s if s > 8 => self.file_newline(),
            _ => { self.screen_newline(); Ok(()) }
        }
    }

    /// ROM `C36E`: send a character to the output stream as it is.
    pub fn output_raw(&mut self, c: u8) -> Res {
        match self.rd(OUT_STREAM) {
            8 => self.printer_char(c),
            s if s > 8 => self.file_char(c),
            _ => { self.txt_output(c); Ok(()) }
        }
    }

    /// ROM `C386`: tidy the screen when returning to direct mode: text at the text
    /// cursor, VDU enabled, and a new line unless the cursor is in column 1.
    pub fn text_tidy(&mut self) {
        self.txt_set_graphic(0);
        self.txt_vdu_enable();
        if self.cursor_column() != 1 { self.screen_newline(); }
    }

    /// ROM `C392`: `CR LF` to the screen.
    fn screen_newline(&mut self) {
        self.txt_output(0x0D);
        self.txt_output(0x0A);
    }

    /// ROM `C39C`: the text cursor column of the current window, after wrapping.
    pub fn cursor_column(&mut self) -> u8 {
        let (c, r) = self.txt_get_cursor();
        self.txt_validate(c, r).0
    }

    /// ROM `C3A8`: `CR LF` to the printer.
    fn printer_newline(&mut self) -> Res {
        self.printer_char(0x0D)?;
        self.printer_char(0x0A)
    }

    /// ROM `C3B5`: a character to the printer, keeping the column count and starting a new
    /// line when `WIDTH` is exceeded.  Waits (polling ESC) while the printer is busy.
    ///
    /// The column counter becomes 1 after `CR`, and a printable character arriving in
    /// column `WIDTH + 1` first gets a new line.  Control characters do not count.
    fn printer_char(&mut self, c: u8) -> Res {
        if c == 0x0D {
            self.wr(PRINTER_COLUMN, 1);
        } else if c >= 0x20 {
            let col = self.rd(PRINTER_COLUMN);
            let width = self.rd(PRINTER_WIDTH);
            if width != 0xFF && col == width.wrapping_add(1) { self.printer_newline()?; }
            let col = self.rd(PRINTER_COLUMN).wrapping_add(1);
            if col != 0 { self.wr(PRINTER_COLUMN, col); }
        }
        while !self.mc_print_char(c) {
            self.poll_escape()?;                          // ESC, ESC breaks
        }
        Ok(())
    }

    /// ROM `C3DF`: the printer column.
    pub fn printer_column(&self) -> u8 { self.rd(PRINTER_COLUMN) }

    /// `WIDTH n` (ROM `C3E3`): the printer line width, 1..255.
    pub fn stmt_width(&mut self) -> Res {
        let w = self.eval_nonzero_byte()?;
        self.wr(PRINTER_WIDTH, w);
        Ok(())
    }

    /// ROM `C3EA`: `CR LF` to the cassette file; column 1.
    fn file_newline(&mut self) -> Res {
        self.wr(FILE_COLUMN, 1);
        self.file_write(0x0D)?;
        self.file_write(0x0A)
    }

    /// ROM `C3F8`: a character to the cassette file, counting columns.
    fn file_char(&mut self, c: u8) -> Res {
        if c == 0x0D {
            self.wr(FILE_COLUMN, 1);
        } else if c >= 0x20 {
            let col = self.rd(FILE_COLUMN).wrapping_add(1);
            if col != 0 { self.wr(FILE_COLUMN, col); }
        }
        self.file_write(c)
    }

    /// ROM `C40D`: `CAS OUT CHAR`; any failure (ESC on the tape prompt, an error) breaks.
    fn file_write(&mut self, c: u8) -> Res {
        if self.cas_out_char(c).is_err() { return self.do_break().map(|n| match n {}); } // JP CB6B
        Ok(())
    }

    // ===============================================================================
    // Input, EOF and ESC   (C414 .. C48B)
    // ===============================================================================

    /// ROM `C414`: put the last cassette character back (`CAS RETURN`).
    pub fn file_unread(&mut self) { self.cas_return(); }

    /// `EOF` (ROM `C417`): -1 at the end of the input file, else 0.
    pub fn fn_eof(&mut self) -> Res {
        match self.cas_test_eof() {
            Err(e) if e.is_escape() => { self.do_break().map(|n| match n {})?; }
            Ok(_) => self.set_acc_int(0),
            Err(_) => self.set_acc_int(0xFFFF),
        }
        Ok(())
    }

    /// ROM `C424`: the next character from the input stream: the cassette file, or the
    /// keyboard (waiting, with the cursor shown, if nothing is buffered).
    /// `Ok(None)` = end of file / error (carry clear).
    pub fn read_input_char(&mut self) -> Res<Option<u8>> {
        if self.rd(IN_STREAM) == 9 {
            return Ok(self.cas_in_char().ok());
        }
        if let Some(c) = self.km_read_char() { return Ok(Some(c)); }
        Ok(Some(self.wait_key_with_cursor()))
    }

    /// ROM `C430`: wait for a character with the cursor shown.
    fn wait_key_with_cursor(&mut self) -> u8 {
        self.txt_cur_on();
        let c = self.km_wait_char();
        self.txt_cur_off();
        c
    }

    /// ROM `C439`: `KM READ CHAR` (used by `INKEY$`).
    pub fn read_key(&mut self) -> Option<u8> { self.km_read_char() }

    /// ROM `C43C`: look for ESC in the keyboard buffer (other keys are thrown away).  On
    /// ESC: pause; a second ESC breaks ([`Exit`] via `CB6B`), anything else continues
    /// and re-arms the break key.
    pub fn poll_escape(&mut self) -> Res {
        let Some(c) = self.km_read_char() else { return Ok(()) };
        if c != 0xFC { return Ok(()); }
        if self.pause() { return self.do_break().map(|n| match n {}); }       // JP C,CB6B
        self.arm_break();
        Ok(())
    }

    /// ROM `C453`: arm the break key with BASIC's break routine ([`BREAK_ROUTINE`]).
    pub fn arm_break(&mut self) {
        self.km_arm_break(BREAK_ROUTINE, ROM_SELECT_UPPER);
    }

    /// The break event routine (ROM `C45E`), run by the event system when ESC is pressed
    /// while the break key is armed (`hl` = event + 4, `priority` = the `B` register):
    /// drop the keys typed so far (up to the break marker `EF`), pause, then let the event
    /// code decide (`C847`).
    pub fn break_event_routine(&mut self, hl: u16, priority: u8) -> Res {
        loop {
            match self.km_read_char() {
                None => break,
                Some(0xEF) => break,
                Some(_) => {}
            }
        }
        let broke = self.pause();
        self.break_event_decide(hl, priority, broke)      // C847
    }

    /// ROM `C46F`: the ESC pause.  Sound is held; wait for a key (with the cursor): a
    /// second ESC returns `true` (break); any other key continues - it is pushed back
    /// unless it was a space - and the sound continues.
    pub fn pause(&mut self) -> bool {
        let was_playing = self.sound_hold();
        let c = loop {
            let c = self.wait_key_with_cursor();
            if c != 0xEF { break c; }
        };
        if c == 0xFC { return true; }
        if c != b' ' { self.km_char_return(c); }
        if was_playing { self.sound_continue(); }
        false
    }
}
