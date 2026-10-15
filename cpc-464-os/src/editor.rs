//! # Line editor  (`BD3A`, ROM `2A98 .. 2E0C`)
//!
//! The editor BASIC uses for `INPUT` and for editing a program line.  The caller gives
//! the address of a zero-terminated text buffer (it may already hold text to edit); the
//! editor shows it, lets the user move around, insert, overwrite and delete, and returns
//! when ENTER or ESC is pressed.
//!
//! ## Z80 registers while editing
//!
//! | reg | meaning in the ROM                    | here              |
//! |-----|---------------------------------------|-------------------|
//! | `HL`| address of the character at the cursor| [`Ed::hl`]        |
//! | `B` | cursor index in the buffer            | [`Ed::b`]         |
//! | `C` | number of characters in the buffer    | [`Ed::c`]         |
//!
//! ## The COPY cursor
//!
//! Besides the normal text cursor the editor has a second, inverse-video block cursor
//! (`B8DE`) that the user moves with SHIFT+arrows and whose character is copied into the
//! line with COPY.  Everything the editor prints therefore goes through
//! [`Cpc::ed_put_char`], which hides that cursor first and shifts it if the screen scrolls.
//!
//! ## Exit values
//!
//! The ROM ends with `CP FC`: *carry* means ENTER (key `0D`), *zero* means ESC (`FC`).
//! A key table entry may also leave the loop with carry set by accident: TAB-like key
//! `E1` pressed on an *empty* line (see [`Cpc::ed_toggle_overwrite`]).

use crate::sysvars::*;
use crate::{Cpc, Hardware};

/// Result of [`Cpc::km_edit_line`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EditOutcome {
    /// Value of `A` on return (the terminating key).
    pub key: u8,
    /// Carry flag: `key < 0xFC` – normally "ENTER was pressed".
    pub carry: bool,
    /// Zero flag: `key == 0xFC` – ESC was pressed.
    pub zero: bool,
}

/// The editor's three working registers.
#[derive(Clone, Copy, Debug)]
struct Ed {
    /// Address in the buffer of the character under the cursor.
    hl: u16,
    /// Cursor index.
    b: u8,
    /// Buffer length.
    c: u8,
}

/// What a key handler tells the main loop (the carry flag on return).
enum Flow {
    /// Carry clear: fetch the next key.
    Continue,
    /// Carry set: leave the editor with this value in `A`.
    Exit(u8),
}

/// "ESC" message.
const BREAK_TEXT: &[u8] = b"*Break*";

impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Entry point
    // ===============================================================================

    /// `BD3A` (ROM `2A98`): edit the zero-terminated line at `buffer`.
    ///
    /// The buffer holds at most 255 characters (the length counter is a byte).
    pub fn km_edit_line(&mut self, buffer: u16) -> EditOutcome {
        // Length of the initial text -> C; the cursor starts at the beginning.
        let mut c = 0u8;
        let mut p = buffer;
        while self.read(p) != 0 {
            c = c.wrapping_add(1);
            p = p.wrapping_add(1);
        }
        let mut st = Ed { hl: buffer, b: 0, c };
        self.wr8(ED_OVERWRITE, 0);          // insert mode
        self.wr16(ED_COPY_CURSOR, 0);       // no copy cursor yet
        self.ed_print_string(buffer);       // show the text already in the buffer

        let a = loop {
            let key = self.ed_get_key();
            if let Flow::Exit(a) = self.ed_dispatch(key, &mut st) { break a; }
        };
        self.ed_toggle_copy_cursor();       // hide the copy cursor
        EditOutcome { key: a, carry: a < 0xFC, zero: a == 0xFC }
    }

    // ===============================================================================
    // Key dispatch   (ROM 2AC6, tables at 2AE0 and 2B1C)
    // ===============================================================================

    /// Choose the key table and run the handler.
    ///
    /// Two tables exist: the normal one, and one used when the line is *empty* and a
    /// plain cursor key (`F0..F3`) arrives – then the cursor keys move the screen cursor
    /// by sending control codes.
    ///
    /// ROM quirk (kept): the handlers are entered with the flags left by the table
    /// selection.  With an empty line and a key below `F0`, carry is **set** on entry;
    /// only a handler that does not change the flags can pass it on (see
    /// [`Self::ed_toggle_overwrite`]).
    fn ed_dispatch(&mut self, key: u8, st: &mut Ed) -> Flow {
        let empty = st.b == 0 && st.c == 0;
        if empty && (0xF0..0xF4).contains(&key) {
            // table 2 (ROM 2B1C): cursor keys on an empty line move the screen cursor
            let code = match key { 0xF0 => 0x0B, 0xF1 => 0x0A, 0xF2 => 0x08, _ => 0x09 };
            self.txt_output(code);
            return Flow::Continue;
        }
        let carry_in = empty && key < 0xF0;
        match key {
            0xFC => self.ed_escape(st),                                // ESC
            0xEF => Flow::Continue,                                    // ignored
            0x0D => self.ed_enter(st, key),                            // ENTER
            0xF0 => self.ed_up(st),
            0xF1 => self.ed_down(st),
            0xF2 => self.ed_left(st),
            0xF3 => self.ed_right(st),
            0xF8 => { let c = st.c; self.ed_move_left(st, c); Flow::Continue }   // CTRL+up: to start
            0xF9 => { let c = st.c; self.ed_move_right(st, c); Flow::Continue }  // CTRL+down: to end
            0xFA => self.ed_line_start(st),                            // CTRL+left
            0xFB => self.ed_line_end(st),                              // CTRL+right
            0xF4 => { self.ed_move_copy_cursor(0, 0xFF); Flow::Continue }        // SHIFT+up
            0xF5 => { self.ed_move_copy_cursor(0, 1); Flow::Continue }           // SHIFT+down
            0xF6 => { self.ed_move_copy_cursor(0xFF, 0); Flow::Continue }        // SHIFT+left
            0xF7 => { self.ed_move_copy_cursor(1, 0); Flow::Continue }           // SHIFT+right
            0xE0 => self.ed_copy_key(st),                              // COPY
            0x7F => self.ed_delete(st, true),                          // DEL
            0x10 => self.ed_delete(st, false),                         // CLR
            0xE1 => self.ed_toggle_overwrite(carry_in),
            _ => self.ed_insert(st, key),                              // everything else
        }
    }

    // ===============================================================================
    // Terminating keys
    // ===============================================================================

    /// `2B42`: ESC – print the rest of the line, then `*Break*` on its own line.
    fn ed_escape(&mut self, st: &Ed) -> Flow {
        self.ed_print_raw(st.hl);
        for &b in BREAK_TEXT { self.ed_put_char(b); }
        let (col, _, _) = self.txt_get_cursor();
        if col != 1 {
            self.txt_output(0x0D);
            self.txt_output(0x0A);
        }
        Flow::Exit(0xFC)
    }

    /// `2B69`: ENTER – print the rest of the line so the cursor ends up after it.
    fn ed_enter(&mut self, st: &Ed, key: u8) -> Flow {
        self.ed_print_raw(st.hl);
        Flow::Exit(key)
    }

    /// `2B69` loop: print a zero-terminated string with [`Self::ed_put_char`].
    fn ed_print_raw(&mut self, mut p: u16) {
        loop {
            let ch = self.read(p);
            p = p.wrapping_add(1);
            if ch == 0 { break; }
            self.ed_put_char(ch);
        }
    }

    /// `2B2B`: error beep.
    fn ed_beep(&mut self) -> Flow {
        self.txt_output(0x07);
        Flow::Continue
    }

    // ===============================================================================
    // Cursor movement inside the line
    // ===============================================================================

    /// `2BEB`: `(D, E)` = (width of the text window, current column).
    fn ed_window_width_and_column(&mut self) -> (u8, u8) {
        let ((left, _), (right, _), _) = self.txt_get_window();
        let (col, _, _) = self.txt_get_cursor();
        (right.wrapping_sub(left).wrapping_add(1), col)
    }

    /// `2B75`: cursor right one character; beep at the end of the line.
    fn ed_right(&mut self, st: &mut Ed) -> Flow {
        if self.ed_move_right(st, 1) { self.ed_beep() } else { Flow::Continue }
    }

    /// `2BAA`: cursor left one character; beep at the start of the line.
    fn ed_left(&mut self, st: &mut Ed) -> Flow {
        if self.ed_move_left(st, 1) { self.ed_beep() } else { Flow::Continue }
    }

    /// `2B7E`: down one screen line = right by the window width (beep if the rest of
    /// the line is shorter than that).
    fn ed_down(&mut self, st: &mut Ed) -> Flow {
        let (width, _) = self.ed_window_width_and_column();
        if st.c.wrapping_sub(st.b) < width { return self.ed_beep(); }
        self.ed_move_right(st, width);
        Flow::Continue
    }

    /// `2BB3`: up one screen line = left by the window width.
    fn ed_up(&mut self, st: &mut Ed) -> Flow {
        let (width, _) = self.ed_window_width_and_column();
        if st.b < width { return self.ed_beep(); }
        self.ed_move_left(st, width);
        Flow::Continue
    }

    /// `2B89`: CTRL+right – to the end of the screen line.
    fn ed_line_end(&mut self, st: &mut Ed) -> Flow {
        let (width, col) = self.ed_window_width_and_column();
        let n = width.wrapping_sub(col);
        if n == 0 { return Flow::Continue; }
        self.ed_move_right(st, n);
        Flow::Continue
    }

    /// `2BBD`: CTRL+left – to the start of the screen line.
    fn ed_line_start(&mut self, st: &mut Ed) -> Flow {
        let (_, col) = self.ed_window_width_and_column();
        let n = col.wrapping_sub(1);
        if n == 0 { return Flow::Continue; }
        self.ed_move_left(st, n);
        Flow::Continue
    }

    /// `2B93`: move right `d` characters.  Returns the Z flag: `true` if the end of
    /// the line was reached (at the start of a step).
    ///
    /// When the screen cursor cannot simply step right (window edge) the character is
    /// printed instead, which wraps to the next line, and the rest is redrawn.
    fn ed_move_right(&mut self, st: &mut Ed, mut d: u8) -> bool {
        loop {
            if st.b == st.c { return true; }
            let inside = self.ed_step(1, 0x09);
            if !inside {
                let ch = self.read(st.hl);
                self.ed_put_char(ch);
            }
            st.b = st.b.wrapping_add(1);
            st.hl = st.hl.wrapping_add(1);
            if !inside { self.ed_print_string(st.hl); }
            d = d.wrapping_sub(1);
            if d == 0 { return false; }
        }
    }

    /// `2BC8`: move left `d` characters.  Returns the Z flag: `true` if already at the
    /// start of the line.
    fn ed_move_left(&mut self, st: &mut Ed, mut d: u8) -> bool {
        loop {
            if st.b == 0 { return true; }
            if self.ed_step(0xFF, 0x08) {
                st.b = st.b.wrapping_sub(1);
                st.hl = st.hl.wrapping_sub(1);
                d = d.wrapping_sub(1);
                if d == 0 { return false; }
                continue;
            }
            // 2BD7: the screen cursor is at the window edge and cannot step left.
            loop {
                if st.b == 0 { break; }
                st.b = st.b.wrapping_sub(1);
                st.hl = st.hl.wrapping_sub(1);
                self.ed_move_copy_cursor_column(1);
                d = d.wrapping_sub(1);
                if d == 0 { break; }
            }
            self.ed_print_string(st.hl);
            return false;
        }
    }

    /// `2D4A` (`d = FF, e = 08`) / `2D50` (`d = 01, e = 09`): if the position one
    /// column left/right of the cursor is inside the window, send control code `e`
    /// (BS / TAB) and return `true`.
    fn ed_step(&mut self, d: u8, e: u8) -> bool {
        let (col, row, _) = self.txt_get_cursor();
        let (_, _, inside) = self.txt_validate(col.wrapping_add(d), row);
        if inside { self.txt_output(e); }
        inside
    }

    // ===============================================================================
    // Inserting and deleting
    // ===============================================================================

    /// `2BF9`: toggle insert/overwrite mode.
    ///
    /// ROM quirk (kept): the routine does not touch the flags, so `carry_in` – set when
    /// the line is empty – becomes the "carry" that ends the editor, with `A` = the
    /// new (complemented) mode byte.
    fn ed_toggle_overwrite(&mut self, carry_in: bool) -> Flow {
        let v = !self.rd8(ED_OVERWRITE);
        self.wr8(ED_OVERWRITE, v);
        if carry_in { Flow::Exit(v) } else { Flow::Continue }
    }

    /// `2C01`: a normal character – insert it (or overwrite) at the cursor.
    fn ed_insert(&mut self, st: &mut Ed, ch: u8) -> Flow {
        if ch == 0 { return Flow::Continue; }
        let overwrite = self.rd8(ED_OVERWRITE) != 0 && st.b != st.c;
        if overwrite {
            self.wr8(st.hl, ch);
            self.ed_put_char(ch);
            st.hl = st.hl.wrapping_add(1);
            st.b = st.b.wrapping_add(1);
            return Flow::Continue;
        }
        if st.c == 0xFF { return self.ed_beep(); }       // buffer full
        self.wr8(ED_COPY_MOVED, 0);
        self.ed_put_char(ch);
        st.c = st.c.wrapping_add(1);
        // Shift the rest of the line (and the terminator) one place to the right.
        let mut p = st.hl;
        let mut carry = ch;
        loop {
            let old = self.rd8(p);
            self.wr8(p, carry);
            carry = old;
            p = p.wrapping_add(1);
            if old == 0 { break; }
        }
        self.wr8(p, 0);
        st.b = st.b.wrapping_add(1);
        st.hl = st.hl.wrapping_add(1);
        self.ed_print_string(st.hl);                     // redraw the tail
        if self.rd8(ED_COPY_MOVED) != 0 { self.ed_move_copy_cursor_column(1); }
        Flow::Continue
    }

    /// `2C3D` (DEL, `back = true`) and `2C4A` (CLR, `back = false`): delete the
    /// character before / under the cursor.
    fn ed_delete(&mut self, st: &mut Ed, back: bool) -> Flow {
        if back {
            if st.b == 0 { return self.ed_beep(); }
            if !self.ed_step(0xFF, 0x08) { return self.ed_beep(); }
            st.b = st.b.wrapping_sub(1);
            st.hl = st.hl.wrapping_sub(1);
        }
        if st.b == st.c { return self.ed_beep(); }
        // Close the gap, then reprint with a trailing space to wipe the last character.
        let mut p = st.hl;
        loop {
            let next = self.rd8(p.wrapping_add(1));
            self.wr8(p, next);
            if next == 0 { break; }
            p = p.wrapping_add(1);
        }
        self.wr8(p, b' ');
        self.wr8(ED_COPY_MOVED, 0);
        self.ed_print_string(st.hl);
        self.wr8(p, 0);
        st.c = st.c.wrapping_sub(1);
        if self.rd8(ED_COPY_MOVED) != 0 { self.ed_move_copy_cursor_column(0xFF); }
        Flow::Continue
    }

    // ===============================================================================
    // The COPY cursor
    // ===============================================================================

    /// `2CEA`: COPY key – take the character under the copy cursor and insert it.
    /// With no copy cursor and an empty line the copy cursor starts at the text cursor.
    fn ed_copy_key(&mut self, st: &mut Ed) -> Flow {
        let (cc, cr, _) = self.txt_get_cursor();
        let w = self.rd16(ED_COPY_CURSOR);
        if w != 0 {
            self.txt_set_cursor((w >> 8) as u8, w as u8);
            self.txt_place_cursor();                      // un-draw the copy cursor
        } else {
            if st.b != 0 || st.c != 0 { return self.ed_beep(); }
            let (c, r, _) = self.txt_get_cursor();
            self.wr16(ED_COPY_CURSOR, (c as u16) << 8 | r as u16);
        }
        let found = self.txt_rd_char();
        self.txt_set_cursor(cc, cr);
        let w = self.rd16(ED_COPY_CURSOR);
        let (c, r, inside) = self.txt_validate(((w >> 8) as u8).wrapping_add(1), w as u8);
        if inside { self.wr16(ED_COPY_CURSOR, (c as u16) << 8 | r as u16); }
        self.ed_toggle_copy_cursor();
        match found {
            Some(ch) => self.ed_insert(st, ch),
            None => self.ed_beep(),
        }
    }

    /// `2C98..2CAA`: SHIFT+cursor – move the copy cursor by (`dcol`, `drow`).
    fn ed_move_copy_cursor(&mut self, dcol: u8, drow: u8) {
        let w = self.rd16(ED_COPY_CURSOR);
        let (col, row) = if w == 0 {
            let (c, r, _) = self.txt_get_cursor();
            (c, r)
        } else {
            ((w >> 8) as u8, w as u8)
        };
        let (c, r, inside) = self.txt_validate(col.wrapping_add(dcol), row.wrapping_add(drow));
        if inside {
            self.ed_toggle_copy_cursor();
            self.wr16(ED_COPY_CURSOR, (c as u16) << 8 | r as u16);
            self.ed_toggle_copy_cursor();
        }
    }

    /// `2D29` (`d = 1`) / `2D2D` (`d = FF`): shift the copy cursor sideways by `d`
    /// columns, keeping it on the same text.
    fn ed_move_copy_cursor_column(&mut self, d: u8) {
        self.ed_toggle_copy_cursor();
        let w = self.rd16(ED_COPY_CURSOR);
        if w != 0 {
            self.ed_copy_cursor_store(((w >> 8) as u8).wrapping_add(d), w as u8);
            self.ed_toggle_copy_cursor();
        }
    }

    /// `2C82`: the screen scrolled by `delta` lines – move the copy cursor with the text.
    fn ed_scroll_copy_cursor(&mut self, delta: u8) {
        let w = self.rd16(ED_COPY_CURSOR);
        if w == 0 { return; }
        self.ed_copy_cursor_store((w >> 8) as u8, (w as u8).wrapping_add(delta));
    }

    /// `2C8C..2C97`: validate a copy cursor position and store it; a position outside
    /// the window removes the copy cursor (stores zero).
    fn ed_copy_cursor_store(&mut self, col: u8, row: u8) {
        let (c, r, inside) = self.txt_validate(col, row);
        let w = if inside { (c as u16) << 8 | r as u16 } else { 0 };
        self.wr16(ED_COPY_CURSOR, w);
    }

    /// `2CCD` / `2CD2`: draw or erase the copy cursor (it is an inverse-video block, so
    /// the same call does both).  Does nothing if there is no copy cursor.
    fn ed_toggle_copy_cursor(&mut self) {
        let w = self.rd16(ED_COPY_CURSOR);
        if w == 0 { return; }
        let (c, r, _) = self.txt_get_cursor();
        self.txt_set_cursor((w >> 8) as u8, w as u8);
        self.txt_place_cursor();
        self.txt_set_cursor(c, r);
    }

    // ===============================================================================
    // Output helpers that keep the copy cursor consistent
    // ===============================================================================

    /// `2DA8`: print one character at the text cursor (no control codes).  If the copy
    /// cursor is on that cell it is hidden first and redrawn afterwards; a scroll shifts it.
    fn ed_put_char(&mut self, ch: u8) {
        let (col, row, scroll) = self.txt_get_cursor();
        let (c, r, inside) = self.txt_validate(col, row);
        let on_copy = inside && self.rd16(ED_COPY_CURSOR) == ((c as u16) << 8 | r as u16);
        if on_copy { self.ed_toggle_copy_cursor(); }
        self.txt_wr_char(ch);
        let (_, _, now) = self.txt_get_cursor();
        let delta = now.wrapping_sub(scroll);
        if delta != 0 { self.ed_scroll_copy_cursor(delta); }
        if on_copy {
            self.wr8(ED_COPY_MOVED, 0xFF);
            self.ed_toggle_copy_cursor();
        }
    }

    /// `2D67`: print a zero-terminated string at the cursor, then put the cursor back
    /// where it started (allowing for any scrolling).
    fn ed_print_string(&mut self, mut p: u16) {
        let (col, row, scroll0) = self.txt_get_cursor();
        loop {
            let ch = self.read(p);
            p = p.wrapping_add(1);
            if ch == 0 { break; }
            self.ed_print_visible(ch, col, row, scroll0);
        }
        let (_, _, now) = self.txt_get_cursor();
        self.txt_set_cursor(col, row.wrapping_add(now.wrapping_sub(scroll0)));
    }

    /// `2D85`: print one character of a redraw, but only while the redraw's start
    /// position (`col`, `row` corrected for scrolling) is still inside the window.
    ///
    /// ROM quirk (kept): when the *current* cursor is outside the window the start row
    /// is nudged by `2*B + 1` before the check, meaning to use the character in `B`.  But
    /// `TXT VALIDATE` has just overwritten `B` with its scroll direction (`FF` = below the
    /// window, `00` = above), so the nudge is really `-1` or `+1` row.
    fn ed_print_visible(&mut self, ch: u8, col: u8, row: u8, scroll0: u8) {
        let (cc, cr, scroll) = self.txt_get_cursor();
        let mut e = scroll.wrapping_sub(scroll0).wrapping_add(row);
        if let Some(b) = self.txt_validate_b(cc, cr) {
            e = b.wrapping_add(b).wrapping_add(1).wrapping_add(e);
        }
        let (_, _, ok) = self.txt_validate(col, e);
        if ok { self.ed_put_char(ch); }
    }

    /// `2DD9`: wait for a key with the text cursor shown (unless the copy cursor is
    /// exactly where the text cursor is).
    fn ed_get_key(&mut self) -> u8 {
        let (col, row, scroll) = self.txt_get_cursor();
        let (c, r, _) = self.txt_validate(col, row);
        if self.rd16(ED_COPY_CURSOR) == ((c as u16) << 8 | r as u16) {
            return self.km_wait_char();
        }
        self.txt_cur_on();
        let (_, _, now) = self.txt_get_cursor();
        let delta = now.wrapping_sub(scroll);
        if delta != 0 { self.ed_scroll_copy_cursor(delta); }
        let key = self.km_wait_char();
        self.txt_cur_off();
        key
    }
}
