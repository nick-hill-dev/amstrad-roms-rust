//! # Text VDU  (ROM `1078 .. 15AF`)
//!
//! Character-cell output: eight independent *streams* (windows), each with its own
//! cursor, pen, paper, window and transparency; control-code ("VDU") interpretation;
//! user-definable character matrices.
//!
//! ## Stream blocks
//!
//! The *selected* stream lives in a 15-byte "working copy" at `B285` (see
//! [`StreamOffsets`](crate::sysvars::StreamOffsets)); the other seven are parked in
//! slots starting at `B20D`, 15 bytes apart.  [`Cpc::txt_str_select`] swaps the working
//! copy with a slot.  Slot *n* is only up to date while stream *n* is **not** selected.
//!
//! ## Positions
//!
//! Inside the ROM a cursor position is a Z80 register pair `HL` with `H` = column and
//! `L` = row, stored little-endian so the *row* is at the lower address (`B285`) and the
//! column at `B286`.  Public functions in this module take `(col, row)` in that order.
//! Positions held in the working copy are **absolute** screen cells (0-based).  The
//! public cursor API (`TXT SET CURSOR`, `TXT GET CURSOR`, `TXT VALIDATE`) converts to and
//! from *window-relative, 1-based* coordinates, exactly as the firmware does.
//!
//! ## Deferred wrapping
//!
//! Printing a character leaves the cursor one cell to the right even if that is outside
//! the window.  Wrapping to the next line (and scrolling) is only done the *next* time
//! the position is used ([`Cpc::txt_fix_cursor`], ROM `11AB`).  This is why a program may
//! print in the last column of the last line without the screen scrolling immediately.
//!
//! ## Indirections
//!
//! Five routines can be replaced by a program: draw cursor (`BDCD`), undraw cursor
//! (`BDD0`), write character (`BDD3`), read character (`BDD6`) and the output action
//! (`BDD9`, which interprets control codes).  See [`crate::Hooks`].

use crate::hw::{Hardware, Regs};
use crate::screen::AccessMode;
use crate::sysvars::*;
use crate::Cpc;

// ---- constants --------------------------------------------------------------------

/// ROM address of the opaque ("paper behind the pen") character writer, as stored in
/// the back-vector `B291`.
const BACK_OPAQUE: u16 = 0x1391;
/// ROM address of the transparent character writer.
const BACK_TRANSPARENT: u16 = 0x139F;

/// Base of the built-in 256-character font inside the ROM image.
pub const FONT_BASE: u16 = 0x3800;

/// Default control-code table (ROM `146B`, 32 entries of *parameter count* + *handler
/// address*).  It is copied to `B2C3` by [`Cpc::txt_reset`]; a program may patch entries
/// found through [`Cpc::txt_get_controls`].
///
/// | Code | Params | Action                                     |
/// |------|--------|--------------------------------------------|
/// | 00   | 0      | NUL – ignored                              |
/// | 01   | 1      | print the parameter as a character         |
/// | 02   | 0      | disable cursor                             |
/// | 03   | 0      | enable cursor                              |
/// | 04   | 1      | `MODE n`                                   |
/// | 05   | 1      | write the parameter at the *graphics* cursor |
/// | 06   | 0      | enable VDU                                 |
/// | 07   | 0      | BEL – beep                                 |
/// | 08-0B| 0      | cursor left / right / down / up            |
/// | 0C   | 0      | clear window (CLS)                         |
/// | 0D   | 0      | carriage return                            |
/// | 0E   | 1      | set paper                                  |
/// | 0F   | 1      | set pen                                    |
/// | 10   | 0      | clear character under cursor               |
/// | 11   | 0      | clear from start of line to cursor         |
/// | 12   | 0      | clear from cursor to end of line           |
/// | 13   | 0      | clear from start of window to cursor       |
/// | 14   | 0      | clear from cursor to end of window         |
/// | 15   | 0      | disable VDU                                |
/// | 16   | 1      | transparent mode (bit 0)                   |
/// | 17   | 1      | graphics pixel write mode (`SCR ACCESS`)   |
/// | 18   | 0      | exchange pen and paper                     |
/// | 19   | 9      | define character matrix (char + 8 bytes)   |
/// | 1A   | 4      | set window (left, right, top, bottom)      |
/// | 1B   | 0      | ESC – ignored                              |
/// | 1C   | 3      | set ink (pen, colour 1, colour 2)          |
/// | 1D   | 2      | set border (colour 1, colour 2)            |
/// | 1E   | 0      | home cursor                                |
/// | 1F   | 2      | locate (column, row)                       |
const DEFAULT_CONTROLS: [(u8, u16); 32] = [
    (0, H_NOP), (1, H_WR_CHAR), (0, H_CUR_DISABLE), (0, H_CUR_ENABLE),
    (1, H_SET_MODE), (1, H_GRA_WR_CHAR), (0, H_VDU_ENABLE), (0, H_BEL),
    (0, H_LEFT), (0, H_RIGHT), (0, H_DOWN), (0, H_UP),
    (0, H_CLEAR_WINDOW), (0, H_CR), (1, H_SET_PAPER), (1, H_SET_PEN),
    (0, H_CLEAR_CHAR), (0, H_CLEAR_LINE_START), (0, H_CLEAR_LINE_END), (0, H_CLEAR_SCREEN_START),
    (0, H_CLEAR_SCREEN_END), (0, H_VDU_DISABLE), (1, H_TRANSPARENT), (1, H_SCR_ACCESS),
    (0, H_INVERSE), (9, H_DEFINE_MATRIX), (4, H_WINDOW), (0, H_NOP),
    (3, H_SET_INK), (2, H_SET_BORDER), (0, H_HOME), (2, H_LOCATE),
];

// ROM addresses of the control-code handlers (used as the identity of a handler: the
// table in RAM stores addresses, and Rust dispatches on them).
const H_NOP: u16 = 0x14E2;
const H_WR_CHAR: u16 = 0x1334;
const H_CUR_DISABLE: u16 = 0x129A;
const H_CUR_ENABLE: u16 = 0x1289;
const H_SET_MODE: u16 = 0x0ACA;
const H_GRA_WR_CHAR: u16 = 0x1945;
const H_VDU_ENABLE: u16 = 0x1451;
const H_BEL: u16 = 0x14D8;
const H_LEFT: u16 = 0x150A;
const H_RIGHT: u16 = 0x150F;
const H_DOWN: u16 = 0x1514;
const H_UP: u16 = 0x1519;
const H_CLEAR_WINDOW: u16 = 0x1540;
const H_CR: u16 = 0x1530;
const H_SET_PAPER: u16 = 0x12AE;
const H_SET_PEN: u16 = 0x12A9;
const H_CLEAR_CHAR: u16 = 0x154F;
const H_CLEAR_LINE_START: u16 = 0x158E;
const H_CLEAR_LINE_END: u16 = 0x1584;
const H_CLEAR_SCREEN_START: u16 = 0x156D;
const H_CLEAR_SCREEN_END: u16 = 0x1556;
const H_VDU_DISABLE: u16 = 0x144B;
const H_TRANSPARENT: u16 = 0x14E3;
const H_SCR_ACCESS: u16 = 0x0C49;
const H_INVERSE: u16 = 0x12C9;
const H_DEFINE_MATRIX: u16 = 0x1504;
const H_WINDOW: u16 = 0x14F8;
const H_SET_INK: u16 = 0x14E8;
const H_SET_BORDER: u16 = 0x14F1;
const H_HOME: u16 = 0x152A;
const H_LOCATE: u16 = 0x1538;

/// The sound queued by BEL (ROM `14CF`): channel A, tone period 0x5A, volume 11, 20 ticks.
const BEL_SOUND: [u8; 9] = [0x87, 0x00, 0x00, 0x5A, 0x00, 0x00, 0x0B, 0x14, 0x00];

/// Where a cell's matrix lives: address plus whether it is in the user table (RAM) or in
/// the built-in font (ROM).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MatrixRef {
    pub addr: u16,
    pub user_defined: bool,
}

/// Result of checking a cursor against the window ([`Cpc::wrap_position`], ROM `11DA`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Wrap {
    /// Still inside the window.
    Inside,
    /// Fell off the window vertically: the window must scroll.  `up == true` means the
    /// cursor went below the bottom (text moves up), otherwise above the top.
    Scroll { up: bool },
}

/// Z80 `CP`/`JP P` idiom: true when the sign bit of `a - b` is clear.
#[inline]
fn sign_pos(a: u8, b: u8) -> bool { a.wrapping_sub(b) & 0x80 == 0 }

/// Clip a window coordinate to `0..=max` (ROM `1244` / `124D`).  Negative (bit 7 set)
/// values become 0.
#[inline]
fn clip(v: u8, max: u8) -> u8 {
    let v = if v & 0x80 != 0 { 0 } else { v };
    v.min(max)
}

impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Small helpers
    // ===============================================================================

    /// Cursor of the selected stream as `(column, row)` – absolute, 0-based.
    #[inline]
    fn txt_cursor_abs(&self) -> (u8, u8) { (self.rd8(TXT_CURSOR_COL), self.rd8(TXT_CURSOR_ROW)) }

    #[inline]
    fn txt_set_cursor_abs(&mut self, col: u8, row: u8) {
        self.wr8(TXT_CURSOR_ROW, row);
        self.wr8(TXT_CURSOR_COL, col);
    }

    /// Copy `len` bytes inside RAM (what `LDIR` does between two RAM addresses).
    fn ram_copy(&mut self, src: u16, dst: u16, len: u16) {
        for i in 0..len {
            let b = self.rd8(src.wrapping_add(i));
            self.wr8(dst.wrapping_add(i), b);
        }
    }

    /// Call a user-patched indirection as machine code.
    fn txt_call_hook(&mut self, addr: u16, regs: &mut Regs) { self.hw.call(addr, regs); }

    /// `BDCD` – draw the cursor (default ROM `1263`).
    fn txt_draw_cursor(&mut self) {
        if let Some(a) = self.patched(Ind::TXT_DRAW_CURSOR, 0x1263) {
            self.txt_call_hook(a, &mut Regs::default());
        } else if self.rd8(TXT_CURSOR_FLAGS) == 0 {          // 1263: only when cursor on+enabled
            self.txt_place_cursor();
        }
    }

    /// `BDD0` – remove the cursor (default ROM `1263`, same routine: the cursor is an
    /// XOR block, so drawing it twice removes it).
    fn txt_undraw_cursor(&mut self) {
        if let Some(a) = self.patched(Ind::TXT_UNDRAW_CURSOR, 0x1263) {
            self.txt_call_hook(a, &mut Regs::default());
        } else if self.rd8(TXT_CURSOR_FLAGS) == 0 {
            self.txt_place_cursor();
        }
    }

    // ===============================================================================
    // Initialisation and stream management                          (1078 .. 113D)
    // ===============================================================================

    /// `TXT INITIALISE` (`BB4E`, ROM `1078`): [`Self::txt_reset`], no user-defined
    /// characters, default stream (paper 0, pen 1, full-screen window), then copy that
    /// state to all eight streams and select stream 0.
    pub fn txt_initialise(&mut self) {
        self.txt_reset();
        self.wr8(TXT_MATRIX_FLAG, 0);
        self.txt_init_stream(0, 1);                          // L_113D with HL = 0001
        // L_10A3: replicate the working copy into the eight stream slots.
        for n in 0..8u16 {
            self.ram_copy(TXT_STREAM, TXT_STREAM_SAVE + n * TXT_STREAM_SIZE, TXT_STREAM_SIZE);
        }
        self.wr8(TXT_CURRENT_STREAM, 0);
    }

    /// `TXT RESET` (`BB51`, ROM `1088`): restore the five default indirections
    /// (`BDCD..BDDB`) and the default control-code table.
    pub fn txt_reset(&mut self) {
        // L_0A8A copies "JP 1263 / JP 1263 / JP 134A / JP 13C0 / JP 140C" to BDCD.
        let defaults = [
            (Ind::TXT_DRAW_CURSOR, 0x1263u16), (Ind::TXT_UNDRAW_CURSOR, 0x1263),
            (Ind::TXT_WRITE_CHAR, 0x134A), (Ind::TXT_UNWRITE, 0x13C0),
            (Ind::TXT_OUT_ACTION, 0x140C),
        ];
        for (addr, target) in defaults {
            self.wr8(addr, 0xC3);
            self.wr16(addr + 1, target);
        }
        // L_145B: reset the control sequence collector and the control table.
        self.wr8(TXT_CTRL_COUNT, 0);
        for (i, &(n, h)) in DEFAULT_CONTROLS.iter().enumerate() {
            let a = TXT_CTRL_TABLE + (i as u16) * 3;
            self.wr8(a, n);
            self.wr16(a + 1, h);
        }
    }

    /// ROM `113D`: give the *selected* stream its initial state – cursor disabled+off,
    /// the given `paper` / `pen` (pen **numbers**), no graphics mode, opaque writing,
    /// full-screen window, VDU enabled.
    fn txt_init_stream(&mut self, paper: u8, pen: u8) {
        self.wr8(TXT_CURSOR_FLAGS, 3);
        self.txt_set_paper(paper);
        self.txt_set_pen(pen);
        self.txt_set_graphic(0);
        self.txt_set_back(0);
        self.txt_win_enable(0, 0x7F, 0, 0x7F);               // clipped to the real screen
        self.txt_vdu_enable();
    }

    /// ROM `10B7`: called by `SCR SET MODE` before the mode changes.  For every stream
    /// the cursor is removed and the pen and paper are turned from *encoded* bytes back
    /// into pen **numbers** (the encoding depends on the mode).  Returns the stream that
    /// was selected so that [`Self::txt_restore_streams`] can reselect it.
    pub(crate) fn txt_save_pen_paper(&mut self) -> u8 {
        let current = self.rd8(TXT_CURRENT_STREAM);
        for n in (0..8u8).rev() {
            self.txt_str_select(n);
            self.txt_undraw_cursor();
            let paper = self.txt_get_paper();
            self.wr8(TXT_PAPER, paper);
            let pen = self.txt_get_pen();
            self.wr8(TXT_PEN, pen);
        }
        current
    }

    /// ROM `10D5`: counterpart of [`Self::txt_save_pen_paper`], run after the mode has
    /// changed.  Every stream is re-initialised ([`Self::txt_init_stream`]) with its
    /// saved pen and paper numbers, which resets its window, cursor and transparency.
    pub(crate) fn txt_restore_streams(&mut self, stream: u8) {
        for n in (0..8u8).rev() {
            self.txt_str_select(n);
            let pen = self.rd8(TXT_PEN);
            let paper = self.rd8(TXT_PAPER);
            self.txt_init_stream(paper, pen);
        }
        self.txt_str_select(stream);
    }

    /// Address of the parked state of stream `n` (ROM `112A`).
    fn txt_stream_slot(n: u8) -> u16 {
        // Z80: 8-bit arithmetic 15*n + 0x0D with the carry added to the high byte.
        TXT_STREAM_SAVE + (n & 7) as u16 * TXT_STREAM_SIZE
    }

    /// `TXT STR SELECT` (`BB9F`, ROM `10E8`): make stream `n & 7` the selected one.
    /// Returns the previously selected stream.
    pub fn txt_str_select(&mut self, n: u8) -> u8 {
        let n = n & 7;
        let old = self.rd8(TXT_CURRENT_STREAM);
        if n == old { return old; }
        self.wr8(TXT_CURRENT_STREAM, n);
        self.ram_copy(TXT_STREAM, Self::txt_stream_slot(old), TXT_STREAM_SIZE);   // park old
        self.ram_copy(Self::txt_stream_slot(n), TXT_STREAM, TXT_STREAM_SIZE);     // load new
        old
    }

    /// `TXT SWAP STREAMS` (`BBA2`, ROM `1107`): exchange the complete state of streams
    /// `b` and `c`.  The currently selected stream *number* is unchanged (but its state
    /// may now be the other stream's).
    pub fn txt_swap_streams(&mut self, b: u8, c: u8) {
        let original = self.rd8(TXT_CURRENT_STREAM);
        self.txt_str_select(c);
        self.wr8(TXT_CURRENT_STREAM, b & 7);                 // pretend B is selected, no copy
        let slot_b = Self::txt_stream_slot(b);
        let slot_c = Self::txt_stream_slot(c);
        self.ram_copy(slot_b, slot_c, TXT_STREAM_SIZE);      // slot[C] = slot[B]
        self.txt_str_select(original);                       // parks the working copy in slot[B]
    }

    // ===============================================================================
    // Cursor position                                                  (115E .. 1263)
    // ===============================================================================

    /// `TXT SET COLUMN` (`BB6F`, ROM `115E`): move to window column `col` (1-based).
    pub fn txt_set_column(&mut self, col: u8) {
        let (_, row) = self.txt_cursor_abs();
        let col = col.wrapping_sub(1).wrapping_add(self.rd8(TXT_WIN_LEFT));
        self.txt_move_cursor_to(col, row);
    }

    /// `TXT SET ROW` (`BB72`, ROM `1169`): move to window row `row` (1-based).
    pub fn txt_set_row(&mut self, row: u8) {
        let (col, _) = self.txt_cursor_abs();
        let row = row.wrapping_sub(1).wrapping_add(self.rd8(TXT_WIN_TOP));
        self.txt_move_cursor_to(col, row);
    }

    /// `TXT SET CURSOR` (`BB75`, ROM `1174`): position the cursor at window-relative,
    /// 1-based (`col`, `row`).  The position is **not** validated – it may lie outside
    /// the window; see [`Self::txt_validate`].
    pub fn txt_set_cursor(&mut self, col: u8, row: u8) {
        let (c, r) = self.txt_to_absolute(col, row);
        self.txt_move_cursor_to(c, r);
    }

    /// ROM `1177`: remove the cursor, store a new absolute position, redraw.
    fn txt_move_cursor_to(&mut self, col: u8, row: u8) {
        self.txt_undraw_cursor();
        self.txt_store_and_draw(col, row);
    }

    /// ROM `117A`: store a new absolute position and redraw the cursor.
    fn txt_store_and_draw(&mut self, col: u8, row: u8) {
        self.txt_set_cursor_abs(col, row);
        self.txt_draw_cursor();
    }

    /// `TXT GET CURSOR` (`BB78`, ROM `1180`): `(column, row, scroll_count)` – window
    /// relative, 1-based; `scroll_count` is the net number of scrolls since it was reset
    /// (+1 per scroll up, -1 per scroll down).
    pub fn txt_get_cursor(&self) -> (u8, u8, u8) {
        let (c, r) = self.txt_cursor_abs();
        let (c, r) = self.txt_to_relative(c, r);
        (c, r, self.rd8(TXT_SCROLL_COUNT))
    }

    /// ROM `118A`: window-relative 1-based → absolute 0-based.
    fn txt_to_absolute(&self, col: u8, row: u8) -> (u8, u8) {
        (
            col.wrapping_sub(1).wrapping_add(self.rd8(TXT_WIN_LEFT)),
            row.wrapping_sub(1).wrapping_add(self.rd8(TXT_WIN_TOP)),
        )
    }

    /// ROM `1197`: absolute 0-based → window-relative 1-based.
    fn txt_to_relative(&self, col: u8, row: u8) -> (u8, u8) {
        (
            col.wrapping_sub(self.rd8(TXT_WIN_LEFT)).wrapping_add(1),
            row.wrapping_sub(self.rd8(TXT_WIN_TOP)).wrapping_add(1),
        )
    }

    /// ROM `11DA`: bring an absolute cursor position back into the window.
    ///
    /// * past the right edge → first column of the next row;
    /// * before the left edge → last column of the previous row;
    /// * above the top / below the bottom → clamp to that row and ask for a scroll.
    fn wrap_position(&self, mut col: u8, mut row: u8) -> (u8, u8, Wrap) {
        let (left, top) = (self.rd8(TXT_WIN_LEFT), self.rd8(TXT_WIN_TOP));
        let (right, bottom) = (self.rd8(TXT_WIN_RIGHT), self.rd8(TXT_WIN_BOTTOM));
        if !sign_pos(right, col) {
            col = left;
            row = row.wrapping_add(1);
        }
        if sign_pos(left.wrapping_sub(1), col) {
            col = right;
            row = row.wrapping_sub(1);
        }
        if sign_pos(top.wrapping_sub(1), row) {
            return (col, top, Wrap::Scroll { up: false });
        }
        if sign_pos(bottom, row) {
            return (col, row, Wrap::Inside);
        }
        (col, bottom, Wrap::Scroll { up: true })
    }

    /// ROM `11A8`: remove the cursor, then [`Self::txt_fix_cursor`].
    fn txt_undraw_and_fix(&mut self) -> (u8, u8) {
        self.txt_undraw_cursor();
        self.txt_fix_cursor()
    }

    /// ROM `11AB`: apply the deferred wrap/scroll to the stored cursor position and
    /// return it.  A scroll uses the software roll for partial windows and the hardware
    /// roll (CRTC start address) for a full-screen window, filling the new line with the
    /// paper ink, and adjusts the scroll counter.
    pub fn txt_fix_cursor(&mut self) -> (u8, u8) {
        let (col, row) = self.txt_cursor_abs();
        let (col, row, wrap) = self.wrap_position(col, row);
        self.txt_set_cursor_abs(col, row);
        if let Wrap::Scroll { up } = wrap {
            let delta: u8 = if up { 0xFF } else { 0x01 };
            let count = self.rd8(TXT_SCROLL_COUNT).wrapping_add(delta);
            self.wr8(TXT_SCROLL_COUNT, count);
            let (tl, br, partial) = self.txt_get_window_cells();
            let paper = self.rd8(TXT_PAPER);
            let dir = up as u8;
            if partial {
                self.scr_sw_roll(dir, paper, tl, br);
            } else {
                self.scr_hw_roll(dir, paper);
            }
        }
        (col, row)
    }

    /// Like [`Self::txt_validate`], but also reports what the ROM leaves in register `B`
    /// when the position is *outside* the window: `0x00` if it was above the top,
    /// `0xFF` if it was below the bottom (the direction a scroll would take).  `None`
    /// when inside (`B` untouched).  The line editor relies on this by accident.
    pub fn txt_validate_b(&self, col: u8, row: u8) -> Option<u8> {
        let (c, r) = self.txt_to_absolute(col, row);
        match self.wrap_position(c, r).2 {
            Wrap::Inside => None,
            Wrap::Scroll { up } => Some(if up { 0xFF } else { 0x00 }),
        }
    }

    /// `TXT VALIDATE` (`BB81`, ROM `11CE`): check a window-relative 1-based position.
    /// Returns the position after wrapping (still relative) and `true` if it was inside
    /// the window without needing to scroll.
    pub fn txt_validate(&self, col: u8, row: u8) -> (u8, u8, bool) {
        let (c, r) = self.txt_to_absolute(col, row);
        let (c, r, wrap) = self.wrap_position(c, r);
        let (c, r) = self.txt_to_relative(c, r);
        (c, r, wrap == Wrap::Inside)
    }

    // ===============================================================================
    // Windows                                                          (120C .. 1262)
    // ===============================================================================

    /// `TXT WIN ENABLE` (`BB66`, ROM `120C`): define the window of the selected stream.
    /// Columns `col1`/`col2` and rows `row1`/`row2` may be given in either order and
    /// are clipped to the screen (negative values count as 0).  The cursor goes to the
    /// top-left corner.
    pub fn txt_win_enable(&mut self, col1: u8, col2: u8, row1: u8, row2: u8) {
        let (max_col, max_row) = self.scr_char_limits();
        let (mut left, mut right) = (clip(col1, max_col), clip(col2, max_col));
        if right < left { std::mem::swap(&mut left, &mut right); }
        let (mut top, mut bottom) = (clip(row1, max_row), clip(row2, max_row));
        if bottom < top { std::mem::swap(&mut top, &mut bottom); }
        self.wr8(TXT_WIN_TOP, top);
        self.wr8(TXT_WIN_LEFT, left);
        self.wr8(TXT_WIN_BOTTOM, bottom);
        self.wr8(TXT_WIN_RIGHT, right);
        // B287 is non-zero when the window does not cover the whole screen.
        let flag = if left | top != 0 {
            left | top
        } else if right ^ max_col != 0 {
            right ^ max_col
        } else {
            bottom ^ max_row
        };
        self.wr8(TXT_WINDOW_FLAG, flag);
        self.txt_move_cursor_to(left, top);
    }

    /// `TXT GET WINDOW` (`BB69`, ROM `1256`): `((left, top), (right, bottom), partial)`
    /// where `partial` is `true` if the window is smaller than the screen.
    pub fn txt_get_window(&self) -> ((u8, u8), (u8, u8), bool) {
        self.txt_get_window_cells()
    }

    fn txt_get_window_cells(&self) -> ((u8, u8), (u8, u8), bool) {
        (
            (self.rd8(TXT_WIN_LEFT), self.rd8(TXT_WIN_TOP)),
            (self.rd8(TXT_WIN_RIGHT), self.rd8(TXT_WIN_BOTTOM)),
            self.rd8(TXT_WINDOW_FLAG) != 0,
        )
    }

    // ===============================================================================
    // Cursor display                                                   (1263 .. 12A8)
    // ===============================================================================

    /// `TXT PLACE CURSOR` / `TXT REMOVE CURSOR` (`BB8A`/`BB8D`, ROM `1268`): the cursor is
    /// drawn by inverting pen and paper in its character cell, so the same routine both
    /// places and removes it.
    pub fn txt_place_cursor(&mut self) {
        let (col, row) = self.txt_fix_cursor();
        let (pen, paper) = (self.rd8(TXT_PEN), self.rd8(TXT_PAPER));
        self.scr_char_invert(paper, pen, col, row);
    }

    /// `TXT CUR ON` (`BB81`, ROM `1279`): show the cursor (clears the "off" flag).
    pub fn txt_cur_on(&mut self) { self.txt_cursor_flags_and(0xFD); }

    /// `TXT CUR OFF` (`BB84`, ROM `1281`): hide the cursor (sets the "off" flag).
    pub fn txt_cur_off(&mut self) { self.txt_cursor_flags_or(0x02); }

    /// `TXT CUR ENABLE` (`BB7B`, ROM `1289`): allow the cursor (used by the user).
    pub fn txt_cur_enable(&mut self) { self.txt_cursor_flags_and(0xFE); }

    /// `TXT CUR DISABLE` (`BB7E`, ROM `129A`): forbid the cursor (used by the system).
    pub fn txt_cur_disable(&mut self) { self.txt_cursor_flags_or(0x01); }

    /// ROM `128B`: clear flag bits, redrawing the cursor if it is now visible.
    fn txt_cursor_flags_and(&mut self, mask: u8) {
        self.txt_undraw_cursor();
        let v = self.rd8(TXT_CURSOR_FLAGS) & mask;
        self.wr8(TXT_CURSOR_FLAGS, v);
        self.txt_draw_cursor();
    }

    /// ROM `129C`: set flag bits after removing the cursor (it stays hidden).
    fn txt_cursor_flags_or(&mut self, bits: u8) {
        self.txt_undraw_cursor();
        let v = self.rd8(TXT_CURSOR_FLAGS) | bits;
        self.wr8(TXT_CURSOR_FLAGS, v);
    }

    // ===============================================================================
    // Pen, paper                                                       (12A9 .. 12D2)
    // ===============================================================================

    /// `TXT SET PEN` (`BB90`, ROM `12A9`): `pen` is a pen number (0..15, masked by the
    /// mode); it is stored *encoded* for fast character drawing.
    pub fn txt_set_pen(&mut self, pen: u8) { self.txt_set_ink_slot(TXT_PEN, pen); }

    /// `TXT SET PAPER` (`BB96`, ROM `12AE`).
    pub fn txt_set_paper(&mut self, paper: u8) { self.txt_set_ink_slot(TXT_PAPER, paper); }

    /// ROM `12B1`: store an encoded ink without leaving a stale cursor on the screen.
    fn txt_set_ink_slot(&mut self, slot: u16, pen: u8) {
        self.txt_undraw_cursor();
        let enc = self.scr_ink_encode(pen);
        self.wr8(slot, enc);
        self.txt_draw_cursor();
    }

    /// `TXT GET PEN` (`BB93`, ROM `12BD`).
    pub fn txt_get_pen(&self) -> u8 { self.scr_ink_decode(self.rd8(TXT_PEN)) }

    /// `TXT GET PAPER` (`BB99`, ROM `12C3`).
    pub fn txt_get_paper(&self) -> u8 { self.scr_ink_decode(self.rd8(TXT_PAPER)) }

    /// `TXT INVERSE` (`BB9C`, ROM `12C9`): exchange pen and paper.
    pub fn txt_inverse(&mut self) {
        let (pen, paper) = (self.rd8(TXT_PEN), self.rd8(TXT_PAPER));
        self.wr8(TXT_PEN, paper);
        self.wr8(TXT_PAPER, pen);
    }

    // ===============================================================================
    // Character matrices                                               (12D3 .. 1333)
    // ===============================================================================

    /// ROM `132A`: `(first_char, user_table_present, table_address)`.
    fn txt_matrix_table(&self) -> (u8, bool, u16) {
        (
            self.rd8(TXT_MATRIX_FIRST),
            self.rd8(TXT_MATRIX_FLAG) & 1 != 0,
            self.rd16(TXT_MATRIX_TABLE),
        )
    }

    /// `TXT GET MATRIX` (`BBA5`, ROM `12D3`): where the 8-byte matrix of `ch` lives.
    /// Characters at or above the user table's first character come from that table
    /// (`user_defined == true`); all others come from the ROM font at `3800`.
    pub fn txt_get_matrix(&self, ch: u8) -> MatrixRef {
        let (first, present, table) = self.txt_matrix_table();
        if present && ch >= first {
            MatrixRef { addr: table.wrapping_add((ch - first) as u16 * 8), user_defined: true }
        } else {
            MatrixRef { addr: FONT_BASE + ch as u16 * 8, user_defined: false }
        }
    }

    /// The eight bytes of a matrix: from the ROM image for the built-in font, otherwise
    /// from the CPU's view of memory.
    pub fn txt_matrix_bytes(&self, m: MatrixRef) -> [u8; 8] {
        let mut out = [0u8; 8];
        for (i, o) in out.iter_mut().enumerate() {
            *o = if m.user_defined {
                self.read(m.addr.wrapping_add(i as u16))
            } else {
                self.rom[(m.addr as usize + i) & 0x3FFF]
            };
        }
        out
    }

    /// `TXT SET MATRIX` (`BBA8`, ROM `12F1`): define the matrix of `ch`.  Only characters
    /// covered by the user table can be changed; returns `true` if it was stored.
    pub fn txt_set_matrix(&mut self, ch: u8, matrix: &[u8; 8]) -> bool {
        let m = self.txt_get_matrix(ch);
        if !m.user_defined { return false; }
        for (i, &b) in matrix.iter().enumerate() {
            self.wr8(m.addr.wrapping_add(i as u16), b);
        }
        true
    }

    /// `TXT SET M TABLE` (`BBAB`, ROM `12FD`): install a user matrix table at `table`
    /// covering characters `first..=255`.  `d_flag` is the high byte of `DE` on entry:
    /// zero = install the table (the current matrices of every character from `first` up
    /// are first copied into it, unless the table is already in use); non-zero = remove
    /// the user table.
    pub fn txt_set_m_table(&mut self, d_flag: u8, first: u8, table: u16) {
        let mut flag = 0u8;
        if d_flag == 0 {
            flag = 0xFF;
            let mut dest = table;
            let mut c = first;
            loop {
                let src = self.txt_get_matrix(c);
                if src.addr == dest { break; }                      // already in this table
                let bytes = self.txt_matrix_bytes(src);
                for (i, &b) in bytes.iter().enumerate() {
                    self.wr8(dest.wrapping_add(i as u16), b);
                }
                dest = dest.wrapping_add(8);
                c = c.wrapping_add(1);
                if c == 0 { break; }
            }
        }
        self.wr8(TXT_MATRIX_FIRST, first);
        self.wr8(TXT_MATRIX_FLAG, flag);
        self.wr16(TXT_MATRIX_TABLE, table);
    }

    // ===============================================================================
    // Writing and reading characters                                   (1334 .. 13FF)
    // ===============================================================================

    /// `TXT WR CHAR` (`BB5D`, ROM `1334`): print `ch` at the cursor (control codes are
    /// *not* interpreted) and advance the cursor.  Does nothing while the VDU is
    /// disabled.
    pub fn txt_wr_char(&mut self, ch: u8) {
        if self.rd8(TXT_VDU_ENABLED) == 0 { return; }
        let (col, row) = self.txt_undraw_and_fix();
        self.txt_set_cursor_abs(col.wrapping_add(1), row);         // advance first ...
        self.txt_write_char_indirect(col, row, ch);                 // ... then draw (BDD3)
        self.txt_draw_cursor();
    }

    /// `BDD3` – write a character glyph at (`col`,`row`).
    fn txt_write_char_indirect(&mut self, col: u8, row: u8, ch: u8) {
        if let Some(a) = self.patched(Ind::TXT_WRITE_CHAR, 0x134A) {
            let mut r = Regs { hl: (col as u16) << 8 | row as u16, af: (ch as u16) << 8, ..Default::default() };
            self.txt_call_hook(a, &mut r);
        } else {
            self.txt_write_char_default(col, row, ch);
        }
    }

    /// Default `BDD3` routine (ROM `134A`): expand the matrix into screen format
    /// (`SCR UNPACK`, buffered at `B298`) and write it cell by cell, eight scan lines,
    /// through the transparent or opaque writer selected by `TXT SET BACK`.
    pub fn txt_write_char_default(&mut self, col: u8, row: u8, ch: u8) {
        let matrix = self.txt_matrix_bytes(self.txt_get_matrix(ch));
        let unpacked = self.scr_unpack(&matrix);
        for (i, &b) in unpacked.iter().enumerate() {
            self.wr8(TXT_CELL_BUFFER + i as u16, b);
        }
        let (mut line, bytes) = self.scr_char_position(col, row);
        for y in 0..8usize {
            let mut a = line;
            for x in 0..bytes as usize {
                let mask = unpacked[y * bytes as usize + x];
                self.txt_back_write(a, mask);
                a = crate::screen::scr_next_byte(a);
            }
            line = crate::screen::scr_next_line(line);
        }
    }

    /// Dispatch through `B291`: write one screen byte of a glyph.
    ///
    /// * opaque (ROM `1391`): pixels set in `mask` get the pen, the others the paper;
    /// * transparent (ROM `139F`): only pixels set in `mask` are changed.
    ///
    /// Both finish in `SCR PIXELS`, i.e. always *force* mode regardless of `SCR ACCESS`.
    fn txt_back_write(&mut self, addr: u16, mask: u8) {
        let pen = self.rd8(TXT_PEN);
        if self.rd16(TXT_BACK_VECTOR) == BACK_TRANSPARENT {
            self.scr_pixels(addr, pen, mask);
        } else {
            let paper = self.rd8(TXT_PAPER);
            let ink = (!mask & paper) | (mask & pen);
            self.scr_pixels(addr, ink, 0xFF);
        }
    }

    /// `TXT SET BACK` (`BB9F`-ish, ROM `137A`): `transparent == 0` = opaque.
    pub fn txt_set_back(&mut self, transparent: u8) {
        let v = if transparent == 0 { BACK_OPAQUE } else { BACK_TRANSPARENT };
        self.wr16(TXT_BACK_VECTOR, v);
    }

    /// `TXT GET BACK` (ROM `1387`): `true` = transparent.
    pub fn txt_get_back(&self) -> bool { self.rd16(TXT_BACK_VECTOR) != BACK_OPAQUE }

    /// `TXT SET GRAPHIC` (`BBB1`, ROM `13A7`): non-zero makes `TXT OUTPUT` draw every
    /// character at the *graphics* cursor.
    pub fn txt_set_graphic(&mut self, flag: u8) { self.wr8(TXT_GRAPHICS_MODE, flag); }

    /// `TXT RD CHAR` (`BB60`, ROM `13AB`): identify the character at the cursor by
    /// comparing the screen with every matrix.  `None` if no matrix matches.
    pub fn txt_rd_char(&mut self) -> Option<u8> {
        self.txt_undraw_cursor();
        let (col, row) = self.txt_cursor_abs();
        let r = self.txt_unwrite_indirect(col, row);
        self.txt_draw_cursor();
        r
    }

    /// `BDD6` – read the character at (`col`,`row`).
    fn txt_unwrite_indirect(&mut self, col: u8, row: u8) -> Option<u8> {
        if let Some(a) = self.patched(Ind::TXT_UNWRITE, 0x13C0) {
            let mut r = Regs { hl: (col as u16) << 8 | row as u16, ..Default::default() };
            self.txt_call_hook(a, &mut r);
            return if r.carry() { Some(r.a()) } else { None };
        }
        self.txt_unwrite_default(col, row)
    }

    /// Default `BDD6` routine (ROM `13C0`): read the cell back as a bit matrix (1 = pixel
    /// is the pen) and look it up; if that fails (or only gives "space") try again with
    /// the paper as the reference colour and the bits complemented, to recognise
    /// inverse-video text.
    pub fn txt_unwrite_default(&mut self, col: u8, row: u8) -> Option<u8> {
        let pen = self.rd8(TXT_PEN);
        let m = self.scr_repack(pen, col, row);
        self.txt_store_cell_buffer(&m);
        if let Some(c) = self.txt_find_matrix() {
            if c != 0x20 { return Some(c); }
        }
        let paper = self.rd8(TXT_PAPER);
        let mut m = self.scr_repack(paper, col, row);
        for b in m.iter_mut() { *b = !*b; }
        self.txt_store_cell_buffer(&m);
        self.txt_find_matrix()
    }

    fn txt_store_cell_buffer(&mut self, m: &[u8; 8]) {
        for (i, &b) in m.iter().enumerate() { self.wr8(TXT_CELL_BUFFER + i as u16, b); }
    }

    /// ROM `13E3`: search characters 0..=255 for a matrix equal to the cell buffer.
    fn txt_find_matrix(&self) -> Option<u8> {
        let mut target = [0u8; 8];
        for (i, t) in target.iter_mut().enumerate() { *t = self.rd8(TXT_CELL_BUFFER + i as u16); }
        (0..=255u8).find(|&c| self.txt_matrix_bytes(self.txt_get_matrix(c)) == target)
    }

    // ===============================================================================
    // Output and control codes                                         (1400 .. 1459)
    // ===============================================================================

    /// `TXT OUTPUT` (`BB5A`, ROM `1400`): print `ch`, interpreting control codes.
    pub fn txt_output(&mut self, ch: u8) {
        if let Some(a) = self.patched(Ind::TXT_OUT_ACTION, 0x140C) {
            let mut r = Regs { af: (ch as u16) << 8, ..Default::default() };
            self.txt_call_hook(a, &mut r);
        } else {
            self.txt_out_action_default(ch);
        }
    }

    /// Default `BDD9` routine (ROM `140C`).
    ///
    /// A control code (`< 0x20`) starts collecting a sequence in `B2B9`; bytes keep being
    /// collected (even printable ones) until the number of parameters listed for the
    /// code in the control table has arrived, then the handler runs and the collector
    /// is reset.  In *graphics* mode everything goes to `GRA WR CHAR` instead.
    pub fn txt_out_action_default(&mut self, ch: u8) {
        if self.rd8(TXT_GRAPHICS_MODE) != 0 {
            self.gra_wr_char(ch);
            return;
        }
        let mut n = self.rd8(TXT_CTRL_COUNT);
        if n < 10 {
            if n == 0 && ch >= 0x20 {
                self.txt_wr_char(ch);                         // the common case: plain text
                return;
            }
            n += 1;
            self.wr8(TXT_CTRL_COUNT, n);
            self.wr8(TXT_CTRL_COUNT + n as u16, ch);          // B2B9 = code, then parameters
            let code = self.rd8(TXT_CTRL_BUFFER);
            let entry = TXT_CTRL_TABLE.wrapping_add(code as u16 * 3);
            let wanted = self.rd8(entry);
            if wanted >= n { return; }                        // still waiting for parameters
            let handler = self.rd16(entry + 1);
            self.txt_run_control(handler, ch);
        }
        self.wr8(TXT_CTRL_COUNT, 0);
    }

    /// Run a control-code handler.  `a` is the last byte received; the full sequence is
    /// in the buffer at `B2B9` (`HL` on the Z80).  Handlers installed by a program are
    /// called as machine code.
    fn txt_run_control(&mut self, handler: u16, a: u8) {
        let p = |s: &Self, i: u16| s.rd8(TXT_CTRL_BUFFER + i);
        match handler {
            H_NOP => {}
            H_WR_CHAR => self.txt_wr_char(a),
            H_CUR_DISABLE => self.txt_cur_disable(),
            H_CUR_ENABLE => self.txt_cur_enable(),
            H_SET_MODE => self.scr_set_mode(a),
            H_GRA_WR_CHAR => self.gra_wr_char(a),
            H_VDU_ENABLE => self.txt_vdu_enable(),
            H_BEL => { self.sound_queue(&BEL_SOUND); }
            H_LEFT => self.txt_step_cursor(0xFF, 0x00),
            H_RIGHT => self.txt_step_cursor(0x01, 0x00),
            H_DOWN => self.txt_step_cursor(0x00, 0x01),
            H_UP => self.txt_step_cursor(0x00, 0xFF),
            H_CLEAR_WINDOW => self.txt_clear_window(),
            H_CR => {
                let (_, row) = self.txt_undraw_and_fix();
                let left = self.rd8(TXT_WIN_LEFT);
                self.txt_store_and_draw(left, row);
            }
            H_SET_PAPER => self.txt_set_paper(a),
            H_SET_PEN => self.txt_set_pen(a),
            H_CLEAR_CHAR => {
                let (c, r) = self.txt_undraw_and_fix();
                self.txt_fill_paper((c, r), (c, r));
            }
            H_CLEAR_LINE_START => self.txt_clear_line_start(),
            H_CLEAR_LINE_END => self.txt_clear_line_end(),
            H_CLEAR_SCREEN_START => self.txt_clear_window_start(),
            H_CLEAR_SCREEN_END => self.txt_clear_window_end(),
            H_VDU_DISABLE => self.txt_vdu_disable(),
            H_TRANSPARENT => self.txt_set_back(if a & 1 != 0 { 0xFF } else { 0 }),
            H_SCR_ACCESS => self.scr_access(match a & 3 {
                0 => AccessMode::Force, 1 => AccessMode::Xor, 2 => AccessMode::And, _ => AccessMode::Or,
            }),
            H_INVERSE => self.txt_inverse(),
            H_DEFINE_MATRIX => {
                let ch = p(self, 1);
                let mut m = [0u8; 8];
                for (i, b) in m.iter_mut().enumerate() { *b = p(self, 2 + i as u16); }
                self.txt_set_matrix(ch, &m);
            }
            H_WINDOW => {
                // parameters: left, right, top, bottom
                let (c1, c2, r1, r2) = (p(self, 1), p(self, 2), p(self, 3), p(self, 4));
                self.txt_win_enable(c1, c2, r1, r2);
            }
            H_SET_INK => {
                let (pen, c1, c2) = (p(self, 1), p(self, 2), p(self, 3));
                self.scr_set_ink(pen, c1, c2);
            }
            H_SET_BORDER => {
                let (c1, c2) = (p(self, 1), p(self, 2));
                self.scr_set_border(c1, c2);
            }
            H_HOME => {
                let (left, top) = (self.rd8(TXT_WIN_LEFT), self.rd8(TXT_WIN_TOP));
                self.txt_move_cursor_to(left, top);
            }
            H_LOCATE => {
                let (c, r) = (p(self, 1), p(self, 2));
                self.txt_set_cursor(c, r);
            }
            other => {
                // A handler patched in by the program.
                let mut r = Regs { hl: TXT_CTRL_BUFFER, af: (a as u16) << 8, ..Default::default() };
                self.txt_call_hook(other, &mut r);
            }
        }
    }

    /// ROM `151C`: move the cursor by (`dcol`, `drow`) – each 0, 1 or 0xFF (= -1) – after
    /// settling any pending wrap.  The new position is *not* validated.
    fn txt_step_cursor(&mut self, dcol: u8, drow: u8) {
        let (c, r) = self.txt_undraw_and_fix();
        self.txt_store_and_draw(c.wrapping_add(dcol), r.wrapping_add(drow));
    }

    /// `TXT VDU DISABLE` (`BB54`, ROM `144B`).
    pub fn txt_vdu_disable(&mut self) {
        self.txt_cur_disable();
        self.wr8(TXT_VDU_ENABLED, 0);
        self.wr8(TXT_CTRL_COUNT, 0);
    }

    /// `TXT VDU ENABLE` (`BB57`, ROM `1451`).
    pub fn txt_vdu_enable(&mut self) {
        self.txt_cur_enable();
        self.wr8(TXT_VDU_ENABLED, 0xFF);
        self.wr8(TXT_CTRL_COUNT, 0);
    }

    /// `TXT GET CONTROLS` (`BB..`, ROM `14CB`): address of the control-code table
    /// (32 entries × 3 bytes: parameter count, handler address).
    pub fn txt_get_controls(&self) -> u16 { TXT_CTRL_TABLE }

    // ===============================================================================
    // Clearing                                                         (1540 .. 15A0)
    // ===============================================================================

    /// Fill a rectangle of character cells with the paper ink (ROM `1597`) and redraw
    /// the cursor.
    fn txt_fill_paper(&mut self, tl: (u8, u8), br: (u8, u8)) {
        let paper = self.rd8(TXT_PAPER);
        self.scr_fill_box(paper, tl, br);
        self.txt_draw_cursor();
    }

    /// `TXT CLEAR WINDOW` (`BB6C`, ROM `1540`): fill the window with paper, cursor home.
    pub fn txt_clear_window(&mut self) {
        self.txt_undraw_cursor();
        let (left, top) = (self.rd8(TXT_WIN_LEFT), self.rd8(TXT_WIN_TOP));
        self.txt_set_cursor_abs(left, top);
        let br = (self.rd8(TXT_WIN_RIGHT), self.rd8(TXT_WIN_BOTTOM));
        self.txt_fill_paper((left, top), br);
    }

    /// ROM `158E`: clear from the left edge of the window to the cursor (inclusive).
    fn txt_clear_line_start(&mut self) {
        let (c, r) = self.txt_undraw_and_fix();
        let left = self.rd8(TXT_WIN_LEFT);
        self.txt_fill_paper((left, r), (c, r));
    }

    /// ROM `1584`: clear from the cursor to the right edge of the window.
    fn txt_clear_line_end(&mut self) {
        let (c, r) = self.txt_undraw_and_fix();
        let right = self.rd8(TXT_WIN_RIGHT);
        self.txt_fill_paper((c, r), (right, r));
    }

    /// ROM `156D`: clear from the top of the window to the cursor.
    fn txt_clear_window_start(&mut self) {
        self.txt_clear_line_start();
        let (left, top) = (self.rd8(TXT_WIN_LEFT), self.rd8(TXT_WIN_TOP));
        let right = self.rd8(TXT_WIN_RIGHT);
        let row = self.rd8(TXT_CURSOR_ROW);
        let last = row.wrapping_sub(1);
        // CP L / CALL NC (unsigned).  Note the ROM quirk: with the cursor on row 0 of a
        // window starting at row 0, `row - 1` wraps to 255 and the test still passes.
        if last >= top {
            let paper = self.rd8(TXT_PAPER);
            self.scr_fill_box(paper, (left, top), (right, last));
        }
    }

    /// ROM `1556`: clear from the cursor to the bottom of the window.
    fn txt_clear_window_end(&mut self) {
        self.txt_clear_line_end();
        let (left, bottom) = (self.rd8(TXT_WIN_LEFT), self.rd8(TXT_WIN_BOTTOM));
        let right = self.rd8(TXT_WIN_RIGHT);
        let row = self.rd8(TXT_CURSOR_ROW);
        if row < bottom {                                   // CP E / CALL C: rows below the cursor
            let paper = self.rd8(TXT_PAPER);
            self.scr_fill_box(paper, (left, row + 1), (right, bottom));
        }
    }
}
