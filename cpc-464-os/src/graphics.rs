//! # Graphics VDU  (ROM `15B0 .. 19DF`)
//!
//! Plotting, line drawing, graphics-cursor character output and the graphics window.
//!
//! ## Coordinate systems
//!
//! * **User coordinates** – signed 16-bit, origin at the user origin (`GRA SET ORIGIN`),
//!   *x* to the right (0..639 over the screen) and *y* upwards (0..399).  These are what
//!   BASIC and programs pass in, and what the graphics *cursor* is stored in.
//! * **Screen (pixel) coordinates** – what [`Cpc::scr_dot_position`] expects.  The
//!   conversion (ROM `161D`) truncates x towards zero to a pixel boundary, adds the
//!   origin and divides by 4 / 2 / 1 for mode 0 / 1 / 2; y is truncated to an even number,
//!   offset by the origin and halved (200 lines).  Bottom-left is (0,0).
//!
//! The graphics window is stored in screen coordinates: `left`/`right` rounded to whole
//! screen bytes, `bottom`/`top` in lines.
//!
//! ## Lines
//!
//! [`Cpc::gra_line_absolute`] uses a run-length formulation of Bresenham: the line is split
//! into `minor + 1` runs along the major axis; each run has `(major + 1) / (minor + 1)`
//! pixels, and the division remainder is spread over the runs with an error accumulator so
//! that some runs are one pixel longer.  Each run is drawn as one horizontal or vertical
//! segment with [`Cpc::scr_horizontal`] / [`Cpc::scr_vertical`].
//!
//! ## Indirections
//!
//! `GRA PLOT`, `GRA TEST` and `GRA LINE` can be replaced by a program (`BDDC`, `BDDF`,
//! `BDE2`); the *relative* variants convert to absolute and go through the same vectors.

use crate::hw::{Hardware, Regs};
use crate::maths::int_div_unsigned;
use crate::screen::{scr_next_byte, scr_next_line};
use crate::sysvars::*;
use crate::Cpc;

/// Z80 idiom: the sign flag of `a - b` (16-bit, wrapping) – `true` means negative (`M`).
#[inline]
fn minus(a: i16, b: i16) -> bool { a.wrapping_sub(b) < 0 }

impl<H: Hardware> Cpc<H> {
    // ---- 16-bit helpers over RAM -------------------------------------------------

    #[inline] fn g16(&self, addr: u16) -> i16 { self.rd16(addr) as i16 }
    #[inline] fn s16(&mut self, addr: u16, v: i16) { self.wr16(addr, v as u16); }

    // ===============================================================================
    // Initialisation                                                   (15B0 .. 15F0)
    // ===============================================================================

    /// `GRA INITIALISE` (`BBBA`, ROM `15B0`): reset the indirections, paper 0, pen 1,
    /// origin (0,0), full-screen window.
    pub fn gra_initialise(&mut self) {
        self.gra_reset();
        self.gra_restore_pen_paper(0x0001);
    }

    /// ROM `15B6`: set paper (`H`), pen (`L`) of `paper_pen`, origin to (0,0) and the
    /// window to the entire screen.  Also used after a mode change.
    pub(crate) fn gra_restore_pen_paper(&mut self, paper_pen: u16) {
        self.gra_set_paper((paper_pen >> 8) as u8);
        self.gra_set_pen(paper_pen as u8);
        self.gra_set_origin(0, 0);
        self.gra_win_width(i16::MIN, i16::MAX);
        self.gra_win_height(i16::MAX, i16::MIN);
    }

    /// ROM `15D6`: the current paper (high byte) and pen (low byte) as pen *numbers*.
    pub(crate) fn gra_save_pen_paper(&mut self) -> u16 {
        (self.gra_get_paper() as u16) << 8 | self.gra_get_pen() as u16
    }

    /// `GRA RESET` (`BBBD`, ROM `15DF`): restore the default `GRA PLOT/TEST/LINE` vectors.
    pub fn gra_reset(&mut self) {
        for (addr, target) in [(Ind::GRA_PLOT, 0x1816u16), (Ind::GRA_TEST, 0x182A), (Ind::GRA_LINE, 0x183C)] {
            self.wr8(addr, 0xC3);
            self.wr16(addr + 1, target);
        }
    }

    // ===============================================================================
    // Cursor and origin                                                (15F1 .. 1656)
    // ===============================================================================

    /// `GRA MOVE RELATIVE` (`BBC0`, ROM `15F1`).
    pub fn gra_move_relative(&mut self, dx: i16, dy: i16) {
        let (x, y) = self.gra_relative_to_absolute(dx, dy);
        self.gra_move_absolute(x, y);
    }

    /// `GRA MOVE ABSOLUTE` (`BBC3`, ROM `15F4`): set the graphics cursor (user coordinates).
    pub fn gra_move_absolute(&mut self, x: i16, y: i16) {
        self.s16(GRA_CURSOR_X, x);
        self.s16(GRA_CURSOR_Y, y);
    }

    /// `GRA ASK CURSOR` (`BBC6`, ROM `15FC`): `(x, y)` in user coordinates.
    pub fn gra_ask_cursor(&self) -> (i16, i16) { (self.g16(GRA_CURSOR_X), self.g16(GRA_CURSOR_Y)) }

    /// `GRA SET ORIGIN` (`BBC9`, ROM `1604`): the user origin, in user units counted from
    /// the bottom-left of the screen.  The cursor is moved to (0,0).
    pub fn gra_set_origin(&mut self, x: i16, y: i16) {
        self.s16(GRA_ORIGIN_X, x);
        self.s16(GRA_ORIGIN_Y, y);
        self.gra_move_absolute(0, 0);                       // L_160B
    }

    /// `GRA GET ORIGIN` (`BBCC`, ROM `1612`).
    pub fn gra_get_origin(&self) -> (i16, i16) { (self.g16(GRA_ORIGIN_X), self.g16(GRA_ORIGIN_Y)) }

    /// ROM `1657`: add a displacement to the graphics cursor.
    fn gra_relative_to_absolute(&self, dx: i16, dy: i16) -> (i16, i16) {
        (self.g16(GRA_CURSOR_X).wrapping_add(dx), self.g16(GRA_CURSOR_Y).wrapping_add(dy))
    }

    /// ROM `161A`: convert the *current* cursor to screen coordinates.
    fn gra_cursor_to_screen(&mut self) -> (i16, i16) {
        let (x, y) = self.gra_ask_cursor();
        self.gra_user_to_screen(x, y)
    }

    /// ROM `161D`: set the cursor to (`x`,`y`) and convert it to screen coordinates.
    ///
    /// x is truncated towards zero to a pixel boundary (mask 3/1/0 for mode 0/1/2), the
    /// origin is added and the sum shifted right by 2/1/0; y is truncated to an even value,
    /// the origin added and the sum halved.
    fn gra_user_to_screen(&mut self, x: i16, y: i16) -> (i16, i16) {
        self.gra_move_absolute(x, y);
        let mode = self.scr_get_mode();
        let (mask, shift): (i16, u32) = match mode { 0 => (3, 2), 1 => (1, 1), _ => (0, 0) };
        let mut xt = x;
        if xt < 0 { xt = xt.wrapping_add(mask); }                  // round towards zero
        xt &= !mask;
        let px = self.g16(GRA_ORIGIN_X).wrapping_add(xt) >> shift;
        let mut yt = y;
        if yt < 0 { yt = yt.wrapping_add(1); }
        yt &= !1;
        let py = self.g16(GRA_ORIGIN_Y).wrapping_add(yt) >> 1;
        (px, py)
    }

    // ===============================================================================
    // Window                                                           (1660 .. 17C4)
    // ===============================================================================

    /// ROM `16FF`: is the screen point inside the graphics window?
    fn gra_point_in_window(&self, x: i16, y: i16) -> bool {
        let (left, right) = (self.g16(GRA_WIN_LEFT), self.g16(GRA_WIN_RIGHT));
        let (top, bottom) = (self.g16(GRA_WIN_TOP), self.g16(GRA_WIN_BOTTOM));
        if !minus(left.wrapping_sub(1), x) { return false; }     // x < left
        if minus(right, x) { return false; }                      // x > right
        if !minus(bottom.wrapping_sub(1), y) { return false; }    // y < bottom
        if minus(top, y) { return false; }                        // y > top
        true
    }

    /// ROM `16B0`: clip the horizontal run `x0..=x1` on line `y` to the window.
    fn gra_clip_horizontal(&self, x0: i16, y: i16, x1: i16) -> Option<(i16, i16)> {
        let (left, right) = (self.g16(GRA_WIN_LEFT), self.g16(GRA_WIN_RIGHT));
        let (top, bottom) = (self.g16(GRA_WIN_TOP), self.g16(GRA_WIN_BOTTOM));
        if !minus(bottom.wrapping_sub(1), y) { return None; }
        if minus(top, y) { return None; }
        if minus(right, x0) { return None; }
        let x0c = if minus(left.wrapping_sub(1), x0) { x0 } else { left };
        if !minus(left.wrapping_sub(1), x1) { return None; }
        let x1c = if minus(right, x1) { right } else { x1 };
        Some((x0c, x1c))
    }

    /// ROM `1664`: clip the vertical run `y0..=y1` on column `x` to the window.
    fn gra_clip_vertical(&self, x: i16, y0: i16, y1: i16) -> Option<(i16, i16)> {
        let (left, right) = (self.g16(GRA_WIN_LEFT), self.g16(GRA_WIN_RIGHT));
        let (top, bottom) = (self.g16(GRA_WIN_TOP), self.g16(GRA_WIN_BOTTOM));
        if !minus(left.wrapping_sub(1), x) { return None; }
        if minus(right, x) { return None; }
        if minus(top, y0) { return None; }
        let y0c = if minus(bottom.wrapping_sub(1), y0) { y0 } else { bottom };
        if !minus(bottom.wrapping_sub(1), y1) { return None; }
        let y1c = if minus(top, y1) { top } else { y1 };
        Some((y0c, y1c))
    }

    /// ROM `1760`: clip a user x coordinate to `0..=639`.
    fn gra_clip_x(v: i16) -> i16 { if v < 0 { 0 } else { v.min(639) } }

    /// `GRA WIN WIDTH` (`BBCF`, ROM `1734`): set the left/right window edges (user x).
    /// They may be given in either order; the left edge is rounded down and the right
    /// edge up to a whole screen byte, and both are stored in pixels of the current mode.
    pub fn gra_win_width(&mut self, x1: i16, x2: i16) {
        let (a, b) = (Self::gra_clip_x(x1), Self::gra_clip_x(x2));
        let (mut left, mut right) = if a < b { (a, b) } else { (b, a) };
        left &= !7;
        right |= 7;
        let shift = match self.scr_get_mode() { 0 => 2, 1 => 1, _ => 0 };
        self.s16(GRA_WIN_LEFT, left >> shift);
        self.s16(GRA_WIN_RIGHT, right >> shift);
    }

    /// `GRA WIN HEIGHT` (`BBD2`, ROM `1779`): set the top/bottom window edges (user y,
    /// 0..399, either order); stored as lines 0..199.
    pub fn gra_win_height(&mut self, y1: i16, y2: i16) {
        let f = |v: i16| if v < 0 { 0 } else { (v >> 1).min(199) };
        let (a, b) = (f(y1), f(y2));
        let (top, bottom) = if b < a { (a, b) } else { (b, a) };
        self.s16(GRA_WIN_TOP, top);
        self.s16(GRA_WIN_BOTTOM, bottom);
    }

    /// `GRA GET W WIDTH` (`BBD5`, ROM `17A6`): `(left, right)` in user x units.
    pub fn gra_get_w_width(&self) -> (i16, i16) {
        let (mut de, mut hl) = (self.g16(GRA_WIN_LEFT), self.g16(GRA_WIN_RIGHT));
        let times = match self.scr_get_mode() { 0 => 2, 1 => 1, _ => 0 };
        for _ in 0..times {                                          // L_17B6
            hl = hl.wrapping_mul(2).wrapping_add(1);
            de = de.wrapping_mul(2);
        }
        (de, hl)
    }

    /// `GRA GET W HEIGHT` (`BBD8`, ROM `17BC`): `(top, bottom)` in user y units.
    ///
    /// ROM quirk (kept): the shared helper doubles the top and doubles-plus-one the
    /// bottom, so `top` comes back even and `bottom` odd – the reverse of what
    /// [`Self::gra_get_w_width`] does for `left`/`right`.
    pub fn gra_get_w_height(&self) -> (i16, i16) {
        let (de, hl) = (self.g16(GRA_WIN_TOP), self.g16(GRA_WIN_BOTTOM));
        (de.wrapping_mul(2), hl.wrapping_mul(2).wrapping_add(1))
    }

    /// `GRA CLEAR WINDOW` (`BBDB`, ROM `17C5`): fill the graphics window with the
    /// graphics paper and move the cursor to the origin.
    pub fn gra_clear_window(&mut self) {
        let (left, right) = self.gra_get_w_width();
        let width_units = right.wrapping_sub(left).wrapping_add(1);
        let bytes = (((width_units >> 2) as u8) >> 1) as u8;          // user units -> bytes
        let top = self.g16(GRA_WIN_TOP);
        let bottom = self.g16(GRA_WIN_BOTTOM);
        let lines = top.wrapping_sub(bottom).wrapping_add(1) as u8;
        let left_px = self.g16(GRA_WIN_LEFT);
        let dot = self.scr_dot_position(left_px as u16, top as u16);
        let paper = self.rd8(GRA_PAPER);
        self.scr_flood_box(dot.addr, bytes, lines, paper);
        self.gra_move_absolute(0, 0);
    }

    // ===============================================================================
    // Pen and paper                                                    (17F6 .. 180D)
    // ===============================================================================

    /// `GRA SET PEN` (`BBDE`, ROM `17F6`): pen number, stored encoded for the mode.
    pub fn gra_set_pen(&mut self, pen: u8) { let e = self.scr_ink_encode(pen); self.wr8(GRA_PEN, e); }
    /// `GRA SET PAPER` (`BBE1`, ROM `17FD`).
    pub fn gra_set_paper(&mut self, paper: u8) { let e = self.scr_ink_encode(paper); self.wr8(GRA_PAPER, e); }
    /// `GRA GET PEN` (`BBE4`, ROM `1804`).
    pub fn gra_get_pen(&self) -> u8 { self.scr_ink_decode(self.rd8(GRA_PEN)) }
    /// `GRA GET PAPER` (`BBE7`, ROM `180A`).
    pub fn gra_get_paper(&self) -> u8 { self.scr_ink_decode(self.rd8(GRA_PAPER)) }

    // ===============================================================================
    // Plot and test                                                    (1810 .. 182F)
    // ===============================================================================

    /// `GRA PLOT RELATIVE` (`BBED`, ROM `1810`).
    pub fn gra_plot_relative(&mut self, dx: i16, dy: i16) {
        let (x, y) = self.gra_relative_to_absolute(dx, dy);
        self.gra_plot_absolute(x, y);
    }

    /// `GRA PLOT ABSOLUTE` (`BBEA`, ROM `1813`): jumps through the `BDDC` indirection.
    pub fn gra_plot_absolute(&mut self, x: i16, y: i16) {
        if let Some(a) = self.patched(Ind::GRA_PLOT, 0x1816) {
            let mut r = Regs { de: x as u16, hl: y as u16, ..Default::default() };
            self.hw.call(a, &mut r);
        } else {
            self.gra_plot_default(x, y);
        }
    }

    /// Default plot (ROM `1816`): move the cursor, and if the point is inside the window
    /// write the pen through `SCR WRITE` (so the current `SCR ACCESS` mode applies).
    pub fn gra_plot_default(&mut self, x: i16, y: i16) {
        let (px, py) = self.gra_user_to_screen(x, y);
        if !self.gra_point_in_window(px, py) { return; }
        let dot = self.scr_dot_position(px as u16, py as u16);
        let pen = self.rd8(GRA_PEN);
        self.scr_write(dot.addr, pen, dot.mask);
    }

    /// `GRA TEST RELATIVE` (`BBF0`, ROM `1824`).
    pub fn gra_test_relative(&mut self, dx: i16, dy: i16) -> u8 {
        let (x, y) = self.gra_relative_to_absolute(dx, dy);
        self.gra_test_absolute(x, y)
    }

    /// `GRA TEST ABSOLUTE` (`BBF3`, ROM `1827`): the pen number at the point.
    pub fn gra_test_absolute(&mut self, x: i16, y: i16) -> u8 {
        if let Some(a) = self.patched(Ind::GRA_TEST, 0x182A) {
            let mut r = Regs { de: x as u16, hl: y as u16, ..Default::default() };
            self.hw.call(a, &mut r);
            return r.a();
        }
        self.gra_test_default(x, y)
    }

    /// Default test (ROM `182A`): outside the window the answer is the graphics paper.
    pub fn gra_test_default(&mut self, x: i16, y: i16) -> u8 {
        let (px, py) = self.gra_user_to_screen(x, y);
        if !self.gra_point_in_window(px, py) { return self.gra_get_paper(); }
        let dot = self.scr_dot_position(px as u16, py as u16);
        self.scr_read(dot.addr, dot.mask)
    }

    // ===============================================================================
    // Lines                                                            (1836 .. 1944)
    // ===============================================================================

    /// `GRA LINE RELATIVE` (`BBF6`, ROM `1836`).
    pub fn gra_line_relative(&mut self, dx: i16, dy: i16) {
        let (x, y) = self.gra_relative_to_absolute(dx, dy);
        self.gra_line_absolute(x, y);
    }

    /// `GRA LINE ABSOLUTE` (`BBF9`, ROM `1839`): draw from the cursor to (`x`,`y`) through
    /// the `BDE2` indirection.
    pub fn gra_line_absolute(&mut self, x: i16, y: i16) {
        if let Some(a) = self.patched(Ind::GRA_LINE, 0x183C) {
            let mut r = Regs { de: x as u16, hl: y as u16, ..Default::default() };
            self.hw.call(a, &mut r);
        } else {
            self.gra_line_default(x, y);
        }
    }

    /// Default line drawer (ROM `183C`).  See the module documentation for the algorithm.
    ///
    /// Working state is kept in the same RAM cells as the ROM (`B33A..B346`).
    pub fn gra_line_default(&mut self, x: i16, y: i16) {
        let (x1, y1) = self.gra_cursor_to_screen();                 // start (also re-stores cursor)
        self.s16(GRA_LINE_X, x1);
        self.s16(GRA_LINE_Y, y1);
        let (x2, y2) = self.gra_user_to_screen(x, y);               // end; cursor := end

        // Normalise so the line runs left to right: start = leftmost point.
        let mut dx = x1.wrapping_sub(x2);
        let end_y;
        if dx >= 0 {
            self.s16(GRA_LINE_X, x2);
            self.s16(GRA_LINE_Y, y2);
            end_y = y1;
        } else {
            dx = 0i16.wrapping_sub(dx);
            end_y = y2;
        }
        let (xs, ys) = (self.g16(GRA_LINE_X), self.g16(GRA_LINE_Y));
        let down = ys.wrapping_sub(end_y);                          // positive: line goes down

        // dy, y step, and whether the line is steep (dy >= dx).
        let (dy, step, steep): (i16, i16, bool);
        if down >= 0 {
            dy = down;
            step = -1;
            steep = (dy as u16) >= (dx as u16);
            if steep {
                // Steep lines are always drawn bottom-to-top: start at the lower end.
                self.s16(GRA_LINE_X, xs.wrapping_add(dx));
                self.s16(GRA_LINE_Y, ys.wrapping_sub(dy));
            }
        } else {
            dy = 0i16.wrapping_sub(down);
            step = 1;
            steep = (dy as u16) >= (dx as u16);
        }
        self.s16(GRA_LINE_MAJOR_STEP, step);

        // major / minor axis lengths
        let (major, minor) = if steep { (dy as u16, dx as u16) } else { (dx as u16, dy as u16) };
        self.wr8(GRA_LINE_STEEP, if steep { 0 } else { 0xFF });
        let runs = minor.wrapping_add(1);
        self.wr16(GRA_LINE_LEN, runs);
        let (quot, rem) = int_div_unsigned(major.wrapping_add(1), runs);
        self.wr16(GRA_LINE_QUOT, quot);
        self.wr16(GRA_LINE_REM, rem);

        let pen = self.rd8(GRA_PEN);
        let mut acc = runs >> 1;
        let mut left = runs;
        while left != 0 {
            let mut len = quot;
            let t = rem.wrapping_add(acc);
            if t > runs {                                            // error overflow: longer run
                acc = t.wrapping_sub(runs);
                len = len.wrapping_add(1);
            } else {
                acc = t;
            }
            let step = self.g16(GRA_LINE_MAJOR_STEP);
            if !steep {
                // horizontal run on the current line, then step y
                let a = self.g16(GRA_LINE_X);
                let next = a.wrapping_add(len as i16);
                self.s16(GRA_LINE_X, next);
                let b = next.wrapping_sub(1);
                let yy = self.g16(GRA_LINE_Y);
                if let Some((c0, c1)) = self.gra_clip_horizontal(a, yy, b) {
                    self.scr_horizontal(pen, c0 as u16, c1 as u16, yy as u16);
                }
                self.s16(GRA_LINE_Y, yy.wrapping_add(step));
            } else {
                // vertical run in the current column, then step x
                let a = self.g16(GRA_LINE_Y);
                let next = a.wrapping_add(len as i16);
                self.s16(GRA_LINE_Y, next);
                let b = next.wrapping_sub(1);
                let xx = self.g16(GRA_LINE_X);
                if let Some((c0, c1)) = self.gra_clip_vertical(xx, a, b) {
                    self.scr_vertical(pen, xx as u16, c0 as u16, c1 as u16);
                }
                self.s16(GRA_LINE_X, xx.wrapping_add(step));
            }
            left -= 1;
        }
    }

    // ===============================================================================
    // Characters at the graphics cursor                                (1945 .. 19DE)
    // ===============================================================================

    /// `GRA WR CHAR` (`BBFC`, ROM `1945`): draw `ch` with its top-left corner at the
    /// graphics cursor, using pen for set bits and paper for clear ones, then advance the
    /// cursor by 8 pixels.  If the whole cell is inside the window the fast path draws
    /// 64 pixels directly with a rotating mask; otherwise every pixel is clipped
    /// individually.
    pub fn gra_wr_char(&mut self, ch: u8) {
        // The matrix is copied to B33A (shared scratch area) and consumed row by row.
        let m = self.txt_get_matrix(ch);
        let rows = self.txt_matrix_bytes(m);
        for (i, &b) in rows.iter().enumerate() { self.wr8(GRA_SCRATCH + i as u16, b); }

        let (x, y) = self.gra_cursor_to_screen();
        let all_inside = self.gra_point_in_window(x, y)
            && self.gra_point_in_window(x.wrapping_add(7), y.wrapping_sub(7));

        if all_inside {
            let dot = self.scr_dot_position(x as u16, y as u16);
            let (mut line, mut mask) = (dot.addr, dot.mask);
            for row in 0..8u16 {
                let bits = self.rd8(GRA_SCRATCH + row);
                let (mut a, mut mk) = (line, mask);
                for bit in 0..8 {
                    self.gra_char_pixel(bits & (0x80 >> bit) != 0, a, mk);
                    let carry = mk & 1 != 0;
                    mk = mk.rotate_right(1);
                    if carry { a = scr_next_byte(a); }
                }
                line = scr_next_line(line);
                mask = dot.mask;
            }
        } else {
            let mut yy = y;
            for row in 0..8u16 {
                let bits = self.rd8(GRA_SCRATCH + row);
                let mut xx = x;
                for bit in 0..8 {
                    if self.gra_point_in_window(xx, yy) {
                        let dot = self.scr_dot_position(xx as u16, yy as u16);
                        self.gra_char_pixel(bits & (0x80 >> bit) != 0, dot.addr, dot.mask);
                    }
                    xx = xx.wrapping_add(1);
                }
                yy = yy.wrapping_sub(1);
            }
        }

        // Advance the cursor by one character cell (8 pixels = 32/16/8 user units).
        let (cx, cy) = self.gra_ask_cursor();
        let adv: i16 = match self.scr_get_mode() { 0 => 32, 1 => 16, _ => 8 };
        self.gra_move_absolute(cx.wrapping_add(adv), cy);
    }

    /// ROM `19CF`: write one pixel of a character – pen if `set`, otherwise paper – via
    /// the `SCR WRITE` vector.
    fn gra_char_pixel(&mut self, set: bool, addr: u16, mask: u8) {
        let ink = if set { self.rd8(GRA_PEN) } else { self.rd8(GRA_PAPER) };
        self.scr_write(addr, ink, mask);
    }
}
