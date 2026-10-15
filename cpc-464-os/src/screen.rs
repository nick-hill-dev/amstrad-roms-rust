//! Screen pack (`SCR`): screen memory layout, modes, ink tables and low-level pixel,
//! byte and character-cell primitives.
//!
//! ROM `0AA0-1077` (plus the vector table at `0ABE`).
//!
//! ## Screen memory layout (all modes)
//!
//! The 16 KiB screen is 200 scan lines × 80 bytes.  A scan line `y` of character row `r`
//! lives at `base + ((r * 80 + offset) & 0x7FF) + (y & 7) * 0x800 + column`; the
//! hardware scroll `offset` wraps inside each 2 KiB block.  [`Cpc::scr_next_byte`],
//! [`Cpc::scr_next_line`] and friends implement exactly that wrap-around.
//!
//! ## Pixel formats
//!
//! | mode | pixels/byte | pens | mask table in `B1CF`   |
//! |------|-------------|------|------------------------|
//! | 0    | 2           | 16   | `AA 55`                |
//! | 1    | 4           | 4    | `88 44 22 11`          |
//! | 2    | 8           | 2    | `80 40 20 10 08 04 02 01` |

use crate::hw::Hardware;
use crate::sysvars::*;
use crate::Cpc;

/// Firmware colour number (0-31) -> hardware colour code, ROM `0D93`.
pub const COLOUR_TO_HW: [u8; 32] = [
    0x14, 0x04, 0x15, 0x1C, 0x18, 0x1D, 0x0C, 0x05, 0x0D, 0x16, 0x06, 0x17, 0x1E, 0x00, 0x1F, 0x0E,
    0x07, 0x0F, 0x12, 0x02, 0x13, 0x1A, 0x19, 0x1B, 0x0A, 0x03, 0x0B, 0x01, 0x08, 0x09, 0x10, 0x11,
];

/// Power-on ink tables, ROM `104D`: 17 bytes each (border, pens 0-15).
/// The first table becomes the *second* flash colour set, the other the *first* set.
pub const DEFAULT_INKS_SECOND: [u8; 17] =
    [0x04, 0x04, 0x0A, 0x13, 0x0C, 0x0B, 0x14, 0x15, 0x0D, 0x06, 0x1E, 0x1F, 0x07, 0x12, 0x19, 0x04, 0x17];
pub const DEFAULT_INKS_FIRST: [u8; 17] =
    [0x04, 0x04, 0x0A, 0x13, 0x0C, 0x0B, 0x14, 0x15, 0x0D, 0x06, 0x1E, 0x1F, 0x07, 0x12, 0x19, 0x0A, 0x07];

/// Result of [`Cpc::scr_dot_position`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DotPosition {
    /// Address of the screen byte containing the pixel.
    pub addr: u16,
    /// Mask of that pixel inside the byte.
    pub mask: u8,
    /// Pixels per byte minus one (1, 3 or 7) – the `B` register on exit.
    pub ppb_minus_1: u8,
}

/// Pixel-write mode installed by [`Cpc::scr_access`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccessMode { Force = 0, Xor = 1, And = 2, Or = 3 }

impl AccessMode {
    /// Address of the matching write routine in the ROM (`SCR PIXELS` etc).
    fn routine(self) -> u16 {
        match self { AccessMode::Force => 0x0C6B, AccessMode::Xor => 0x0C72, AccessMode::And => 0x0C77, AccessMode::Or => 0x0C7D }
    }
    fn from_routine(a: u16) -> AccessMode {
        match a { 0x0C72 => AccessMode::Xor, 0x0C77 => AccessMode::And, 0x0C7D => AccessMode::Or, _ => AccessMode::Force }
    }
}

// ---- pure address arithmetic (no state needed) ------------------------------------

/// `SCR NEXT BYTE` (`BC20`, ROM `0BF9`): address of the byte to the right, wrapping
/// inside the 2 KiB screen block.
pub fn scr_next_byte(hl: u16) -> u16 {
    let l = (hl as u8).wrapping_add(1);
    let mut h = (hl >> 8) as u8;
    if l == 0 {
        h = h.wrapping_add(1);
        if h & 7 == 0 { h = h.wrapping_sub(8); }
    }
    (h as u16) << 8 | l as u16
}

/// `SCR PREV BYTE` (`BC23`, ROM `0C05`).
pub fn scr_prev_byte(hl: u16) -> u16 {
    let l0 = hl as u8;
    let h0 = (hl >> 8) as u8;
    let l = l0.wrapping_sub(1);
    if l0 != 0 { return (h0 as u16) << 8 | l as u16; }
    let mut h = h0.wrapping_sub(1);
    if h0 & 7 == 0 { h = h.wrapping_add(8); }
    (h as u16) << 8 | l as u16
}

/// `SCR NEXT LINE` (`BC26`, ROM `0C13`): same column, next scan line down.
pub fn scr_next_line(hl: u16) -> u16 {
    let mut h = ((hl >> 8) as u8).wrapping_add(8);
    let mut l = hl as u8;
    if h & 0x38 != 0 { return (h as u16) << 8 | l as u16; }
    // ran past the 8th line of the character row: back to line 0 of the next row (+80)
    h = h.wrapping_sub(0x40);
    let (nl, carry) = l.overflowing_add(0x50);
    l = nl;
    if carry {
        h = h.wrapping_add(1);
        if h & 7 == 0 { h = h.wrapping_sub(8); }
    }
    (h as u16) << 8 | l as u16
}

/// `SCR PREV LINE` (`BC29`, ROM `0C2D`): same column, previous scan line.
pub fn scr_prev_line(hl: u16) -> u16 {
    let mut h = ((hl >> 8) as u8).wrapping_sub(8);
    let mut l = hl as u8;
    if h & 0x38 != 0x38 { return (h as u16) << 8 | l as u16; }
    h = h.wrapping_add(0x40);
    let (nl, borrow) = l.overflowing_sub(0x50);
    l = nl;
    if borrow {
        let old = h;
        h = h.wrapping_sub(1);
        if old & 7 == 0 { h = h.wrapping_add(8); }
    }
    (h as u16) << 8 | l as u16
}

impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Initialisation and mode                                          (0AA0 .. 0B4D)
    // ===============================================================================

    /// `SCR INITIALISE` (`BBFF`, ROM `0AA0`): blank the inks, screen at `C000`, [`Self::scr_reset`],
    /// mode 1, clear the screen.
    pub fn scr_initialise(&mut self) {
        self.mc_clear_inks(&DEFAULT_INKS_SECOND);
        self.wr8(SCR_BASE, 0xC0);
        self.scr_reset();
        self.set_mode_tables(1);       // L_0B11 with A = 1
        self.scr_clear();
    }

    /// `SCR RESET` (`BC02`, ROM `0AB1`): pixel write = force mode, default indirections
    /// for `SCR READ/WRITE/MODE CLEAR`, default ink tables and flash periods `0x0A0A`.
    pub fn scr_reset(&mut self) {
        self.scr_access(AccessMode::Force);
        for (ind, target) in [(Ind::SCR_READ, 0x0C82u16), (Ind::SCR_WRITE, 0x0C68), (Ind::SCR_MODE_CLEAR, 0x0AF7)] {
            self.wr8(ind, 0xC3);
            self.wr16(ind + 1, target);
        }
        // L_0CD2 – default ink tables
        for i in 0..17 {
            self.wr8(SCR_INKS_SECOND + i, DEFAULT_INKS_SECOND[i as usize]);
            self.wr8(SCR_INKS_FIRST + i, DEFAULT_INKS_FIRST[i as usize]);
        }
        self.wr8(SCR_FLASH_PHASE, 0);
        self.scr_set_flashing(0x0A, 0x0A);
    }

    /// `SCR SET MODE` (`BC0E`, ROM `0ACA`): switch to `mode & 3` (3 is ignored).
    ///
    /// Sequence: stop ink flashing and blank the inks, remember every text stream's
    /// pen/paper and the graphics pens, program the mode, clear the screen (through
    /// the `SCR MODE CLEAR` vector, which restarts flashing) and then re-initialise
    /// the text streams and graphics window for the new mode.
    pub fn scr_set_mode(&mut self, mode: u8) {
        let mode = mode & 3;
        if mode == 3 { return; }
        self.scr_flash_stop();
        let stream = self.txt_save_pen_paper();            // L_10B7
        let pen_paper = self.gra_save_pen_paper();         // L_15D6
        self.set_mode_tables(mode);                        // L_0B11
        self.scr_mode_clear();                             // CALL BDEB
        self.gra_restore_pen_paper(pen_paper);             // L_15B6
        self.txt_restore_streams(stream);                  // L_10D5
    }

    /// ROM `0B11`: copy the pixel-mask table for `mode`, remember the mode and tell the
    /// gate array.
    fn set_mode_tables(&mut self, mode: u8) {
        let src = match mode { 0 => 0x0B3Au16, 1 => 0x0B36, _ => 0x0B2E };
        for i in 0..8u16 {
            let b = self.rom[(src + i) as usize];
            self.wr8(SCR_PIXEL_MASKS + i, b);
        }
        self.wr8(SCR_MODE, mode);
        self.mc_set_mode(mode);
    }

    /// `SCR GET MODE` (`BC11`, ROM `0AEC`): 0, 1 or 2.
    pub fn scr_get_mode(&self) -> u8 { self.rd8(SCR_MODE) }

    /// `SCR CLEAR` (`BC14`, ROM `0AF7`): stop flashing, offset 0, zero the 16 KiB screen,
    /// restart flashing.
    pub fn scr_clear(&mut self) {
        self.scr_flash_stop();
        self.scr_set_offset(0);
        // `LD (HL),0 : LDIR` – the LDIR reads through the CPU's view of memory, so a screen
        // based at 0000 (under the lower ROM) is "cleared" with a smear of ROM bytes.
        let base = (self.rd8(SCR_BASE) as u16) << 8;
        self.wr8(base, 0);
        for i in 0..0x3FFFu16 {
            let v = self.read(base.wrapping_add(i));
            self.wr8(base.wrapping_add(i).wrapping_add(1), v);
        }
        self.scr_flash_start();
    }

    /// The `SCR MODE CLEAR` indirection (`BDEB`), default [`Self::scr_clear`].
    fn scr_mode_clear(&mut self) {
        match self.patched(Ind::SCR_MODE_CLEAR, 0x0AF7) {
            None => self.scr_clear(),
            Some(a) => { let mut r = crate::Regs::default(); self.hw.call(a, &mut r); }
        }
    }

    /// `SCR SET OFFSET` (`BC05`, ROM `0B3C`): hardware scroll offset (11 bits).
    pub fn scr_set_offset(&mut self, offset: u16) {
        self.wr16(SCR_OFFSET, offset & 0x07FF);
        self.scr_update_crtc();
    }

    /// `SCR SET BASE` (`BC08`, ROM `0B45`): which 16 K bank the screen is in (`& 0xC0`).
    pub fn scr_set_base(&mut self, base: u8) {
        self.wr8(SCR_BASE, base & 0xC0);
        self.scr_update_crtc();
    }

    fn scr_update_crtc(&mut self) {
        let (off, base) = self.scr_get_location();
        self.mc_screen_offset(base, off);
    }

    /// `SCR GET LOCATION` (`BC0B`, ROM `0B50`): `(offset, base_high_byte)`.
    pub fn scr_get_location(&self) -> (u16, u8) { (self.rd16(SCR_OFFSET), self.rd8(SCR_BASE)) }

    // ===============================================================================
    // Geometry                                                         (0B57 .. 0C48)
    // ===============================================================================

    /// `SCR CHAR LIMITS` (`BC17`, ROM `0B57`): `(max_column, max_row)` = (19|39|79, 24).
    pub fn scr_char_limits(&self) -> (u8, u8) {
        match self.scr_get_mode() { 0 => (19, 24), 1 => (39, 24), _ => (79, 24) }
    }

    /// Bytes per character cell width: 4 / 2 / 1.
    fn bytes_per_char(&self) -> u8 { match self.scr_get_mode() { 0 => 4, 1 => 2, _ => 1 } }

    /// `SCR CHAR POSITION` (`BC1A`, ROM `0B64`): address of the top-left byte of the character
    /// cell at (`col`, `row`) and the cell width in bytes (the `B` register on exit).
    pub fn scr_char_position(&self, col: u8, row: u8) -> (u16, u8) {
        let bpc = self.bytes_per_char();
        let mut hl = (row as u16) * 80;
        hl = hl.wrapping_add((col as u16) * bpc as u16);
        hl = hl.wrapping_add(self.rd16(SCR_OFFSET));
        let h = ((hl >> 8) as u8 & 7).wrapping_add(self.rd8(SCR_BASE));
        ((h as u16) << 8 | (hl & 0xFF), bpc)
    }

    /// ROM `0B95`: geometry of a character box: `(address of top-left byte, width in bytes,
    /// height in scan lines)` for the box (`tl`) .. (`br`) given as (column, row) pairs.
    fn box_geometry(&self, tl: (u8, u8), br: (u8, u8)) -> (u16, u8, u8) {
        let lines = br.1.wrapping_sub(tl.1).wrapping_add(1).wrapping_mul(8);
        let cols = br.0.wrapping_sub(tl.0).wrapping_add(1);
        let (addr, bpc) = self.scr_char_position(tl.0, tl.1);
        let mut w = 0u8;
        for _ in 0..bpc { w = w.wrapping_add(cols); }
        (addr, w, lines)
    }

    /// `SCR DOT POSITION` (`BC1D`, ROM `0BA9`): screen byte and pixel mask for the point
    /// (`x` 0..639, `y` 0..199, origin at the **bottom** left – `y` is flipped via `199 - y`).
    pub fn scr_dot_position(&self, x: u16, y: u16) -> DotPosition {
        let ppb_minus_1: u8 = match self.scr_get_mode() { 0 => 1, 1 => 3, _ => 7 };
        let yy = 199u16.wrapping_sub(y);
        let line_in_char = ((yy as u8) & 7) << 3;           // added to the high byte
        let char_row_y = yy & 0xFFF8;                       // 8 * character row
        let mut hl = char_row_y.wrapping_mul(10);           // 80 bytes per character row
        let index = (x as u8) & ppb_minus_1;
        let mut dx = x;
        let mut r = ppb_minus_1.rotate_right(1);
        loop {                                              // shift x right log2(ppb) times
            dx >>= 1;
            let carry = r & 1 != 0;
            r = r.rotate_right(1);
            if !carry { break; }
        }
        hl = hl.wrapping_add(dx).wrapping_add(self.rd16(SCR_OFFSET));
        let h = ((hl >> 8) as u8 & 7).wrapping_add(self.rd8(SCR_BASE)).wrapping_add(line_in_char);
        DotPosition {
            addr: (h as u16) << 8 | (hl & 0xFF),
            mask: self.rd8(SCR_PIXEL_MASKS + index as u16),
            ppb_minus_1,
        }
    }

    // ===============================================================================
    // Pixel access                                                     (0C49 .. 0CC2)
    // ===============================================================================

    /// `SCR ACCESS` (`BC59`, ROM `0C49`): choose how `SCR WRITE` combines the new pixel
    /// with the old one.
    pub fn scr_access(&mut self, mode: AccessMode) {
        self.wr8(SCR_WRITE_JP, 0xC3);                    // JP opcode
        self.wr16(SCR_WRITE_JP + 1, mode.routine());
    }

    /// Current access mode, decoded from the `JP` stored at `B1CC`.
    pub fn scr_access_mode(&self) -> AccessMode { AccessMode::from_routine(self.rd16(SCR_WRITE_JP + 1)) }

    /// `SCR PIXELS` (`BC5C`, ROM `0C6B`): replace the pixels selected by `mask` (C) in the
    /// byte at `addr` by those of `ink` (B), leaving the others alone.
    pub fn scr_pixels(&mut self, addr: u16, ink: u8, mask: u8) {
        let x = self.read(addr);
        self.wr8(addr, (x & !mask) | (ink & mask));
    }

    /// `SCR WRITE` indirection (`BDE8`, default `JP 0C68` -> the routine chosen with
    /// [`Self::scr_access`]).  `ink` = B, `mask` = C.
    pub fn scr_write(&mut self, addr: u16, ink: u8, mask: u8) {
        if let Some(a) = self.patched(Ind::SCR_WRITE, 0x0C68) {
            let mut r = crate::Regs { hl: addr, bc: (ink as u16) << 8 | mask as u16, ..Default::default() };
            self.hw.call(a, &mut r);
            return;
        }
        let x = self.read(addr);
        let v = match self.scr_access_mode() {
            AccessMode::Force => (x & !mask) | (ink & mask),   // 0C6B
            AccessMode::Xor => x ^ (ink & mask),               // 0C72
            AccessMode::And => x & (ink | !mask),              // 0C77
            AccessMode::Or => x | (ink & mask),                // 0C7D
        };
        self.wr8(addr, v);
    }

    /// `SCR READ` indirection (`BDE5`, default `JP 0C82`): the pen of the pixel selected by
    /// `mask` in the byte at `addr`.
    pub fn scr_read(&mut self, addr: u16, mask: u8) -> u8 {
        if let Some(a) = self.patched(Ind::SCR_READ, 0x0C82) {
            let mut r = crate::Regs { hl: addr, bc: mask as u16, ..Default::default() };
            self.hw.call(a, &mut r);
            return r.a();
        }
        let b = self.read(addr);
        self.decode_pixel(b, mask)
    }

    /// Swap bits 1 and 2 of a pen number in mode 0 (ROM `0CC2`); identity in modes 1/2.
    /// Used both before encoding and after decoding because mode-0 pixel bits are
    /// interleaved in an unusual order.
    fn mode0_swap(&self, a: u8) -> u8 {
        if self.scr_get_mode() != 0 { return a; }
        if ((a >> 1) ^ (a >> 2)) & 1 != 0 { a ^ 0x06 } else { a }
    }

    /// `SCR INK ENCODE` (`BC2C`, ROM `0C86`): turn pen number `pen` into a byte with that
    /// pen in every pixel (e.g. mode 1 pen 1 = `0xF0`).
    pub fn scr_ink_encode(&self, pen: u8) -> u8 {
        let mut e = self.mode0_swap(pen);
        let mut c = self.rd8(SCR_PIXEL_MASKS);
        let mut a = c;
        for _ in 0..8 {
            let bit = e & 1;
            e = e.rotate_right(1);                 // RRC E
            a = (a << 1) | bit;                    // RLA (carry = bit)
            let mask_bit = c & 1;
            c = c.rotate_right(1);                 // RRC C
            if mask_bit == 0 { e = e.rotate_left(1); } // un-consume the pen bit
        }
        a
    }

    /// `SCR INK DECODE` (`BC2F`, ROM `0CA0`): pen number of the **leftmost** pixel of a
    /// screen byte (inverse of [`Self::scr_ink_encode`]).
    pub fn scr_ink_decode(&self, byte: u8) -> u8 {
        let mask = self.rd8(SCR_PIXEL_MASKS);
        self.decode_pixel(byte, mask)
    }

    /// ROM `0CAC`: gather the pen bits selected by `mask` from `byte`.
    fn decode_pixel(&self, byte: u8, mut mask: u8) -> u8 {
        let mut a = byte;
        let mut d = 0u8;
        for _ in 0..8 {
            let bit = a & 1;
            a = a.rotate_right(1);                 // RRCA
            d = (d << 1) | bit;                    // RL D
            let m = mask & 1;
            mask = mask.rotate_right(1);           // RRC C
            if m == 0 { d >>= 1; }                 // RR D (undo)
        }
        self.mode0_swap(d)
    }

    // ===============================================================================
    // Inks and flashing                                                (0CD2 .. 0D80)
    // ===============================================================================

    /// `SCR SET FLASHING` (`BC3E`, ROM `0CE4`): periods, in 1/50 s, of the first and second colour.
    pub fn scr_set_flashing(&mut self, first_period: u8, second_period: u8) {
        self.wr8(SCR_FLASH_PERIODS + 1, first_period);
        self.wr8(SCR_FLASH_PERIODS, second_period);
    }
    /// `SCR GET FLASHING` (`BC41`, ROM `0CE8`): `(first_period, second_period)`.
    pub fn scr_get_flashing(&self) -> (u8, u8) {
        (self.rd8(SCR_FLASH_PERIODS + 1), self.rd8(SCR_FLASH_PERIODS))
    }

    /// ROM `0D2F`: addresses of the (first-colour, second-colour) table entries for ink index
    /// `idx` (0 = border, 1..=16 = pens 0..=15).
    fn ink_slots(idx: u8) -> (u16, u16) { (SCR_INKS_FIRST + idx as u16, SCR_INKS_SECOND + idx as u16) }

    fn set_ink_slot(&mut self, idx: u8, colour1: u8, colour2: u8) {
        let hw1 = COLOUR_TO_HW[(colour1 & 0x1F) as usize];
        let hw2 = COLOUR_TO_HW[(colour2 & 0x1F) as usize];
        let (first, second) = Self::ink_slots(idx);
        self.wr8(second, hw2);
        self.wr8(first, hw1);
        self.wr8(SCR_INKS_DIRTY, 0xFF);
    }

    fn get_ink_slot(&self, idx: u8) -> (u8, u8) {
        let (first, second) = Self::ink_slots(idx);
        let find = |hw: u8| COLOUR_TO_HW.iter().position(|&x| x == hw).unwrap_or(0) as u8;
        (find(self.rd8(first)), find(self.rd8(second)))
    }

    /// `SCR SET INK` (`BC32`, ROM `0CEC`): pen `pen & 15` flashes between firmware colours
    /// `colour1` (first) and `colour2` (second).
    pub fn scr_set_ink(&mut self, pen: u8, colour1: u8, colour2: u8) {
        self.set_ink_slot((pen & 0x0F) + 1, colour1, colour2);
    }
    /// `SCR GET INK` (`BC35`, ROM `0D14`): `(colour1, colour2)`.
    ///
    /// If the stored hardware code is not one of the 32 legal ones the ROM's lookup would
    /// run off the table; here that case yields 0.
    pub fn scr_get_ink(&self, pen: u8) -> (u8, u8) { self.get_ink_slot((pen & 0x0F) + 1) }
    /// `SCR SET BORDER` (`BC38`, ROM `0CF1`).
    pub fn scr_set_border(&mut self, colour1: u8, colour2: u8) { self.set_ink_slot(0, colour1, colour2); }
    /// `SCR GET BORDER` (`BC3B`, ROM `0D19`).
    pub fn scr_get_border(&self) -> (u8, u8) { self.get_ink_slot(0) }

    /// ROM `0D81`: which ink table is "current" and for how long.
    fn flash_current(&self) -> (u16, u8) {
        if self.rd8(SCR_FLASH_PHASE) == 0 {
            (SCR_INKS_FIRST, self.rd8(SCR_FLASH_PERIODS + 1))
        } else {
            (SCR_INKS_SECOND, self.rd8(SCR_FLASH_PERIODS))
        }
    }

    fn ink_table(&self, base: u16) -> [u8; 17] {
        let mut t = [0u8; 17];
        for i in 0..17 { t[i] = self.rd8(base + i as u16); }
        t
    }

    /// ROM `0D3C`: (re)start the flash engine – a frame-flyback event running
    /// [`Self::scr_flash_tick`] every frame.
    fn scr_flash_start(&mut self) {
        self.kl_del_frame_fly(SCR_FLASH_BLOCK);
        self.scr_flash_swap();
        self.kl_new_frame_fly(SCR_FLASH_BLOCK, 0x0D5B, 0, 0x81);
    }

    /// ROM `0D4F`: stop flashing and blank all inks (used around mode changes and clears).
    fn scr_flash_stop(&mut self) {
        self.kl_del_frame_fly(SCR_FLASH_BLOCK);
        let (t, _) = self.flash_current();
        let table = self.ink_table(t);
        self.mc_clear_inks(&table);
    }

    /// ROM `0D6D`: show the colour set that is due, arm the countdown and flip the phase.
    fn scr_flash_swap(&mut self) {
        let (t, period) = self.flash_current();
        self.wr8(SCR_FLASH_COUNTDOWN, period);
        let table = self.ink_table(t);
        self.mc_set_inks(&table);
        let ph = !self.rd8(SCR_FLASH_PHASE);
        self.wr8(SCR_FLASH_PHASE, ph);
        self.wr8(SCR_INKS_DIRTY, 0);
    }

    /// Event routine at ROM `0D5B`, run once per frame: when the countdown reaches zero
    /// swap colour sets; otherwise, if `SCR SET INK`/`BORDER` changed a table, reprogram the
    /// gate array now.  (Quirk: the refresh uses the table selected by the *already
    /// flipped* phase, i.e. the one due next.)
    pub fn scr_flash_tick(&mut self) {
        let c = self.rd8(SCR_FLASH_COUNTDOWN).wrapping_sub(1);
        self.wr8(SCR_FLASH_COUNTDOWN, c);
        if c == 0 { self.scr_flash_swap(); return; }
        if self.rd8(SCR_INKS_DIRTY) == 0 { return; }
        let (t, _) = self.flash_current();
        let table = self.ink_table(t);
        self.mc_set_inks(&table);
        self.wr8(SCR_INKS_DIRTY, 0);
    }

    // ===============================================================================
    // Boxes, rolls and character cells                                 (0DB3 .. 0F98)
    // ===============================================================================

    /// `SCR FILL BOX` (`BC44`, ROM `0DB3`): fill the character box `tl`..`br` (column,row)
    /// with the encoded ink byte `pattern`.
    pub fn scr_fill_box(&mut self, pattern: u8, tl: (u8, u8), br: (u8, u8)) {
        let (addr, w, h) = self.box_geometry(tl, br);
        self.scr_flood_box(addr, w, h, pattern);
    }

    /// `SCR FLOOD BOX` (`BC47`, ROM `0DB7`): fill `width` bytes × `lines` scan lines
    /// starting at `addr` with `pattern`.  (ROM registers: `HL` address, `D` width,
    /// `E` lines, `C` pattern; a count of 0 means 256.)
    ///
    /// Each line is filled either byte by byte, when it would cross a 2 KiB screen block
    /// boundary (ROM `0EE8` decides), or by writing the first byte and `LDIR`-copying it
    /// along the line.  ROM quirk (kept): that `LDIR` *reads* through the CPU's memory
    /// view, so with the screen based at `0000` (under the lower ROM) it replicates ROM
    /// bytes instead of the pattern.
    pub fn scr_flood_box(&mut self, addr: u16, width: u8, lines: u8, pattern: u8) {
        let mut line = addr;
        let mut e = lines;
        loop {
            let carry = (width.wrapping_sub(1) as u16 + (line & 0xFF)) > 0xFF;
            if carry && (line >> 8) & 7 == 7 {
                let mut a = line;
                let mut b = width;
                loop {
                    self.wr8(a, pattern);
                    a = scr_next_byte(a);
                    b = b.wrapping_sub(1);
                    if b == 0 { break; }
                }
            } else {
                self.wr8(line, pattern);
                let n = width.wrapping_sub(1);
                if n != 0 {
                    for i in 0..n as u16 {
                        let v = self.read(line.wrapping_add(i));
                        self.wr8(line.wrapping_add(i).wrapping_add(1), v);
                    }
                }
            }
            line = scr_next_line(line);
            e = e.wrapping_sub(1);
            if e == 0 { break; }
        }
    }

    /// `SCR CHAR INVERT` (`BC4A`, ROM `0DDF`): exchange pen `b` and paper `c` (encoded inks)
    /// in the character cell at (`col`,`row`) by XOR-ing with `b ^ c`.
    pub fn scr_char_invert(&mut self, b: u8, c: u8, col: u8, row: u8) {
        let x = b ^ c;
        let (mut line, bpc) = self.scr_char_position(col, row);
        for _ in 0..8 {
            let mut a = line;
            for _ in 0..bpc {
                let v = self.read(a) ^ x;
                self.wr8(a, v);
                a = scr_next_byte(a);
            }
            line = scr_next_line(line);
        }
    }

    /// `SCR HW ROLL` (`BC4D`, ROM `0DFA`): scroll the whole screen one character row by moving
    /// the CRTC start address, after clearing the row that is about to appear with `pattern`.
    ///
    /// `direction == 0`: contents move *down* (start address − 80); non-zero: contents move *up*.
    pub fn scr_hw_roll(&mut self, direction: u8, pattern: u8) {
        // 1. pre-clear the last 0x30 bytes before the current start (8 lines)
        self.roll_fill(-0x30, 0x30, pattern);
        self.mc_wait_flyback();
        if direction == 0 {
            self.roll_set_offset(-0x50);
            self.roll_fill(0, 0x20, pattern);
        } else {
            self.roll_set_offset(0x50);
            self.roll_fill(-0x50, 0x20, pattern);
        }
    }

    fn roll_addr(&self, delta: i32) -> u16 {
        let off = (self.rd16(SCR_OFFSET) as i32 + delta) as u16;
        let h = ((off >> 8) as u8 & 7).wrapping_add(self.rd8(SCR_BASE));
        (h as u16) << 8 | (off & 0xFF)
    }
    fn roll_fill(&mut self, delta: i32, width: u8, pattern: u8) {
        let a = self.roll_addr(delta);
        self.scr_flood_box(a, width, 8, pattern);
    }
    fn roll_set_offset(&mut self, delta: i32) {
        let off = (self.rd16(SCR_OFFSET) as i32 + delta) as u16;
        self.scr_set_offset(off);
    }

    /// `SCR SW ROLL` (`BC50`, ROM `0E3E`): scroll the character box `tl`..`br` by one
    /// character row by copying scan lines; the vacated row is filled with `pattern`.
    /// `direction != 0`: text moves up, otherwise down.
    pub fn scr_sw_roll(&mut self, direction: u8, pattern: u8, tl: (u8, u8), br: (u8, u8)) {
        let (addr, width, lines) = self.box_geometry(tl, br);
        let copy_lines = lines.wrapping_sub(8);
        if direction != 0 {
            // up: line n+8 -> line n, then clear the bottom row
            let (mut src, _) = self.scr_char_position(tl.0, tl.1 + 1);
            let mut dst = addr;
            if copy_lines != 0 { self.mc_wait_flyback(); }
            for _ in 0..copy_lines {
                self.copy_screen_line(src, dst, width);
                src = scr_next_line(src);
                dst = scr_next_line(dst);
            }
            self.scr_flood_box(dst, width, 8, pattern);
        } else {
            // down: copy from the bottom up, then clear the top row
            if copy_lines != 0 {
                let (mut dst, _) = self.scr_char_position(tl.0, br.1 + 1);
                let (mut src, _) = self.scr_char_position(tl.0, br.1);
                self.mc_wait_flyback();
                for _ in 0..copy_lines {
                    dst = scr_prev_line(dst);
                    src = scr_prev_line(src);
                    self.copy_screen_line(src, dst, width);
                }
            }
            self.scr_flood_box(addr, width, 8, pattern);
        }
    }

    /// ROM `0EA4`: copy `width` bytes between two screen addresses, each wrapping inside its
    /// 2 KiB block.
    fn copy_screen_line(&mut self, mut src: u16, mut dst: u16, width: u8) {
        for _ in 0..width {
            let v = self.read(src);
            self.wr8(dst, v);
            src = scr_next_byte(src);
            dst = scr_next_byte(dst);
        }
    }

    /// `SCR UNPACK` (`BC53`, ROM `0EF3`): expand an 8-byte 1-bit-per-pixel character
    /// `matrix` into screen format using pen 1 of the current mode (set pixels get all
    /// plane bits).  Output length: 8 bytes in mode 2, 16 in mode 1, 32 in mode 0.
    pub fn scr_unpack(&self, matrix: &[u8; 8]) -> Vec<u8> {
        let m = |i: usize| self.rd8(SCR_PIXEL_MASKS + i as u16);
        let mut out = Vec::new();
        match self.scr_get_mode() {
            2 => out.extend_from_slice(matrix),
            1 => for &b in matrix {
                for half in 0..2 {
                    let mut a = 0u8;
                    for k in 0..4 {
                        if b & (0x80 >> (half * 4 + k)) != 0 { a |= m(k); }
                    }
                    out.push(a);
                }
            },
            _ => for &b in matrix {
                for pair in 0..4 {
                    let mut a = 0u8;
                    if b & (0x80 >> (pair * 2)) != 0 { a = m(0); }
                    if b & (0x80 >> (pair * 2 + 1)) != 0 { a |= m(1); }
                    out.push(a);
                }
            },
        }
        out
    }

    /// `SCR REPACK` (`BC56`, ROM `0F49`): read the character cell at (`col`,`row`) back as an
    /// 8-byte matrix where a 1 bit means "pixel equals `pen`" (an encoded ink byte).
    pub fn scr_repack(&mut self, pen: u8, col: u8, row: u8) -> [u8; 8] {
        let (mut line, _) = self.scr_char_position(col, row);
        let mode = self.scr_get_mode();
        let mut out = [0u8; 8];
        for o in out.iter_mut() {
            *o = match mode {
                2 => !(self.read(line) ^ pen),
                1 => {
                    let mut e = 0u8;
                    let mut a = line;
                    for _ in 0..2 {
                        let x = self.read(a) ^ pen;
                        for k in 0..4 {
                            e = (e << 1) | ((x & self.rd8(SCR_PIXEL_MASKS + k) == 0) as u8);
                        }
                        a = scr_next_byte(a);
                    }
                    e
                }
                _ => {
                    let mut e = 0u8;
                    let mut a = line;
                    for _ in 0..4 {
                        let x = self.read(a) ^ pen;
                        e = (e << 1) | ((x & self.rd8(SCR_PIXEL_MASKS) == 0) as u8);
                        e = (e << 1) | ((x & self.rd8(SCR_PIXEL_MASKS + 1) == 0) as u8);
                        a = scr_next_byte(a);
                    }
                    e
                }
            };
            line = scr_next_line(line);
        }
        out
    }

    // ===============================================================================
    // Lines                                                            (0FC4 .. 104C)
    // ===============================================================================

    /// `SCR HORIZONTAL` (`BC5F`, ROM `0FC4`): draw the pixels `x1..=x2` of row `y` using `ink`
    /// (encoded) through the `SCR WRITE` vector: partial first byte, whole bytes, partial
    /// last byte.
    pub fn scr_horizontal(&mut self, ink: u8, x1: u16, x2: u16, y: u16) {
        let mut count = x2.wrapping_sub(x1).wrapping_add(1);
        let dp = self.scr_dot_position(x1, y);
        let ppb = dp.ppb_minus_1 as u16 + 1;
        // The ROM keeps `-(pixels per byte)` in a scratch word (`B207`) for its loop; nothing
        // reads it afterwards, but it is written so the whole of RAM matches the ROM's.
        self.wr16(SCR_LINE_SCRATCH, 0xFF00 | !dp.ppb_minus_1 as u16);
        let mut addr = dp.addr;
        let lead = 0u8.wrapping_sub(x1 as u8) & dp.ppb_minus_1;

        if lead != 0 {
            // pixels from x1 to the end of its byte (limited by `count`)
            let mut idx = (x1 as u8 & dp.ppb_minus_1) as u16;
            let mut mask = dp.mask;
            let mut b = lead;
            loop {
                count = count.wrapping_sub(1);
                if count == 0 { self.scr_write(addr, ink, mask); return; }
                idx += 1;
                b -= 1;
                if b == 0 { break; }
                mask |= self.rd8(SCR_PIXEL_MASKS + idx);
            }
            self.scr_write(addr, ink, mask);
            addr = scr_next_byte(addr);
        }
        // whole bytes
        while count >= ppb {
            self.scr_write(addr, ink, 0xFF);
            addr = scr_next_byte(addr);
            count -= ppb;
        }
        // trailing pixels
        if count != 0 {
            let mut mask = 0u8;
            for i in 0..count { mask |= self.rd8(SCR_PIXEL_MASKS + i); }
            self.scr_write(addr, ink, mask);
        }
    }

    /// `SCR VERTICAL` (`BC62`, ROM `102F`): draw from `(x, y1)` upwards to `y2` (inclusive,
    /// y measured from the bottom) one pixel per scan line.
    pub fn scr_vertical(&mut self, ink: u8, x: u16, y1: u16, y2: u16) {
        let mut count = y2.wrapping_sub(y1).wrapping_add(1);
        let dp = self.scr_dot_position(x, y1);
        let mut addr = dp.addr;
        loop {
            self.scr_write(addr, ink, dp.mask);
            addr = scr_prev_line(addr);
            count = count.wrapping_sub(1);
            if count == 0 { break; }
        }
    }
}
