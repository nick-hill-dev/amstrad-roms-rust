//! # Keyboard manager  (ROM `19E0 .. 1E67`)
//!
//! Scans the 10×8 keyboard matrix every 50 Hz (called from the interrupt, see
//! [`Cpc::keyboard_scan`]), turns *new* key presses into entries of a 20-entry key queue,
//! generates auto-repeat, and translates queued keys into characters using three
//! 80-byte tables (plain / SHIFT / CONTROL).  Characters `0x80..=0x9F` are *expansion
//! tokens*: `KM READ CHAR` replaces them by the programmable strings (function keys).
//! Pressing ESC raises the *break* event; CONTROL+SHIFT+ESC resets the machine.
//!
//! ## Key numbers
//!
//! A key number is `row * 8 + bit` where `row` is the matrix line 0-9 and `bit` the
//! column 0-7 (so ESC is `8*8+2 = 66`).  The line/bit layout is documented next to the
//! default tables below.
//!
//! ## State (all in firmware RAM)
//!
//! | Address     | Meaning                                                          |
//! |-------------|------------------------------------------------------------------|
//! | `B34C`      | translation table (80 bytes), then SHIFT `B39C`, CONTROL `B3EC`   |
//! | `B43C`      | repeat-enable bit table (10 bytes, 1 bit per key)                 |
//! | `B446`      | default expansion string buffer (`0x98` bytes)                    |
//! | `B4DE/DF`   | expansion in progress: index / token                              |
//! | `B4E0`      | character pushed back by `KM CHAR RETURN` (`FF` = none)           |
//! | `B4E1..E5`  | expansion buffer: start, limit, end of used part                  |
//! | `B4E7/E8`   | SHIFT LOCK / CAPS LOCK (`00` off, `FF` on)                        |
//! | `B4E9/EA`   | repeat period / start delay (in 1/50 s)                           |
//! | `B4EB`      | 10 bytes: keys down at the last scan (1 = down)                   |
//! | `B4F5`      | 10 bytes: raw matrix of the previous scan (active low)            |
//! | `B4FF`      | 10 bytes: keys down now *or* at the previous scan (de-bounce)     |
//! | `B509`      | repeat countdown; `B50A/B`: key being repeated (row, mask)        |
//! | `B50C/B50D` | break event armed flag / its event block                          |
//! | `B514`      | key queue: 20 × (key-info, bit-mask); `B53C..B540` queue counters |
//! | `B541..B547`| pointers to the four tables above                                 |

use crate::hw::{Hardware, Regs};
use crate::sysvars::Ind;
use crate::Cpc;

// ---- addresses -------------------------------------------------------------------
const KM_TRANSLATE_TABLE: u16 = 0xB34C;
const KM_REPEAT_TABLE: u16 = 0xB43C;
const KM_DEFAULT_EXP_BUFFER: u16 = 0xB446;
const KM_DEFAULT_EXP_SIZE: u16 = 0x0098;
const KM_EXPANSION_INDEX: u16 = 0xB4DE;
const KM_EXPANSION_TOKEN: u16 = 0xB4DF;
const KM_CHAR_RETURNED: u16 = 0xB4E0;
const KM_EXP_TABLE: u16 = 0xB4E1;
const KM_EXP_LIMIT: u16 = 0xB4E3;
const KM_EXP_END: u16 = 0xB4E5;
const KM_SHIFT_LOCK: u16 = 0xB4E7;
const KM_CAPS_LOCK: u16 = 0xB4E8;
const KM_REPEAT_PERIOD: u16 = 0xB4E9;
const KM_START_DELAY: u16 = 0xB4EA;
const KM_KEY_STATE: u16 = 0xB4EB;
const KM_RAW: u16 = 0xB4F5;
const KM_HELD: u16 = 0xB4FF;
const KM_REPEAT_COUNT: u16 = 0xB509;
const KM_REPEAT_KEY: u16 = 0xB50A;
const KM_BREAK_ARMED: u16 = 0xB50C;
const KM_BREAK_BLOCK: u16 = 0xB50D;
const KM_QUEUE: u16 = 0xB514;
const KM_QUEUE_FREE: u16 = 0xB53C;
const KM_QUEUE_PUT: u16 = 0xB53D;
const KM_QUEUE_USED: u16 = 0xB53E;
const KM_QUEUE_GET: u16 = 0xB53F;
const KM_QUEUE_COUNT: u16 = 0xB540;
const KM_PTR_TRANSLATE: u16 = 0xB541;
const KM_PTR_SHIFT: u16 = 0xB543;
const KM_PTR_CONTROL: u16 = 0xB545;
const KM_PTR_REPEAT: u16 = 0xB547;

/// Queue entry written by `KM BREAK EVENT`; `KM READ KEY` returns it unchanged as the
/// ESC/break character.
const BREAK_ENTRY: u8 = 0xEF;

/// Expansion strings installed by reset (ROM `1AB3`): token 10 = ".", 11 = CR,
/// 12 = `RUN"` CR.  Tokens 0-9 are the digits `0`-`9`.
const DEFAULT_EXPANSIONS: [u8; 10] = [0x01, b'.', 0x01, 0x0D, 0x05, b'R', b'U', b'N', b'"', 0x0D];

/// Default key translation table (ROM `1D69`), no modifiers.
///
/// One line per keyboard-matrix row (bit 0 first); `0xFF` = no character.
/// `0x80..=0x9F` are expansion tokens, `0xE0` COPY, `0xF0..=0xFB` cursor keys with
/// modifiers, `0xFC` ESC, `0xFD` CAPS LOCK toggle, `0xFE` SHIFT LOCK toggle.
pub const DEFAULT_TRANSLATE: [u8; 80] = [
    0xF0, 0xF3, 0xF1, 0x89, 0x86, 0x83, 0x8B, 0x8A, // row 0: CUR UP, CUR RIGHT, CUR DOWN, F9, F6, F3, ENTER, F.
    0xF2, 0xE0, 0x87, 0x88, 0x85, 0x81, 0x82, 0x80, // row 1: CUR LEFT, COPY, F7, F8, F5, F1, F2, F0
    0x10, 0x5B, 0x0D, 0x5D, 0x84, 0xFF, 0x5C, 0xFF, // row 2: CLR, [, RETURN, ], F4, SHIFT, \, CONTROL
    0x5E, 0x2D, 0x40, 0x70, 0x3B, 0x3A, 0x2F, 0x2E, // row 3: ^, -, @, P, ;, :, /, .
    0x30, 0x39, 0x6F, 0x69, 0x6C, 0x6B, 0x6D, 0x2C, // row 4: 0, 9, O, I, L, K, M, ,
    0x38, 0x37, 0x75, 0x79, 0x68, 0x6A, 0x6E, 0x20, // row 5: 8, 7, U, Y, H, J, N, SPACE
    0x36, 0x35, 0x72, 0x74, 0x67, 0x66, 0x62, 0x76, // row 6: 6, 5, R, T, G, F, B, V
    0x34, 0x33, 0x65, 0x77, 0x73, 0x64, 0x63, 0x78, // row 7: 4, 3, E, W, S, D, C, X
    0x31, 0x32, 0xFC, 0x71, 0x09, 0x61, 0xFD, 0x7A, // row 8: 1, 2, ESC, Q, TAB, A, CAPS LOCK, Z
    0x0B, 0x0A, 0x08, 0x09, 0x58, 0x5A, 0xFF, 0x7F, // row 9: JOY UP, JOY DOWN, JOY LEFT, JOY RIGHT, FIRE 2, FIRE 1, (spare), DEL
];
/// Default SHIFT translation table (ROM `1DB9`).
///
/// One line per keyboard-matrix row (bit 0 first); `0xFF` = no character.
/// `0x80..=0x9F` are expansion tokens, `0xE0` COPY, `0xF0..=0xFB` cursor keys with
/// modifiers, `0xFC` ESC, `0xFD` CAPS LOCK toggle, `0xFE` SHIFT LOCK toggle.
pub const DEFAULT_SHIFT: [u8; 80] = [
    0xF4, 0xF7, 0xF5, 0x89, 0x86, 0x83, 0x8B, 0x8A, // row 0: CUR UP, CUR RIGHT, CUR DOWN, F9, F6, F3, ENTER, F.
    0xF6, 0xE0, 0x87, 0x88, 0x85, 0x81, 0x82, 0x80, // row 1: CUR LEFT, COPY, F7, F8, F5, F1, F2, F0
    0x10, 0x7B, 0x0D, 0x7D, 0x84, 0xFF, 0x60, 0xFF, // row 2: CLR, [, RETURN, ], F4, SHIFT, \, CONTROL
    0xA3, 0x3D, 0x7C, 0x50, 0x2B, 0x2A, 0x3F, 0x3E, // row 3: ^, -, @, P, ;, :, /, .
    0x5F, 0x29, 0x4F, 0x49, 0x4C, 0x4B, 0x4D, 0x3C, // row 4: 0, 9, O, I, L, K, M, ,
    0x28, 0x27, 0x55, 0x59, 0x48, 0x4A, 0x4E, 0x20, // row 5: 8, 7, U, Y, H, J, N, SPACE
    0x26, 0x25, 0x52, 0x54, 0x47, 0x46, 0x42, 0x56, // row 6: 6, 5, R, T, G, F, B, V
    0x24, 0x23, 0x45, 0x57, 0x53, 0x44, 0x43, 0x58, // row 7: 4, 3, E, W, S, D, C, X
    0x21, 0x22, 0xFC, 0x51, 0x09, 0x41, 0xFD, 0x5A, // row 8: 1, 2, ESC, Q, TAB, A, CAPS LOCK, Z
    0x0B, 0x0A, 0x08, 0x09, 0x58, 0x5A, 0xFF, 0x7F, // row 9: JOY UP, JOY DOWN, JOY LEFT, JOY RIGHT, FIRE 2, FIRE 1, (spare), DEL
];
/// Default CONTROL translation table (ROM `1E09`).
///
/// One line per keyboard-matrix row (bit 0 first); `0xFF` = no character.
/// `0x80..=0x9F` are expansion tokens, `0xE0` COPY, `0xF0..=0xFB` cursor keys with
/// modifiers, `0xFC` ESC, `0xFD` CAPS LOCK toggle, `0xFE` SHIFT LOCK toggle.
pub const DEFAULT_CONTROL: [u8; 80] = [
    0xF8, 0xFB, 0xF9, 0x89, 0x86, 0x83, 0x8C, 0x8A, // row 0: CUR UP, CUR RIGHT, CUR DOWN, F9, F6, F3, ENTER, F.
    0xFA, 0xE0, 0x87, 0x88, 0x85, 0x81, 0x82, 0x80, // row 1: CUR LEFT, COPY, F7, F8, F5, F1, F2, F0
    0x10, 0x1B, 0x0D, 0x1D, 0x84, 0xFF, 0x1C, 0xFF, // row 2: CLR, [, RETURN, ], F4, SHIFT, \, CONTROL
    0x1E, 0xFF, 0x00, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, // row 3: ^, -, @, P, ;, :, /, .
    0x1F, 0xFF, 0x0F, 0x09, 0x0C, 0x0B, 0x0D, 0xFF, // row 4: 0, 9, O, I, L, K, M, ,
    0xFF, 0xFF, 0x15, 0x19, 0x08, 0x0A, 0x0E, 0xFF, // row 5: 8, 7, U, Y, H, J, N, SPACE
    0xFF, 0xFF, 0x12, 0x14, 0x07, 0x06, 0x02, 0x16, // row 6: 6, 5, R, T, G, F, B, V
    0xFF, 0xFF, 0x05, 0x17, 0x13, 0x04, 0x03, 0x18, // row 7: 4, 3, E, W, S, D, C, X
    0xFF, 0x7E, 0xFC, 0x11, 0xE1, 0x01, 0xFE, 0x1A, // row 8: 1, 2, ESC, Q, TAB, A, CAPS LOCK, Z
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, // row 9: JOY UP, JOY DOWN, JOY LEFT, JOY RIGHT, FIRE 2, FIRE 1, (spare), DEL
];
/// Default auto-repeat enables (ROM `1E59`): one bit per key, 1 = repeats.
pub const DEFAULT_REPEAT: [u8; 10] = [0x07, 0x03, 0x4B, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAB, 0x8F];


impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Initialisation                                                   (19E0 .. 1AB2)
    // ===============================================================================

    /// `KM INITIALISE` (`BB00`, ROM `19E0`): delays 30/2, locks off, default tables, key
    /// state "nothing pressed", then [`Self::km_reset`].
    pub fn km_initialise(&mut self) {
        self.km_set_delay(0x1E, 0x02);
        self.wr8(KM_REPEAT_KEY + 1, 0);
        self.wr16(KM_SHIFT_LOCK, 0);                     // SHIFT LOCK and CAPS LOCK off

        // Table pointers: four consecutive blocks, 0x50 bytes apart.
        self.wr16(KM_PTR_TRANSLATE, KM_TRANSLATE_TABLE);
        self.wr16(KM_PTR_SHIFT, KM_TRANSLATE_TABLE + 0x50);
        self.wr16(KM_PTR_CONTROL, KM_TRANSLATE_TABLE + 0xA0);
        self.wr16(KM_PTR_REPEAT, KM_REPEAT_TABLE);
        for (i, &b) in DEFAULT_TRANSLATE.iter().enumerate() { self.wr8(KM_TRANSLATE_TABLE + i as u16, b); }
        for (i, &b) in DEFAULT_SHIFT.iter().enumerate() { self.wr8(KM_TRANSLATE_TABLE + 0x50 + i as u16, b); }
        for (i, &b) in DEFAULT_CONTROL.iter().enumerate() { self.wr8(KM_TRANSLATE_TABLE + 0xA0 + i as u16, b); }
        for (i, &b) in DEFAULT_REPEAT.iter().enumerate() { self.wr8(KM_REPEAT_TABLE + i as u16, b); }

        for i in 0..10 { self.wr8(KM_KEY_STATE + i, 0x00); }   // nothing down
        for i in 0..10 { self.wr8(KM_RAW + i, 0xFF); }         // matrix is active low
        self.km_reset();
    }

    /// `KM RESET` (`BB03`, ROM `1A1E`): clear the key queue and the pushed-back character,
    /// reset the expansion strings, restore the `KM TEST BREAK` vector and disarm break.
    pub fn km_reset(&mut self) {
        self.km_init_queue();
        self.wr8(KM_CHAR_RETURNED, 0xFF);
        self.km_init_expansion(KM_DEFAULT_EXP_BUFFER, KM_DEFAULT_EXP_SIZE);
        self.wr8(Ind::KM_TEST_BREAK, 0xC3);
        self.wr16(Ind::KM_TEST_BREAK + 1, 0x1C2F);
        self.km_disarm_break();
    }

    /// ROM `1CED`: empty the key queue.  The two position counters start at 1 and the
    /// free counter at 21 so that "empty"/"full" show up as a decrement reaching 0.
    fn km_init_queue(&mut self) {
        self.wr8(KM_QUEUE_FREE, 0x15);
        self.wr8(KM_QUEUE_PUT, 0);
        self.wr8(KM_QUEUE_USED, 1);
        self.wr8(KM_QUEUE_GET, 0);
        self.wr8(KM_QUEUE_COUNT, 0);
    }

    // ===============================================================================
    // Reading characters                                               (1A3C .. 1A76)
    // ===============================================================================

    /// `KM WAIT CHAR` (`BB06`, ROM `1A3C`): wait for a character.  Time only passes
    /// through [`Self::interrupt`], so each failed poll runs one interrupt.
    pub fn km_wait_char(&mut self) -> u8 {
        loop {
            if let Some(c) = self.km_read_char() { return c; }
            self.interrupt();
        }
    }

    /// `KM READ CHAR` (`BB09`, ROM `1A42`): next character, or `None`.
    ///
    /// Order: a character pushed back with [`Self::km_char_return`], then the rest of an
    /// expansion string in progress, then the next key from the queue.  Keys `0x80..0x9F`
    /// start an expansion; other codes are returned as they are.
    pub fn km_read_char(&mut self) -> Option<u8> {
        let pushed = self.rd8(KM_CHAR_RETURNED);
        self.wr8(KM_CHAR_RETURNED, 0xFF);
        if pushed != 0xFF { return Some(pushed); }
        loop {
            let mut index = self.rd8(KM_EXPANSION_INDEX);
            let mut token = self.rd8(KM_EXPANSION_TOKEN);
            if token == 0 {
                let key = self.km_read_key()?;
                if !(0x80..0xA0).contains(&key) { return Some(key); }
                token = key;
                index = 0;
            }
            let ch = self.km_get_expand(token, index);
            if ch.is_none() { token = 0; }                 // end of string: forget the token
            self.wr8(KM_EXPANSION_INDEX, index.wrapping_add(1));
            self.wr8(KM_EXPANSION_TOKEN, token);
            if ch.is_some() { return ch; }
        }
    }

    /// `KM CHAR RETURN` (`BB0C`, ROM `1A77`): push `ch` back so the next
    /// [`Self::km_read_char`] returns it.
    pub fn km_char_return(&mut self, ch: u8) { self.wr8(KM_CHAR_RETURNED, ch); }

    // ===============================================================================
    // Expansion strings                                                (1A7B .. 1B3D)
    // ===============================================================================

    /// `KM EXP BUFFER` (`BB0F`, ROM `1A7B`): use `len` bytes at `buffer` for expansion
    /// strings (at least 0x31 bytes), reinstalling the defaults.  `false` if too small.
    pub fn km_exp_buffer(&mut self, buffer: u16, len: u16) -> bool { self.km_init_expansion(buffer, len) }

    /// ROM `1A81`: lay out the default strings in a new buffer.  The buffer is a sequence
    /// of 32 strings, each a length byte followed by that many characters.
    fn km_init_expansion(&mut self, buffer: u16, len: u16) -> bool {
        if len < 0x31 { return false; }
        self.wr16(KM_EXP_LIMIT, buffer.wrapping_add(len));
        self.wr16(KM_EXP_TABLE, buffer);
        let mut p = buffer;
        for i in 0..10u8 {                                     // tokens 0..9 = '0'..'9'
            self.wr8(p, 1);
            self.wr8(p + 1, b'0' + i);
            p += 2;
        }
        for &b in DEFAULT_EXPANSIONS.iter() {                  // tokens 10, 11, 12
            self.wr8(p, b);
            p += 1;
        }
        for _ in 0..19 {                                       // tokens 13..31 empty
            self.wr8(p, 0);
            p += 1;
        }
        self.wr16(KM_EXP_END, p);
        self.wr8(KM_EXPANSION_TOKEN, 0);
        true
    }

    /// ROM `1B3E`: locate expansion string `token & 0x7F` → `(length, address of its text)`.
    fn km_find_expansion(&self, token: u8) -> Option<(u8, u16)> {
        let t = token & 0x7F;
        if t >= 0x20 { return None; }
        let mut p = self.rd16(KM_EXP_TABLE);
        let mut len = 0u16;
        for _ in 0..=t {
            p = p.wrapping_add(len);
            len = self.rd8(p) as u16;
            p = p.wrapping_add(1);
        }
        Some((len as u8, p))
    }

    /// `KM GET EXPAND` (`BB12`, ROM `1B2E`): character `index` of expansion `token`;
    /// `None` if the token is invalid or `index` is past the end.
    pub fn km_get_expand(&self, token: u8, index: u8) -> Option<u8> {
        let (len, text) = self.km_find_expansion(token)?;
        if len > index { Some(self.rd8(text.wrapping_add(index as u16))) } else { None }
    }

    /// `KM SET EXPAND` (`BB15`, ROM `1ABE`): define expansion `token` (0..31) as `text`.
    /// Other strings are moved to make room; `false` if the buffer is full or the token
    /// invalid.
    ///
    /// ROM quirk (kept): the check that cancels an expansion already in progress compares
    /// the raw `token` argument with the stored token (which includes bit 7), so it only
    /// matches when the caller passes the full `0x80+n` code.
    pub fn km_set_expand(&mut self, token: u8, text: &[u8]) -> bool {
        let Some((old_len, text_addr)) = self.km_find_expansion(token) else { return false };
        let new_len = text.len() as u8;
        if !self.km_resize_expansion(old_len, new_len, text_addr) { return false; }
        let mut a = text_addr.wrapping_sub(1);
        self.wr8(a, new_len);
        for &b in text {
            a = a.wrapping_add(1);
            self.wr8(a, b);
        }
        if token == self.rd8(KM_EXPANSION_TOKEN) { self.wr8(KM_EXPANSION_TOKEN, 0); }
        true
    }

    /// ROM `1AE5`: make the string at `text` change length from `old_len` to `new_len` by
    /// sliding everything after it.  `false` if it would not fit before the buffer limit.
    fn km_resize_expansion(&mut self, old_len: u8, new_len: u8, text: u16) -> bool {
        if new_len == old_len { return true; }
        let end = self.rd16(KM_EXP_END);
        let after_old = text.wrapping_add(old_len as u16);
        let tail = end.wrapping_sub(after_old);                // bytes after the old string
        if new_len < old_len {
            let dest = text.wrapping_add(new_len as u16);
            for i in 0..tail {                                  // LDIR (moving down)
                let b = self.rd8(after_old.wrapping_add(i));
                self.wr8(dest.wrapping_add(i), b);
            }
            self.wr16(KM_EXP_END, dest.wrapping_add(tail));
        } else {
            let new_end = end.wrapping_add((new_len - old_len) as u16);
            if self.rd16(KM_EXP_LIMIT) < new_end { return false; }
            for i in (0..tail).rev() {                          // LDDR (moving up)
                let b = self.rd8(after_old.wrapping_add(i));
                self.wr8(new_end.wrapping_sub(tail).wrapping_add(i), b);
            }
            self.wr16(KM_EXP_END, new_end);
        }
        true
    }

    // ===============================================================================
    // Reading keys                                                     (1B56 .. 1BB6)
    // ===============================================================================

    /// `KM WAIT KEY` (`BB18`, ROM `1B56`): wait for a key (see [`Self::km_wait_char`]).
    pub fn km_wait_key(&mut self) -> u8 {
        loop {
            if let Some(c) = self.km_read_key() { return c; }
            self.interrupt();
        }
    }

    /// `KM READ KEY` (`BB1B`, ROM `1B5C`): next key from the queue, translated through the
    /// SHIFT / CONTROL / plain table, with CAPS LOCK applied.  Codes `FF` (nothing), `FE`
    /// (SHIFT LOCK toggle) and `FD` (CAPS LOCK toggle) are consumed and the next key tried.
    /// A break entry comes back as `0xEF`.
    pub fn km_read_key(&mut self) -> Option<u8> {
        loop {
            let (info, mut mask) = self.km_queue_get()?;
            if info == BREAK_ENTRY { return Some(BREAK_ENTRY); }
            // key number = row*8 + index of the lowest set bit of the mask
            let mut key = ((info & 0x0F) << 3).wrapping_sub(1);
            loop {
                key = key.wrapping_add(1);
                let carry = mask & 1 != 0;
                mask = mask.rotate_right(1);
                if carry { break; }
            }
            let mut ch = self.km_translate(key, info);
            if self.rd8(KM_CAPS_LOCK) & 0x80 != 0 && (0x61..0x7B).contains(&ch) {
                ch = ch.wrapping_add(0xE0);                      // to upper case
            }
            match ch {
                0xFF => continue,
                0xFE => { let v = !self.rd8(KM_SHIFT_LOCK); self.wr8(KM_SHIFT_LOCK, v); }
                0xFD => { let v = !self.rd8(KM_CAPS_LOCK); self.wr8(KM_CAPS_LOCK, v); }
                _ => return Some(ch),
            }
        }
    }

    /// ROM `1BA0`: pick the table for a queued key.  `info` bit 7 = CONTROL held,
    /// bit 5 = SHIFT held; SHIFT LOCK (`B4E7`) also selects the shift table.
    fn km_translate(&self, key: u8, info: u8) -> u8 {
        if info & 0x80 != 0 { return self.km_get_control(key); }
        if (self.rd8(KM_SHIFT_LOCK) | (info << 1)) & 0x40 != 0 {
            self.km_get_shift(key)
        } else {
            self.km_get_translate(key)
        }
    }

    /// `KM GET STATE` (`BB1E`, ROM `1BB3`): `(shift_lock, caps_lock)`, each `00` or `FF`.
    pub fn km_get_state(&self) -> (u8, u8) { (self.rd8(KM_SHIFT_LOCK), self.rd8(KM_CAPS_LOCK)) }

    // ===============================================================================
    // Key scan (called from the interrupt)                             (1BB7 .. 1C47)
    // ===============================================================================

    /// ROM `1BB7`: the 50 Hz keyboard scan.
    ///
    /// 1. Read the matrix; a key counts as down if it was down in *either* of the last two
    ///    scans (de-bounce).
    /// 2. SHIFT and CONTROL (row 2, bits 5 and 7) are never "new" keys: they only
    ///    modify keys pressed while they are held.
    /// 3. Every key that is down now but was not at the previous scan is queued with its
    ///    row, bit mask and the shift/control state, and becomes the repeating key.
    /// 4. If one of the row-8 keys was among them, `KM TEST BREAK` (`BDEE`) is called.
    /// 5. Otherwise, auto-repeat: after the start delay, and then every repeat period, the
    ///    held key is queued again (only when the queue is empty).
    pub fn keyboard_scan(&mut self) {
        let mut raw = [0u8; 10];
        let mut held = [0u8; 10];
        for i in 0..10 { raw[i] = self.rd8(KM_RAW + i as u16); }
        self.scan_keyboard_matrix(&mut raw, &mut held);
        for i in 0..10 {
            self.wr8(KM_RAW + i as u16, raw[i]);
            self.wr8(KM_HELD + i as u16, held[i]);
        }

        let modifiers = held[2] & 0xA0;                          // SHIFT (0x20) | CONTROL (0x80)
        let st2 = self.rd8(KM_KEY_STATE + 2) | modifiers;
        self.wr8(KM_KEY_STATE + 2, st2);

        let mut c = modifiers;                                   // low nibble = row counter
        let mut b = 0u8;                                         // last new-key mask (non-zero if any)
        for row in 0..10u16 {
            let old = self.rd8(KM_KEY_STATE + row);
            let now = held[row as usize];
            let new_keys = (old ^ now) & now;
            if new_keys != 0 { self.km_new_keys(new_keys, &mut c, &mut b); }
            self.wr8(KM_KEY_STATE + row, now);
            c = c.wrapping_add(1);
        }

        let break_candidate = c & 0x40 != 0;
        let c = c & 0xA0;
        if break_candidate { self.km_test_break_indirect(c); }
        if b != 0 { return; }                                    // new key: no repeat this time

        // ---- auto-repeat ----
        let count = self.rd8(KM_REPEAT_COUNT).wrapping_sub(1);
        self.wr8(KM_REPEAT_COUNT, count);
        if count != 0 { return; }
        let key = self.rd16(KM_REPEAT_KEY);
        let (row, mask) = (key as u8, (key >> 8) as u8);
        let down = self.rd8(KM_KEY_STATE + row as u16);
        let repeats = self.rd8(self.rd16(KM_PTR_REPEAT).wrapping_add(row as u16));
        if down & repeats & mask == 0 { return; }
        self.wr8(KM_REPEAT_COUNT, 1);                            // try again next scan if blocked
        if self.rd8(KM_QUEUE_COUNT) != 0 { return; }             // only repeat into an empty queue
        let mut c = c | row;
        let period = self.rd8(KM_REPEAT_PERIOD);
        self.km_key_pressed(period, &mut c, mask);
    }

    /// ROM `1C48`/`1C4A`: queue every key in `bits` (one row), lowest bit first.
    fn km_new_keys(&mut self, mut bits: u8, c: &mut u8, b: &mut u8) {
        loop {
            let e = bits;
            let lowest = e & e.wrapping_neg();
            *b = lowest;
            let delay = self.rd8(KM_START_DELAY);
            self.km_key_pressed(delay, c, lowest);
            bits = lowest ^ e;
            if bits == 0 { break; }
        }
    }

    /// ROM `1C18`: queue key (`c` = modifier bits | row, `mask`), start the repeat
    /// countdown at `countdown`, and remember it as the repeating key.  Keys on row 8
    /// other than TAB set bit 6 of `c`, asking the scan to call `KM TEST BREAK` (ESC lives
    /// on that row).
    fn km_key_pressed(&mut self, countdown: u8, c: &mut u8, mask: u8) {
        self.wr8(KM_REPEAT_COUNT, countdown);
        self.km_queue_put(*c, mask);
        let row = *c & 0x0F;
        self.wr16(KM_REPEAT_KEY, (mask as u16) << 8 | row as u16);
        if row == 8 && mask & 0x10 == 0 { *c |= 0x40; }
    }

    /// `BDEE` – `KM TEST BREAK` indirection; `c` = SHIFT/CONTROL bits.
    fn km_test_break_indirect(&mut self, c: u8) {
        if let Some(a) = self.patched(Ind::KM_TEST_BREAK, 0x1C2F) {
            let mut r = Regs { bc: c as u16, ..Default::default() };
            self.hw.call(a, &mut r);
        } else {
            self.km_test_break_default(c);
        }
    }

    /// Default `KM TEST BREAK` (ROM `1C2F`): if ESC is down, raise the break event – unless
    /// SHIFT+CONTROL are down and *nothing else* is, in which case the machine is reset.
    pub fn km_test_break_default(&mut self, c: u8) {
        if self.rd8(KM_KEY_STATE + 8) & 0x04 == 0 { return; }    // ESC not down
        if c != 0xA0 { self.km_break_event(); return; }
        // ADC chain over all ten state bytes: exactly SHIFT + CONTROL + ESC = 0xA4.
        let (mut sum, mut carry) = (0u8, false);
        for i in (0..10u16).rev() {
            let t = sum as u16 + self.rd8(KM_KEY_STATE + i) as u16 + carry as u16;
            sum = t as u8;
            carry = t > 0xFF;
        }
        if sum != 0xA4 { self.km_break_event(); return; }
        self.reset_requested = true;                              // RST 0
    }

    // ===============================================================================
    // Joystick, delays, repeat, tables                                 (1C5C .. 1D68)
    // ===============================================================================

    /// `KM GET JOYSTICK` (`BB24`, ROM `1C5C`): `(joystick 0, joystick 1)`; bits: 0 up,
    /// 1 down, 2 left, 3 right, 4 fire 2, 5 fire 1.
    pub fn km_get_joystick(&self) -> (u8, u8) {
        (self.rd8(KM_KEY_STATE + 9) & 0x7F, self.rd8(KM_KEY_STATE + 6) & 0x7F)
    }

    /// `KM GET DELAY` (`BB2D`, ROM `1C69`): `(start_delay, repeat_period)` in 1/50 s.
    pub fn km_get_delay(&self) -> (u8, u8) { (self.rd8(KM_START_DELAY), self.rd8(KM_REPEAT_PERIOD)) }

    /// `KM SET DELAY` (`BB2A`, ROM `1C6D`).
    pub fn km_set_delay(&mut self, start_delay: u8, repeat_period: u8) {
        self.wr8(KM_START_DELAY, start_delay);
        self.wr8(KM_REPEAT_PERIOD, repeat_period);
    }

    /// `KM ARM BREAK` (`BB48`, ROM `1C71`): the next ESC raises an express event running
    /// `routine` (in ROM select `rom`); one shot.
    pub fn km_arm_break(&mut self, routine: u16, rom: u8) {
        self.km_disarm_break();
        self.kl_init_event(KM_BREAK_BLOCK, routine, rom, 0x40);
        self.wr8(KM_BREAK_ARMED, 0xFF);
    }

    /// `KM DISARM BREAK` (`BB4B`, ROM `1C82`).
    pub fn km_disarm_break(&mut self) {
        self.wr8(KM_BREAK_ARMED, 0);
        self.kl_del_synchronous(KM_BREAK_BLOCK);
    }

    /// `KM BREAK EVENT` (`BB4E`, ROM `1C90`): if armed, disarm, raise the event and put
    /// the break marker in the key queue.
    pub fn km_break_event(&mut self) {
        let armed = self.rd8(KM_BREAK_ARMED);
        self.wr8(KM_BREAK_ARMED, 0);
        if armed == 0 { return; }
        self.kl_event(KM_BREAK_BLOCK);
        self.km_queue_put(BREAK_ENTRY, 0);
    }

    /// `KM GET REPEAT` (`BB..`, ROM `1CA6`): does the key repeat?
    pub fn km_get_repeat(&self, key: u8) -> bool {
        let (off, mask) = Self::km_bit(key);
        self.rd8(self.rd16(KM_PTR_REPEAT).wrapping_add(off)) & mask != 0
    }

    /// `KM SET REPEAT` (`BB..`, ROM `1CAB`): `repeat == 0xFF` enables, `0` disables
    /// (other values only enable where the bit happens to be set).  Keys ≥ 0x50 ignored.
    pub fn km_set_repeat(&mut self, key: u8, repeat: u8) {
        if key >= 0x50 { return; }
        let (off, mask) = Self::km_bit(key);
        let a = self.rd16(KM_PTR_REPEAT).wrapping_add(off);
        let cleared = self.rd8(a) & !mask;
        self.wr8(a, cleared | (mask & repeat));
    }

    /// `KM TEST KEY` (`BB1E`-ish, ROM `1CBD`): is key `key` down?  Also returns the
    /// SHIFT/CONTROL bits (0x20 / 0x80) as of the last scan.
    pub fn km_test_key(&self, key: u8) -> (bool, u8) {
        let mods = self.rd8(KM_KEY_STATE + 2) & 0xA0;
        let (off, mask) = Self::km_bit(key);
        (self.rd8(KM_KEY_STATE + off) & mask != 0, mods)
    }

    /// ROM `1CCD`: byte offset and bit mask of a key in a 1-bit-per-key table.
    fn km_bit(key: u8) -> (u16, u8) { ((key >> 3) as u16, 1 << (key & 7)) }

    /// `KM GET TRANSLATE` (`BB..`, ROM `1D3E`).
    pub fn km_get_translate(&self, key: u8) -> u8 { self.rd8(self.rd16(KM_PTR_TRANSLATE).wrapping_add(key as u16)) }
    /// `KM GET SHIFT` (ROM `1D43`).
    pub fn km_get_shift(&self, key: u8) -> u8 { self.rd8(self.rd16(KM_PTR_SHIFT).wrapping_add(key as u16)) }
    /// `KM GET CONTROL` (ROM `1D48`).
    pub fn km_get_control(&self, key: u8) -> u8 { self.rd8(self.rd16(KM_PTR_CONTROL).wrapping_add(key as u16)) }

    /// `KM SET TRANSLATE` (ROM `1D52`); keys ≥ 0x50 are ignored.
    pub fn km_set_translate(&mut self, key: u8, code: u8) { self.km_set_table(KM_PTR_TRANSLATE, key, code); }
    /// `KM SET SHIFT` (ROM `1D57`).
    pub fn km_set_shift(&mut self, key: u8, code: u8) { self.km_set_table(KM_PTR_SHIFT, key, code); }
    /// `KM SET CONTROL` (ROM `1D5C`).
    pub fn km_set_control(&mut self, key: u8, code: u8) { self.km_set_table(KM_PTR_CONTROL, key, code); }

    fn km_set_table(&mut self, ptr: u16, key: u8, code: u8) {
        if key >= 0x50 { return; }
        let a = self.rd16(ptr).wrapping_add(key as u16);
        self.wr8(a, code);
    }

    // ===============================================================================
    // Key queue (20 entries of 2 bytes)                                (1CFE .. 1D3D)
    // ===============================================================================

    /// ROM `1D2C`: address of queue slot `index` (0..19).
    fn km_slot(index: u8) -> u16 { KM_QUEUE + 2 * index as u16 }

    /// ROM `1CFE`: append (`info`, `mask`); `false` if the queue is full.
    fn km_queue_put(&mut self, info: u8, mask: u8) -> bool {
        let free = self.rd8(KM_QUEUE_FREE).wrapping_sub(1);
        if free == 0 {                                           // full: undo
            self.wr8(KM_QUEUE_FREE, 1);
            return false;
        }
        self.wr8(KM_QUEUE_FREE, free);
        let mut i = self.rd8(KM_QUEUE_PUT).wrapping_add(1);
        if i == 20 { i = 0; }
        self.wr8(KM_QUEUE_PUT, i);
        let a = Self::km_slot(i);
        self.wr8(a, info);
        self.wr8(a + 1, mask);
        let n = self.rd8(KM_QUEUE_COUNT).wrapping_add(1);
        self.wr8(KM_QUEUE_COUNT, n);
        let u = self.rd8(KM_QUEUE_USED).wrapping_add(1);
        self.wr8(KM_QUEUE_USED, u);
        true
    }

    /// ROM `1D15`: remove the oldest (`info`, `mask`).
    fn km_queue_get(&mut self) -> Option<(u8, u8)> {
        let used = self.rd8(KM_QUEUE_USED).wrapping_sub(1);
        if used == 0 {                                           // empty: undo
            self.wr8(KM_QUEUE_USED, 1);
            return None;
        }
        self.wr8(KM_QUEUE_USED, used);
        let mut i = self.rd8(KM_QUEUE_GET).wrapping_add(1);
        if i == 20 { i = 0; }
        self.wr8(KM_QUEUE_GET, i);
        let a = Self::km_slot(i);
        let (info, mask) = (self.rd8(a), self.rd8(a + 1));
        let n = self.rd8(KM_QUEUE_COUNT).wrapping_sub(1);
        self.wr8(KM_QUEUE_COUNT, n);
        let f = self.rd8(KM_QUEUE_FREE).wrapping_add(1);
        self.wr8(KM_QUEUE_FREE, f);
        Some((info, mask))
    }
}
