//! Firmware calls, through the RAM jumpblock.
//!
//! The BASIC ROM never calls into the lower ROM directly: every firmware service is a
//! `CALL` to its jumpblock entry in RAM (`BB00-BDCA`), so that a program or an expansion
//! ROM can redirect it by patching the entry.  This module keeps that behaviour:
//!
//! * if the entry still holds what `JUMP RESTORE` put there (`RST 1`/`RST 5` + target),
//!   the Rust implementation of the firmware routine in [`cpc464_os`] runs;
//! * otherwise the entry is called through [`Hardware::call`] with the Z80 registers the
//!   ROM would have loaded, and the results are taken from the returned registers.
//!
//! Each wrapper documents the entry it uses.  The Rust signatures carry only what BASIC
//! actually passes or looks at.  The `RST 5` entries of the arithmetic package
//! (`BD3D-BDCA`) are treated the same way.
//!
//! Besides the jumpblock, BASIC calls one routine of the RAM-resident kernel,
//! `KL POLL SYNCHRONOUS` at `B921`; that one is not patchable in practice and is called
//! directly.

use crate::{Basic, LineInput, Res, Exit};
use cpc464_os::cassette::{CasError, CasOpen};
use cpc464_os::jumpblock::{JumpEntry, JUMPBLOCK, JUMPBLOCK_BASE, LOW_JUMP_ENTRIES, RST_FIRM_JUMP, RST_LOW_JUMP};
pub use cpc464_os::kernel::FoundCommand;
use cpc464_os::maths::{self, Flags};
use cpc464_os::{Hardware, Regs};

/// Z80 flag bits as found in the low byte of `AF`.
const F_CARRY: u16 = 0x01;
const F_ZERO: u16 = 0x40;
const F_SIGN: u16 = 0x80;

fn flags_of(r: &Regs) -> Flags {
    Flags { carry: r.af & F_CARRY != 0, zero: r.af & F_ZERO != 0, sign: r.af & F_SIGN != 0 }
}
fn a_of(r: &Regs) -> u8 { (r.af >> 8) as u8 }
fn regs_a(a: u8) -> Regs { Regs { af: (a as u16) << 8, ..Regs::default() } }
fn hi(v: u16) -> u8 { (v >> 8) as u8 }
fn lo(v: u16) -> u8 { v as u8 }
fn pair(h: u8, l: u8) -> u16 { (h as u16) << 8 | l as u16 }

/// The three bytes `JUMP RESTORE` writes for the entry at `entry`.
pub fn pristine_entry(entry: u16) -> [u8; 3] {
    let i = ((entry - JUMPBLOCK_BASE) / 3) as usize;
    let JumpEntry { target, .. } = JUMPBLOCK[i];
    if i < LOW_JUMP_ENTRIES {
        [RST_LOW_JUMP, target as u8, (target >> 8) as u8 | 0x80]
    } else {
        [RST_FIRM_JUMP, target as u8, (target >> 8) as u8]
    }
}

// ---- jumpblock addresses used by BASIC ---------------------------------------------
pub const KM_WAIT_CHAR: u16 = 0xBB06;
pub const KM_READ_CHAR: u16 = 0xBB09;
pub const KM_CHAR_RETURN: u16 = 0xBB0C;
pub const KM_SET_EXPAND: u16 = 0xBB0F;
pub const KM_TEST_KEY: u16 = 0xBB1E;
pub const KM_GET_JOYSTICK: u16 = 0xBB24;
pub const KM_SET_TRANSLATE: u16 = 0xBB27;
pub const KM_SET_SHIFT: u16 = 0xBB2D;
pub const KM_SET_CONTROL: u16 = 0xBB33;
pub const KM_SET_REPEAT: u16 = 0xBB39;
pub const KM_SET_DELAY: u16 = 0xBB3F;
pub const KM_ARM_BREAK: u16 = 0xBB45;
pub const KM_DISARM_BREAK: u16 = 0xBB48;
pub const TXT_VDU_ENABLE: u16 = 0xBB54;
pub const TXT_OUTPUT: u16 = 0xBB5A;
pub const TXT_SET_GRAPHIC: u16 = 0xBB63;
pub const TXT_WIN_ENABLE: u16 = 0xBB66;
pub const TXT_GET_WINDOW: u16 = 0xBB69;
pub const TXT_SET_CURSOR: u16 = 0xBB75;
pub const TXT_GET_CURSOR: u16 = 0xBB78;
pub const TXT_CUR_ON: u16 = 0xBB81;
pub const TXT_CUR_OFF: u16 = 0xBB84;
pub const TXT_VALIDATE: u16 = 0xBB87;
pub const TXT_SET_PEN: u16 = 0xBB90;
pub const TXT_SET_PAPER: u16 = 0xBB96;
pub const TXT_GET_MATRIX: u16 = 0xBBA5;
pub const TXT_SET_M_TABLE: u16 = 0xBBAB;
pub const TXT_GET_M_TABLE: u16 = 0xBBAE;
pub const TXT_STR_SELECT: u16 = 0xBBB4;
pub const TXT_SWAP_STREAMS: u16 = 0xBBB7;
pub const GRA_MOVE_ABSOLUTE: u16 = 0xBBC0;
pub const GRA_MOVE_RELATIVE: u16 = 0xBBC3;
pub const GRA_ASK_CURSOR: u16 = 0xBBC6;
pub const GRA_SET_ORIGIN: u16 = 0xBBC9;
pub const GRA_WIN_WIDTH: u16 = 0xBBCF;
pub const GRA_WIN_HEIGHT: u16 = 0xBBD2;
pub const GRA_CLEAR_WINDOW: u16 = 0xBBDB;
pub const GRA_SET_PEN: u16 = 0xBBDE;
pub const GRA_SET_PAPER: u16 = 0xBBE4;
pub const GRA_PLOT_ABSOLUTE: u16 = 0xBBEA;
pub const GRA_PLOT_RELATIVE: u16 = 0xBBED;
pub const GRA_TEST_ABSOLUTE: u16 = 0xBBF0;
pub const GRA_TEST_RELATIVE: u16 = 0xBBF3;
pub const GRA_LINE_ABSOLUTE: u16 = 0xBBF6;
pub const GRA_LINE_RELATIVE: u16 = 0xBBF9;
pub const SCR_SET_MODE: u16 = 0xBC0E;
pub const SCR_SET_INK: u16 = 0xBC32;
pub const SCR_SET_BORDER: u16 = 0xBC38;
pub const SCR_SET_FLASHING: u16 = 0xBC3E;
pub const CAS_SET_SPEED: u16 = 0xBC68;
pub const CAS_NOISY: u16 = 0xBC6B;
pub const CAS_IN_OPEN: u16 = 0xBC77;
pub const CAS_IN_CLOSE: u16 = 0xBC7A;
pub const CAS_IN_ABANDON: u16 = 0xBC7D;
pub const CAS_IN_CHAR: u16 = 0xBC80;
pub const CAS_IN_DIRECT: u16 = 0xBC83;
pub const CAS_RETURN: u16 = 0xBC86;
pub const CAS_TEST_EOF: u16 = 0xBC89;
pub const CAS_OUT_OPEN: u16 = 0xBC8C;
pub const CAS_OUT_CLOSE: u16 = 0xBC8F;
pub const CAS_OUT_ABANDON: u16 = 0xBC92;
pub const CAS_OUT_CHAR: u16 = 0xBC95;
pub const CAS_OUT_DIRECT: u16 = 0xBC98;
pub const CAS_CATALOG: u16 = 0xBC9B;
pub const SOUND_RESET: u16 = 0xBCA7;
pub const SOUND_QUEUE: u16 = 0xBCAA;
pub const SOUND_CHECK: u16 = 0xBCAD;
pub const SOUND_ARM_EVENT: u16 = 0xBCB0;
pub const SOUND_RELEASE: u16 = 0xBCB3;
pub const SOUND_HOLD: u16 = 0xBCB6;
pub const SOUND_CONTINUE: u16 = 0xBCB9;
pub const SOUND_AMPL_ENVELOPE: u16 = 0xBCBC;
pub const SOUND_TONE_ENVELOPE: u16 = 0xBCBF;
pub const KL_ROM_WALK: u16 = 0xBCCB;
pub const KL_FIND_COMMAND: u16 = 0xBCD4;
pub const KL_ADD_TICKER: u16 = 0xBCE9;
pub const KL_DEL_TICKER: u16 = 0xBCEC;
pub const KL_INIT_EVENT: u16 = 0xBCEF;
pub const KL_SYNC_RESET: u16 = 0xBCF5;
pub const KL_NEXT_SYNC: u16 = 0xBCFB;
pub const KL_DO_SYNC: u16 = 0xBCFE;
pub const KL_DONE_SYNC: u16 = 0xBD01;
pub const KL_EVENT_DISABLE: u16 = 0xBD04;
pub const KL_EVENT_ENABLE: u16 = 0xBD07;
pub const KL_TIME_PLEASE: u16 = 0xBD0D;
pub const MC_BOOT_PROGRAM: u16 = 0xBD13;
pub const MC_PRINT_CHAR: u16 = 0xBD2B;
pub const EDIT: u16 = 0xBD3A;
pub const REAL_COPY: u16 = 0xBD3D;
pub const INT_TO_REAL: u16 = 0xBD40;
pub const BIN_TO_REAL: u16 = 0xBD43;
pub const REAL_TO_INT: u16 = 0xBD46;
pub const REAL_TO_BIN: u16 = 0xBD49;
pub const REAL_FIX: u16 = 0xBD4C;
pub const REAL_INT: u16 = 0xBD4F;
pub const REAL_DECIMAL_PREPARE: u16 = 0xBD52;
pub const REAL_SCALE_POW10: u16 = 0xBD55;
pub const REAL_ADD: u16 = 0xBD58;
pub const REAL_REVERSE_SUB: u16 = 0xBD5E;
pub const REAL_MULTIPLY: u16 = 0xBD61;
pub const REAL_DIVIDE: u16 = 0xBD64;
pub const REAL_COMPARE: u16 = 0xBD6A;
pub const REAL_NEGATE: u16 = 0xBD6D;
pub const REAL_SIGN: u16 = 0xBD70;
pub const SET_ANGLE_MODE: u16 = 0xBD73;
pub const REAL_PI: u16 = 0xBD76;
pub const REAL_SQRT: u16 = 0xBD79;
pub const REAL_POWER: u16 = 0xBD7C;
pub const REAL_LN: u16 = 0xBD7F;
pub const REAL_LOG10: u16 = 0xBD82;
pub const REAL_EXP: u16 = 0xBD85;
pub const REAL_SIN: u16 = 0xBD88;
pub const REAL_COS: u16 = 0xBD8B;
pub const REAL_TAN: u16 = 0xBD8E;
pub const REAL_ATN: u16 = 0xBD91;
pub const BIN_TO_REAL_SCALED: u16 = 0xBD94;
pub const RANDOMIZE_RESET: u16 = 0xBD97;
pub const RANDOMIZE_SEED: u16 = 0xBD9A;
pub const RND: u16 = 0xBD9D;
pub const RND_LAST: u16 = 0xBDA0;
pub const INT_ABS_SPLIT: u16 = 0xBDA3;
pub const INT_UNSIGNED_SPLIT: u16 = 0xBDA6;
pub const INT_APPLY_SIGN: u16 = 0xBDA9;
pub const INT_ADD: u16 = 0xBDAC;
pub const INT_REVERSE_SUB: u16 = 0xBDB2;
pub const INT_MULTIPLY: u16 = 0xBDB5;
pub const INT_DIVIDE: u16 = 0xBDB8;
pub const INT_MODULUS: u16 = 0xBDBB;
pub const INT_MULTIPLY_UNSIGNED: u16 = 0xBDBE;
pub const INT_DIVIDE_UNSIGNED: u16 = 0xBDC1;
pub const INT_COMPARE: u16 = 0xBDC4;
pub const INT_NEGATE: u16 = 0xBDC7;
pub const INT_SIGN: u16 = 0xBDCA;

/// A real-number routine's flags as BASIC tests them.
pub type MathsFlags = Flags;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Plumbing
    // ===============================================================================

    /// Has a program changed the jumpblock entry at `entry`?
    pub fn jumpblock_patched(&self, entry: u16) -> bool {
        let want = pristine_entry(entry);
        (0..3).any(|i| self.ram(entry + i) != want[i as usize])
    }

    /// If `entry` is patched, call it with `r` and return the registers it left.
    fn patched(&mut self, entry: u16, r: Regs) -> Option<Regs> {
        if !self.jumpblock_patched(entry) { return None; }
        let mut r = r;
        self.cpc.hw.call(entry, &mut r);
        Some(r)
    }

    // ===============================================================================
    // Keyboard manager
    // ===============================================================================

    /// `KM WAIT CHAR` (`BB06`).
    pub fn km_wait_char(&mut self) -> u8 {
        if let Some(r) = self.patched(KM_WAIT_CHAR, Regs::default()) { return a_of(&r); }
        self.cpc.km_wait_char()
    }
    /// `KM READ CHAR` (`BB09`): `None` = carry clear (no character).
    pub fn km_read_char(&mut self) -> Option<u8> {
        if let Some(r) = self.patched(KM_READ_CHAR, Regs::default()) {
            return if r.carry() { Some(a_of(&r)) } else { None };
        }
        self.cpc.km_read_char()
    }
    /// `KM CHAR RETURN` (`BB0C`).
    pub fn km_char_return(&mut self, ch: u8) {
        if self.patched(KM_CHAR_RETURN, regs_a(ch)).is_some() { return; }
        self.cpc.km_char_return(ch)
    }
    /// `KM SET EXPAND` (`BB0F`): `B` = token, `C` = length, `HL` = text.  Carry = success.
    pub fn km_set_expand(&mut self, token: u8, len: u8, text: u16) -> bool {
        if let Some(r) = self.patched(KM_SET_EXPAND, Regs { bc: pair(token, len), hl: text, ..Regs::default() }) {
            return r.carry();
        }
        let bytes: Vec<u8> = (0..len as u16).map(|i| self.rd(text.wrapping_add(i))).collect();
        self.cpc.km_set_expand(token, &bytes)
    }
    /// `KM TEST KEY` (`BB1E`): `(pressed, C = shift/control state)`.
    pub fn km_test_key(&mut self, key: u8) -> (bool, u8) {
        if let Some(r) = self.patched(KM_TEST_KEY, regs_a(key)) {
            return (r.af & F_ZERO == 0, lo(r.bc));
        }
        self.cpc.km_test_key(key)
    }
    /// `KM GET JOYSTICK` (`BB24`): `(H = joystick 0, L = joystick 1)`.
    pub fn km_get_joystick(&mut self) -> (u8, u8) {
        if let Some(r) = self.patched(KM_GET_JOYSTICK, Regs::default()) { return (hi(r.hl), lo(r.hl)); }
        self.cpc.km_get_joystick()
    }
    /// One of `KM SET TRANSLATE` / `SHIFT` / `CONTROL` (`BB27`/`BB2D`/`BB33`): `A` = key,
    /// `B` = code.  BASIC's `KEY DEF` reaches them through a table of entry addresses.
    pub fn km_set_table_entry(&mut self, entry: u16, key: u8, code: u8) {
        let mut r = regs_a(key);
        r.bc = pair(code, 0);
        if self.patched(entry, r).is_some() { return; }
        match entry {
            KM_SET_TRANSLATE => self.cpc.km_set_translate(key, code),
            KM_SET_SHIFT => self.cpc.km_set_shift(key, code),
            KM_SET_CONTROL => self.cpc.km_set_control(key, code),
            _ => unreachable!("not a key table entry: {entry:04X}"),
        }
    }
    /// `KM SET REPEAT` (`BB39`): `A` = key, `B` = repeat flag.
    pub fn km_set_repeat(&mut self, key: u8, repeat: u8) {
        let mut r = regs_a(key);
        r.bc = pair(repeat, 0);
        if self.patched(KM_SET_REPEAT, r).is_some() { return; }
        self.cpc.km_set_repeat(key, repeat)
    }
    /// `KM SET DELAY` (`BB3F`): `H` = start delay, `L` = repeat period.
    pub fn km_set_delay(&mut self, delay: u8, period: u8) {
        if self.patched(KM_SET_DELAY, Regs { hl: pair(delay, period), ..Regs::default() }).is_some() { return; }
        self.cpc.km_set_delay(delay, period)
    }
    /// `KM ARM BREAK` (`BB45`): `DE` = event routine, `C` = its ROM select.
    pub fn km_arm_break(&mut self, routine: u16, rom: u8) {
        if self.patched(KM_ARM_BREAK, Regs { de: routine, bc: rom as u16, ..Regs::default() }).is_some() { return; }
        self.cpc.km_arm_break(routine, rom)
    }
    /// `KM DISARM BREAK` (`BB48`).
    pub fn km_disarm_break(&mut self) {
        if self.patched(KM_DISARM_BREAK, Regs::default()).is_some() { return; }
        self.cpc.km_disarm_break()
    }

    // ===============================================================================
    // Text VDU
    // ===============================================================================

    /// `TXT VDU ENABLE` (`BB54`).
    pub fn txt_vdu_enable(&mut self) {
        if self.patched(TXT_VDU_ENABLE, Regs::default()).is_some() { return; }
        self.cpc.txt_vdu_enable()
    }
    /// `TXT OUTPUT` (`BB5A`).
    pub fn txt_output(&mut self, ch: u8) {
        if self.patched(TXT_OUTPUT, regs_a(ch)).is_some() { return; }
        self.cpc.txt_output(ch)
    }
    /// `TXT SET GRAPHIC` (`BB63`).
    pub fn txt_set_graphic(&mut self, flag: u8) {
        if self.patched(TXT_SET_GRAPHIC, regs_a(flag)).is_some() { return; }
        self.cpc.txt_set_graphic(flag)
    }
    /// `TXT WIN ENABLE` (`BB66`): `H`,`D` = columns, `L`,`E` = rows (0-based, either order).
    pub fn txt_win_enable(&mut self, col1: u8, col2: u8, row1: u8, row2: u8) {
        if self.patched(TXT_WIN_ENABLE, Regs { hl: pair(col1, row1), de: pair(col2, row2), ..Regs::default() }).is_some() { return; }
        self.cpc.txt_win_enable(col1, col2, row1, row2)
    }
    /// `TXT GET WINDOW` (`BB69`): `(H left, L top, D right, E bottom)`.
    pub fn txt_get_window(&mut self) -> (u8, u8, u8, u8) {
        if let Some(r) = self.patched(TXT_GET_WINDOW, Regs::default()) {
            return (hi(r.hl), lo(r.hl), hi(r.de), lo(r.de));
        }
        let ((l, t), (rt, b), _) = self.cpc.txt_get_window();
        (l, t, rt, b)
    }
    /// `TXT SET CURSOR` (`BB75`): `H` = column, `L` = row (1-based, window relative).
    pub fn txt_set_cursor(&mut self, col: u8, row: u8) {
        if self.patched(TXT_SET_CURSOR, Regs { hl: pair(col, row), ..Regs::default() }).is_some() { return; }
        self.cpc.txt_set_cursor(col, row)
    }
    /// `TXT GET CURSOR` (`BB78`): `(H column, L row)`.
    pub fn txt_get_cursor(&mut self) -> (u8, u8) {
        if let Some(r) = self.patched(TXT_GET_CURSOR, Regs::default()) { return (hi(r.hl), lo(r.hl)); }
        let (c, r, _) = self.cpc.txt_get_cursor();
        (c, r)
    }
    /// `TXT CUR ON` (`BB81`).
    pub fn txt_cur_on(&mut self) {
        if self.patched(TXT_CUR_ON, Regs::default()).is_some() { return; }
        self.cpc.txt_cur_on()
    }
    /// `TXT CUR OFF` (`BB84`).
    pub fn txt_cur_off(&mut self) {
        if self.patched(TXT_CUR_OFF, Regs::default()).is_some() { return; }
        self.cpc.txt_cur_off()
    }
    /// `TXT VALIDATE` (`BB87`): the position `(H, L)` after wrapping into the window.
    pub fn txt_validate(&mut self, col: u8, row: u8) -> (u8, u8) {
        if let Some(r) = self.patched(TXT_VALIDATE, Regs { hl: pair(col, row), ..Regs::default() }) {
            return (hi(r.hl), lo(r.hl));
        }
        let (c, r, _) = self.cpc.txt_validate(col, row);
        (c, r)
    }
    /// `TXT SET PEN` (`BB90`).
    pub fn txt_set_pen(&mut self, pen: u8) {
        if self.patched(TXT_SET_PEN, regs_a(pen)).is_some() { return; }
        self.cpc.txt_set_pen(pen)
    }
    /// `TXT SET PAPER` (`BB96`).
    pub fn txt_set_paper(&mut self, paper: u8) {
        if self.patched(TXT_SET_PAPER, regs_a(paper)).is_some() { return; }
        self.cpc.txt_set_paper(paper)
    }
    /// `TXT GET MATRIX` (`BBA5`): `(HL = matrix address, carry = user defined)`.
    pub fn txt_get_matrix(&mut self, ch: u8) -> (u16, bool) {
        if let Some(r) = self.patched(TXT_GET_MATRIX, regs_a(ch)) { return (r.hl, r.carry()); }
        let m = self.cpc.txt_get_matrix(ch);
        (m.addr, m.user_defined)
    }
    /// `TXT SET M TABLE` (`BBAB`): `D` = 0 to install / non-zero to remove, `E` = first
    /// character, `HL` = table.
    pub fn txt_set_m_table(&mut self, de: u16, table: u16) {
        if self.patched(TXT_SET_M_TABLE, Regs { de, hl: table, ..Regs::default() }).is_some() { return; }
        self.cpc.txt_set_m_table(hi(de), lo(de), table)
    }
    /// `TXT GET M TABLE` (`BBAE`): `Some((A = first character, HL = table))` if a user table
    /// is installed (carry set).
    pub fn txt_get_m_table(&mut self) -> Option<(u8, u16)> {
        if let Some(r) = self.patched(TXT_GET_M_TABLE, Regs::default()) {
            return if r.carry() { Some((a_of(&r), r.hl)) } else { None };
        }
        use cpc464_os::sysvars::{TXT_MATRIX_FIRST, TXT_MATRIX_FLAG, TXT_MATRIX_TABLE};
        if self.ram(TXT_MATRIX_FLAG) & 1 != 0 {
            Some((self.ram(TXT_MATRIX_FIRST), self.ram16(TXT_MATRIX_TABLE)))
        } else {
            None
        }
    }
    /// `TXT STR SELECT` (`BBB4`): returns the previous stream.
    pub fn txt_str_select(&mut self, n: u8) -> u8 {
        if let Some(r) = self.patched(TXT_STR_SELECT, regs_a(n)) { return a_of(&r); }
        self.cpc.txt_str_select(n)
    }
    /// `TXT SWAP STREAMS` (`BBB7`): `B`, `C` = the two streams.
    pub fn txt_swap_streams(&mut self, b: u8, c: u8) {
        if self.patched(TXT_SWAP_STREAMS, Regs { bc: pair(b, c), ..Regs::default() }).is_some() { return; }
        self.cpc.txt_swap_streams(b, c)
    }

    // ===============================================================================
    // Graphics VDU
    // ===============================================================================

    /// One of the graphics routines that take `DE` = x and `HL` = y and return nothing:
    /// `GRA MOVE`/`PLOT`/`LINE ABSOLUTE`/`RELATIVE`, `GRA SET ORIGIN`.  BASIC picks the
    /// entry from a table, so it is passed in.
    pub fn gra_xy(&mut self, entry: u16, x: u16, y: u16) {
        if self.patched(entry, Regs { de: x, hl: y, ..Regs::default() }).is_some() { return; }
        let (x, y) = (x as i16, y as i16);
        match entry {
            GRA_MOVE_ABSOLUTE => self.cpc.gra_move_absolute(x, y),
            GRA_MOVE_RELATIVE => self.cpc.gra_move_relative(x, y),
            GRA_SET_ORIGIN => self.cpc.gra_set_origin(x, y),
            GRA_PLOT_ABSOLUTE => self.cpc.gra_plot_absolute(x, y),
            GRA_PLOT_RELATIVE => self.cpc.gra_plot_relative(x, y),
            GRA_LINE_ABSOLUTE => self.cpc.gra_line_absolute(x, y),
            GRA_LINE_RELATIVE => self.cpc.gra_line_relative(x, y),
            _ => unreachable!("not a graphics x/y entry: {entry:04X}"),
        }
    }
    /// `GRA TEST ABSOLUTE` / `GRA TEST RELATIVE` (`BBF0`/`BBF3`): the pen at the point.
    pub fn gra_test(&mut self, entry: u16, x: u16, y: u16) -> u8 {
        if let Some(r) = self.patched(entry, Regs { de: x, hl: y, ..Regs::default() }) { return a_of(&r); }
        let (x, y) = (x as i16, y as i16);
        match entry {
            GRA_TEST_ABSOLUTE => self.cpc.gra_test_absolute(x, y),
            GRA_TEST_RELATIVE => self.cpc.gra_test_relative(x, y),
            _ => unreachable!("not a graphics test entry: {entry:04X}"),
        }
    }
    /// `GRA ASK CURSOR` (`BBC6`): `(DE = x, HL = y)`.
    pub fn gra_ask_cursor(&mut self) -> (u16, u16) {
        if let Some(r) = self.patched(GRA_ASK_CURSOR, Regs::default()) { return (r.de, r.hl); }
        let (x, y) = self.cpc.gra_ask_cursor();
        (x as u16, y as u16)
    }
    /// `GRA WIN WIDTH` (`BBCF`): `DE`, `HL` = the two x edges.
    pub fn gra_win_width(&mut self, x1: u16, x2: u16) {
        if self.patched(GRA_WIN_WIDTH, Regs { de: x1, hl: x2, ..Regs::default() }).is_some() { return; }
        self.cpc.gra_win_width(x1 as i16, x2 as i16)
    }
    /// `GRA WIN HEIGHT` (`BBD2`): `DE`, `HL` = the two y edges.
    pub fn gra_win_height(&mut self, y1: u16, y2: u16) {
        if self.patched(GRA_WIN_HEIGHT, Regs { de: y1, hl: y2, ..Regs::default() }).is_some() { return; }
        self.cpc.gra_win_height(y1 as i16, y2 as i16)
    }
    /// `GRA CLEAR WINDOW` (`BBDB`).
    pub fn gra_clear_window(&mut self) {
        if self.patched(GRA_CLEAR_WINDOW, Regs::default()).is_some() { return; }
        self.cpc.gra_clear_window()
    }
    /// `GRA SET PEN` (`BBDE`).
    pub fn gra_set_pen(&mut self, pen: u8) {
        if self.patched(GRA_SET_PEN, regs_a(pen)).is_some() { return; }
        self.cpc.gra_set_pen(pen)
    }
    /// `GRA SET PAPER` (`BBE4`).
    pub fn gra_set_paper(&mut self, paper: u8) {
        if self.patched(GRA_SET_PAPER, regs_a(paper)).is_some() { return; }
        self.cpc.gra_set_paper(paper)
    }

    // ===============================================================================
    // Screen pack
    // ===============================================================================

    /// `SCR SET MODE` (`BC0E`).
    pub fn scr_set_mode(&mut self, mode: u8) {
        if self.patched(SCR_SET_MODE, regs_a(mode)).is_some() { return; }
        self.cpc.scr_set_mode(mode)
    }
    /// `SCR SET INK` (`BC32`): `A` = pen, `B`/`C` = colours.
    pub fn scr_set_ink(&mut self, pen: u8, c1: u8, c2: u8) {
        let mut r = regs_a(pen);
        r.bc = pair(c1, c2);
        if self.patched(SCR_SET_INK, r).is_some() { return; }
        self.cpc.scr_set_ink(pen, c1, c2)
    }
    /// `SCR SET BORDER` (`BC38`): `B`/`C` = colours.
    pub fn scr_set_border(&mut self, c1: u8, c2: u8) {
        if self.patched(SCR_SET_BORDER, Regs { bc: pair(c1, c2), ..Regs::default() }).is_some() { return; }
        self.cpc.scr_set_border(c1, c2)
    }
    /// `SCR SET FLASHING` (`BC3E`): `H` = first period, `L` = second period.
    pub fn scr_set_flashing(&mut self, first: u8, second: u8) {
        if self.patched(SCR_SET_FLASHING, Regs { hl: pair(first, second), ..Regs::default() }).is_some() { return; }
        self.cpc.scr_set_flashing(first, second)
    }

    // ===============================================================================
    // Cassette manager
    // ===============================================================================

    /// `CAS SET SPEED` (`BC68`): `HL` = half-zero length, `A` = precompensation.
    pub fn cas_set_speed(&mut self, half_zero: u16, precomp: u8) {
        let mut r = regs_a(precomp);
        r.hl = half_zero;
        if self.patched(CAS_SET_SPEED, r).is_some() { return; }
        self.cpc.cas_set_speed(half_zero, precomp)
    }
    /// `CAS NOISY` (`BC6B`).
    pub fn cas_noisy(&mut self, flag: u8) {
        if self.patched(CAS_NOISY, regs_a(flag)).is_some() { return; }
        self.cpc.cas_noisy(flag)
    }
    fn cas_err(r: &Regs) -> CasError { CasError { a: a_of(r), zero: r.af & F_ZERO != 0 } }
    /// `CAS IN OPEN` (`BC77`): `B` = name length, `HL` = name, `DE` = 2K buffer.
    pub fn cas_in_open(&mut self, name: u16, len: u8, buffer: u16) -> Result<CasOpen, CasError> {
        if let Some(r) = self.patched(CAS_IN_OPEN, Regs { bc: pair(len, 0), hl: name, de: buffer, ..Regs::default() }) {
            return if r.carry() {
                Ok(CasOpen { header: r.hl, load_address: r.de, logical_length: r.bc, file_type: a_of(&r) })
            } else { Err(Self::cas_err(&r)) };
        }
        self.cpc.cas_in_open(name, len, buffer)
    }
    /// `CAS IN CLOSE` (`BC7A`).
    pub fn cas_in_close(&mut self) {
        if self.patched(CAS_IN_CLOSE, Regs::default()).is_some() { return; }
        self.cpc.cas_in_close();
    }
    /// `CAS IN ABANDON` (`BC7D`).
    pub fn cas_in_abandon(&mut self) {
        if self.patched(CAS_IN_ABANDON, Regs::default()).is_some() { return; }
        self.cpc.cas_in_abandon();
    }
    /// `CAS IN CHAR` (`BC80`).
    pub fn cas_in_char(&mut self) -> Result<u8, CasError> {
        if let Some(r) = self.patched(CAS_IN_CHAR, Regs::default()) {
            return if r.carry() { Ok(a_of(&r)) } else { Err(Self::cas_err(&r)) };
        }
        self.cpc.cas_in_char()
    }
    /// `CAS IN DIRECT` (`BC83`): `HL` = destination; returns the entry address.
    pub fn cas_in_direct(&mut self, dest: u16) -> Result<u16, CasError> {
        if let Some(r) = self.patched(CAS_IN_DIRECT, Regs { hl: dest, ..Regs::default() }) {
            return if r.carry() { Ok(r.hl) } else { Err(Self::cas_err(&r)) };
        }
        self.cpc.cas_in_direct(dest)
    }
    /// `CAS RETURN` (`BC86`).
    pub fn cas_return(&mut self) {
        if self.patched(CAS_RETURN, Regs::default()).is_some() { return; }
        self.cpc.cas_return()
    }
    /// `CAS TEST EOF` (`BC89`).
    pub fn cas_test_eof(&mut self) -> Result<u8, CasError> {
        if let Some(r) = self.patched(CAS_TEST_EOF, Regs::default()) {
            return if r.carry() { Ok(a_of(&r)) } else { Err(Self::cas_err(&r)) };
        }
        self.cpc.cas_test_eof()
    }
    /// `CAS OUT OPEN` (`BC8C`): returns the header address.
    pub fn cas_out_open(&mut self, name: u16, len: u8, buffer: u16) -> Result<u16, CasError> {
        if let Some(r) = self.patched(CAS_OUT_OPEN, Regs { bc: pair(len, 0), hl: name, de: buffer, ..Regs::default() }) {
            return if r.carry() { Ok(r.hl) } else { Err(Self::cas_err(&r)) };
        }
        self.cpc.cas_out_open(name, len, buffer)
    }
    /// `CAS OUT CLOSE` (`BC8F`).
    pub fn cas_out_close(&mut self) -> Result<(), CasError> {
        if let Some(r) = self.patched(CAS_OUT_CLOSE, Regs::default()) {
            return if r.carry() { Ok(()) } else { Err(Self::cas_err(&r)) };
        }
        self.cpc.cas_out_close().map(|_| ())
    }
    /// `CAS OUT ABANDON` (`BC92`).
    pub fn cas_out_abandon(&mut self) {
        if self.patched(CAS_OUT_ABANDON, Regs::default()).is_some() { return; }
        self.cpc.cas_out_abandon();
    }
    /// `CAS OUT CHAR` (`BC95`).
    pub fn cas_out_char(&mut self, ch: u8) -> Result<(), CasError> {
        if let Some(r) = self.patched(CAS_OUT_CHAR, regs_a(ch)) {
            return if r.carry() { Ok(()) } else { Err(Self::cas_err(&r)) };
        }
        self.cpc.cas_out_char(ch)
    }
    /// `CAS OUT DIRECT` (`BC98`): `HL` = start, `DE` = length, `BC` = entry, `A` = type.
    pub fn cas_out_direct(&mut self, start: u16, length: u16, entry: u16, file_type: u8) -> Result<(), CasError> {
        let mut r = regs_a(file_type);
        r.hl = start; r.de = length; r.bc = entry;
        if let Some(r) = self.patched(CAS_OUT_DIRECT, r) {
            return if r.carry() { Ok(()) } else { Err(Self::cas_err(&r)) };
        }
        self.cpc.cas_out_direct(start, length, entry, file_type)
    }
    /// `CAS CATALOG` (`BC9B`): `DE` = 2K buffer.
    pub fn cas_catalog(&mut self, buffer: u16) -> Result<(), CasError> {
        if let Some(r) = self.patched(CAS_CATALOG, Regs { de: buffer, ..Regs::default() }) {
            return if r.carry() { Ok(()) } else { Err(Self::cas_err(&r)) };
        }
        self.cpc.cas_catalog(buffer).map(|_| ())
    }

    // ===============================================================================
    // Sound manager
    // ===============================================================================

    /// `SOUND RESET` (`BCA7`).
    pub fn sound_reset(&mut self) {
        if self.patched(SOUND_RESET, Regs::default()).is_some() { return; }
        self.cpc.sound_reset()
    }
    /// `SOUND QUEUE` (`BCAA`): `HL` = the 9-byte block.  Carry = queued.
    pub fn sound_queue(&mut self, block: u16) -> bool {
        if let Some(r) = self.patched(SOUND_QUEUE, Regs { hl: block, ..Regs::default() }) { return r.carry(); }
        let mut p = [0u8; 9];
        for (i, b) in p.iter_mut().enumerate() { *b = self.rd(block + i as u16); }
        self.cpc.sound_queue(&p)
    }
    /// `SOUND CHECK` (`BCAD`).
    pub fn sound_check(&mut self, channel: u8) -> u8 {
        if let Some(r) = self.patched(SOUND_CHECK, regs_a(channel)) { return a_of(&r); }
        self.cpc.sound_check(channel)
    }
    /// `SOUND ARM EVENT` (`BCB0`): `A` = channel, `HL` = event block.
    pub fn sound_arm_event(&mut self, channel: u8, event: u16) {
        let mut r = regs_a(channel);
        r.hl = event;
        if self.patched(SOUND_ARM_EVENT, r).is_some() { return; }
        self.cpc.sound_arm_event(channel, event)
    }
    /// `SOUND RELEASE` (`BCB3`).
    pub fn sound_release(&mut self, channels: u8) {
        if self.patched(SOUND_RELEASE, regs_a(channels)).is_some() { return; }
        self.cpc.sound_release(channels)
    }
    /// `SOUND HOLD` (`BCB6`): carry = sound was active.
    pub fn sound_hold(&mut self) -> bool {
        if let Some(r) = self.patched(SOUND_HOLD, Regs::default()) { return r.carry(); }
        self.cpc.sound_hold()
    }
    /// `SOUND CONTINUE` (`BCB9`).
    pub fn sound_continue(&mut self) {
        if self.patched(SOUND_CONTINUE, Regs::default()).is_some() { return; }
        self.cpc.sound_continue()
    }
    /// `SOUND AMPL ENVELOPE` / `SOUND TONE ENVELOPE` (`BCBC`/`BCBF`): `A` = envelope
    /// number, `HL` = the 16-byte block.
    pub fn sound_envelope(&mut self, entry: u16, n: u8, block: u16) {
        let mut r = regs_a(n);
        r.hl = block;
        if self.patched(entry, r).is_some() { return; }
        let mut d = [0u8; 16];
        for (i, b) in d.iter_mut().enumerate() { *b = self.rd(block + i as u16); }
        if entry == SOUND_AMPL_ENVELOPE { self.cpc.sound_ampl_envelope(n, &d); } else { self.cpc.sound_tone_envelope(n, &d); }
    }

    // ===============================================================================
    // Kernel
    // ===============================================================================

    /// `KL ROM WALK` (`BCCB`): `DE`/`HL` = free memory, updated by background ROMs.
    pub fn kl_rom_walk(&mut self, de: u16, hl: u16) -> (u16, u16) {
        if let Some(r) = self.patched(KL_ROM_WALK, Regs { de, hl, ..Regs::default() }) { return (r.de, r.hl); }
        self.cpc.kl_rom_walk(de, hl)
    }
    /// `KL FIND COMMAND` (`BCD4`): `HL` = name (last character with bit 7 set).
    pub fn kl_find_command(&mut self, name: u16) -> FoundCommand {
        if let Some(r) = self.patched(KL_FIND_COMMAND, Regs { hl: name, ..Regs::default() }) {
            return if r.carry() { FoundCommand::Extension(r.hl, lo(r.bc)) } else { FoundCommand::NotFound };
        }
        // The firmware copies 16 bytes from `HL` whatever the name's length.
        let bytes: Vec<u8> = (0..16u16).map(|i| self.ram(name.wrapping_add(i))).collect();
        self.cpc.kl_find_command(&bytes)
    }
    /// `KL ADD TICKER` (`BCE9`): `HL` = block, `DE` = count, `BC` = recharge.
    pub fn kl_add_ticker(&mut self, block: u16, count: u16, recharge: u16) {
        if self.patched(KL_ADD_TICKER, Regs { hl: block, de: count, bc: recharge, ..Regs::default() }).is_some() { return; }
        self.cpc.kl_add_ticker(block, count, recharge)
    }
    /// `KL DEL TICKER` (`BCEC`): `Some(DE = remaining count)` if it was ticking (carry).
    pub fn kl_del_ticker(&mut self, block: u16) -> Option<u16> {
        if let Some(r) = self.patched(KL_DEL_TICKER, Regs { hl: block, ..Regs::default() }) {
            return if r.carry() { Some(r.de) } else { None };
        }
        self.cpc.kl_del_ticker(block)
    }
    /// `KL INIT EVENT` (`BCEF`): `HL` = block, `B` = class, `C` = ROM, `DE` = routine.
    pub fn kl_init_event(&mut self, block: u16, class: u8, rom: u8, routine: u16) {
        if self.patched(KL_INIT_EVENT, Regs { hl: block, bc: pair(class, rom), de: routine, ..Regs::default() }).is_some() { return; }
        self.cpc.kl_init_event(block, routine, rom, class)
    }
    /// `KL SYNC RESET` (`BCF5`).
    pub fn kl_sync_reset(&mut self) {
        if self.patched(KL_SYNC_RESET, Regs::default()).is_some() { return; }
        self.cpc.kl_sync_reset()
    }
    /// `KL NEXT SYNC` (`BCFB`): `Some((HL = event, A = previous priority))`.
    pub fn kl_next_sync(&mut self) -> Option<(u16, u8)> {
        if let Some(r) = self.patched(KL_NEXT_SYNC, Regs::default()) {
            return if r.carry() { Some((r.hl, a_of(&r))) } else { None };
        }
        self.cpc.kl_next_sync()
    }
    /// `KL DONE SYNC` (`BD01`): `A` = priority, `HL` = event.
    pub fn kl_done_sync(&mut self, event: u16, priority: u8) {
        let mut r = regs_a(priority);
        r.hl = event;
        if self.patched(KL_DONE_SYNC, r).is_some() { return; }
        self.cpc.kl_done_sync(event, priority)
    }
    /// `KL EVENT DISABLE` (`BD04`).
    pub fn kl_event_disable(&mut self) {
        if self.patched(KL_EVENT_DISABLE, Regs::default()).is_some() { return; }
        self.cpc.kl_event_disable()
    }
    /// `KL EVENT ENABLE` (`BD07`).
    pub fn kl_event_enable(&mut self) {
        if self.patched(KL_EVENT_ENABLE, Regs::default()).is_some() { return; }
        self.cpc.kl_event_enable()
    }
    /// `KL TIME PLEASE` (`BD0D`): `(DE high, HL low)`.
    pub fn kl_time_please(&mut self) -> (u16, u16) {
        if let Some(r) = self.patched(KL_TIME_PLEASE, Regs::default()) { return (r.de, r.hl); }
        self.cpc.kl_time_please()
    }
    /// `KL POLL SYNCHRONOUS` (`B921`, RAM-resident kernel): is a synchronous event of
    /// higher priority than the current one waiting?
    pub fn kl_poll_synchronous(&self) -> bool { self.cpc.kl_poll_synchronous() }

    // ===============================================================================
    // Machine pack
    // ===============================================================================

    /// `MC BOOT PROGRAM` (`BD13`): `HL` = loader.  Never returns on the real machine; here
    /// the firmware's start target is reported to the host as [`Exit::Stop`] after the
    /// call (the Rust port cannot enter a machine-code program).
    pub fn mc_boot_program(&mut self, loader: u16) -> Res<()> {
        if self.patched(MC_BOOT_PROGRAM, Regs { hl: loader, ..Regs::default() }).is_some() { return Err(Exit::Stop); }
        let target = self.cpc.mc_boot_program(loader);
        self.boot_target = Some(target);
        Err(Exit::Stop)
    }
    /// `MC PRINT CHAR` (`BD2B`): carry = sent.
    pub fn mc_print_char(&mut self, ch: u8) -> bool {
        if let Some(r) = self.patched(MC_PRINT_CHAR, regs_a(ch)) { return r.carry(); }
        self.cpc.mc_print_char(ch)
    }

    // ===============================================================================
    // Line editor
    // ===============================================================================

    /// `EDIT` (`BD3A`): edit the zero-terminated line at `buffer`.  `true` (carry) = ENTER,
    /// `false` = ESC.
    ///
    /// If the host installed [`Basic::line_input`], it replaces the editor: the text it
    /// returns is put in the buffer as if typed.
    pub fn edit_line(&mut self, buffer: u16) -> Res<bool> {
        if self.line_input.is_some() {
            let mut cur = vec![];
            let mut a = buffer;
            while self.ram(a) != 0 { cur.push(self.ram(a)); a = a.wrapping_add(1); }
            let mut answer = (self.line_input.as_mut().unwrap())(&cur);
            while let LineInput::Ticks(n) = answer {
                for _ in 0..n { self.cpc.interrupt(); }
                answer = (self.line_input.as_mut().unwrap())(&cur);
            }
            return match answer {
                LineInput::Line(text) => {
                    for (i, b) in text.iter().take(255).enumerate() { self.wr(buffer + i as u16, *b); }
                    let n = text.len().min(255) as u16;
                    self.wr(buffer + n, 0);
                    Ok(true)
                }
                LineInput::Escape => Ok(false),
                LineInput::Stop => Err(Exit::Stop),
                LineInput::Ticks(_) => unreachable!(),
            };
        }
        if let Some(r) = self.patched(EDIT, Regs { hl: buffer, ..Regs::default() }) { return Ok(r.carry()); }
        Ok(self.cpc.km_edit_line(buffer).carry)
    }

    // ===============================================================================
    // Arithmetic package
    // ===============================================================================

    /// `REAL COPY` (`BD3D`): `HL` = destination, `DE` = source.
    pub fn real_copy(&mut self, dest: u16, src: u16) {
        if self.patched(REAL_COPY, Regs { hl: dest, de: src, ..Regs::default() }).is_some() { return; }
        self.cpc.real_copy(dest, src);
    }
    /// `INT TO REAL` (`BD40`): `HL` = magnitude, `DE` = destination, `A` bit 7 = sign.
    pub fn int_to_real(&mut self, value: u16, dest: u16, sign: u8) {
        let mut r = regs_a(sign);
        r.hl = value; r.de = dest;
        if self.patched(INT_TO_REAL, r).is_some() { return; }
        self.cpc.int_to_real(value, dest, sign);
    }
    /// `BIN TO REAL` (`BD43`): `HL` = 4-byte integer (in place), `A` bit 7 = sign.
    pub fn bin_to_real(&mut self, addr: u16, sign: u8) {
        let mut r = regs_a(sign);
        r.hl = addr;
        if self.patched(BIN_TO_REAL, r).is_some() { return; }
        self.cpc.bin_to_real(addr, sign);
    }
    /// `BIN TO REAL` scaled by 2^-8 (`BD94`); `C` = the guard byte below the 4 bytes.
    pub fn bin_to_real_scaled(&mut self, addr: u16, sign: u8, guard: u8) {
        let mut r = regs_a(sign);
        r.hl = addr;
        r.bc = guard as u16;
        if self.patched(BIN_TO_REAL_SCALED, r).is_some() { return; }
        self.cpc.bin_to_real_a8_guard(addr, sign, guard);
    }
    /// `REAL TO INT` (`BD46`): `(HL = magnitude, flags)`; carry = fits, sign = negative.
    pub fn real_to_int(&mut self, addr: u16) -> (u16, Flags) {
        if let Some(r) = self.patched(REAL_TO_INT, Regs { hl: addr, ..Regs::default() }) { return (r.hl, flags_of(&r)); }
        self.cpc.real_to_int(addr)
    }
    /// `REAL TO BIN` / `REAL FIX` / `REAL INT` (`BD49`/`BD4C`/`BD4F`): convert in place
    /// to a 4-byte integer.  Returns `(B = sign, C = significant bytes, flags)`.
    pub fn real_to_binary(&mut self, entry: u16, addr: u16) -> (u8, u8, Flags) {
        if let Some(r) = self.patched(entry, Regs { hl: addr, ..Regs::default() }) {
            return (hi(r.bc), lo(r.bc), flags_of(&r));
        }
        let (fx, f) = match entry {
            REAL_TO_BIN => self.cpc.real_to_bin(addr),
            REAL_FIX => self.cpc.real_fix(addr),
            REAL_INT => self.cpc.real_int(addr),
            _ => unreachable!("not a real-to-binary entry: {entry:04X}"),
        };
        (fx.b, fx.c, f)
    }
    /// `REAL DECIMAL PREPARE` (`BD52`): `(HL, B, C, D, E)` - see the firmware port.
    pub fn real_decimal_prepare(&mut self, addr: u16) -> (u16, u8, u8, u8, u8) {
        if let Some(r) = self.patched(REAL_DECIMAL_PREPARE, Regs { hl: addr, ..Regs::default() }) {
            return (r.hl, hi(r.bc), lo(r.bc), hi(r.de), lo(r.de));
        }
        let p = self.cpc.real_decimal_prepare(addr);
        (p.hl, p.b, p.c, p.d, p.e)
    }
    /// `REAL SCALE POW10` (`BD55`): multiply by 10^`a`.  Carry = ok.
    pub fn real_pow10(&mut self, addr: u16, a: u8) -> bool {
        let mut r = regs_a(a);
        r.hl = addr;
        if let Some(r) = self.patched(REAL_SCALE_POW10, r) { return r.carry(); }
        self.cpc.real_pow10(addr, a)
    }
    /// Binary real operations `HL = HL op DE` (`BD58` add, `BD5E` reverse subtract,
    /// `BD61` multiply, `BD7C` power).  Carry = ok.
    pub fn real_op(&mut self, entry: u16, hl: u16, de: u16) -> bool {
        if let Some(r) = self.patched(entry, Regs { hl, de, ..Regs::default() }) { return r.carry(); }
        match entry {
            REAL_ADD => self.cpc.real_add(hl, de),
            REAL_REVERSE_SUB => self.cpc.real_sub_reverse(hl, de),
            REAL_MULTIPLY => self.cpc.real_mul(hl, de),
            REAL_POWER => self.cpc.real_power(hl, de),
            _ => unreachable!("not a binary real entry: {entry:04X}"),
        }
    }
    /// `REAL POWER` (`BD7C`): `HL` = `HL` ^ `DE`.  As for [`Self::real_function`], the
    /// failure flags are reconstructed: a zero base (to a negative power) is "zero", an
    /// overflow (result: the largest number) "sign", a negative base with a non-integer
    /// exponent neither.
    pub fn real_power(&mut self, hl: u16, de: u16) -> Flags {
        if let Some(r) = self.patched(REAL_POWER, Regs { hl, de, ..Regs::default() }) { return flags_of(&r); }
        let base_zero = self.ram(hl + 4) == 0;
        if self.cpc.real_power(hl, de) { return Flags { carry: true, zero: false, sign: false }; }
        if base_zero { return Flags { carry: false, zero: true, sign: false }; }
        if self.is_largest_real(hl) { return Flags { carry: false, zero: false, sign: true }; }
        Flags { carry: false, zero: false, sign: false }
    }
    /// `REAL DIVIDE` (`BD64`): carry = ok, zero = division by zero.
    pub fn real_divide(&mut self, hl: u16, de: u16) -> Flags {
        if let Some(r) = self.patched(REAL_DIVIDE, Regs { hl, de, ..Regs::default() }) { return flags_of(&r); }
        self.cpc.real_div(hl, de)
    }
    /// `REAL COMPARE` (`BD6A`): `A` = `FF`/0/1.
    pub fn real_compare(&mut self, hl: u16, de: u16) -> u8 {
        if let Some(r) = self.patched(REAL_COMPARE, Regs { hl, de, ..Regs::default() }) { return a_of(&r); }
        self.cpc.real_compare(hl, de).0
    }
    /// `REAL NEGATE` (`BD6D`).
    pub fn real_negate(&mut self, addr: u16) {
        if self.patched(REAL_NEGATE, Regs { hl: addr, ..Regs::default() }).is_some() { return; }
        self.cpc.real_negate(addr)
    }
    /// `REAL SIGN` (`BD70`): `A` = `FF`/0/1 and the flags.
    pub fn real_sign(&mut self, addr: u16) -> (u8, Flags) {
        if let Some(r) = self.patched(REAL_SIGN, Regs { hl: addr, ..Regs::default() }) { return (a_of(&r), flags_of(&r)); }
        self.cpc.real_sign(addr)
    }
    /// `SET ANGLE MODE` (`BD73`): non-zero = degrees.
    pub fn set_angle_mode(&mut self, degrees: u8) {
        if self.patched(SET_ANGLE_MODE, regs_a(degrees)).is_some() { return; }
        self.cpc.real_set_angle_mode(degrees)
    }
    /// `REAL PI` (`BD76`).
    pub fn real_pi(&mut self, addr: u16) {
        if self.patched(REAL_PI, Regs { hl: addr, ..Regs::default() }).is_some() { return; }
        self.cpc.real_pi(addr);
    }
    /// The one-operand real functions (`BD79` SQRT, `BD7F` LN, `BD82` LOG10, `BD85` EXP,
    /// `BD88` SIN, `BD8B` COS, `BD8E` TAN, `BD91` ATN): in place, carry = ok.
    ///
    /// On failure the firmware also leaves `A` (and the flags from `OR A`): `FF`/`FE`
    /// (sign: overflow), `00` (zero: division by zero) or `01` (neither: improper
    /// argument), which BASIC turns into its error.  The Rust firmware only reports
    /// success, so the flags are reconstructed from the case, as measured on the ROM:
    /// `LN`/`LOG10` of 0 or less and an `EXP` overflow give "sign"; `TAN` dividing by a
    /// zero cosine (result: the largest number) gives "zero"; `SQR` of a negative
    /// number and `SIN`/`COS`/`TAN` of an argument too large give neither.
    pub fn real_function(&mut self, entry: u16, addr: u16) -> Flags {
        if let Some(r) = self.patched(entry, Regs { hl: addr, ..Regs::default() }) { return flags_of(&r); }
        let ok = match entry {
            REAL_SQRT => self.cpc.real_sqrt(addr),
            REAL_LN => self.cpc.real_ln(addr),
            REAL_LOG10 => self.cpc.real_log10(addr),
            REAL_EXP => self.cpc.real_exp(addr),
            REAL_SIN => self.cpc.real_sin(addr),
            REAL_COS => self.cpc.real_cos(addr),
            REAL_TAN => self.cpc.real_tan(addr),
            REAL_ATN => self.cpc.real_atn(addr),
            _ => unreachable!("not a real function entry: {entry:04X}"),
        };
        if ok { return Flags { carry: true, zero: false, sign: false }; }
        match entry {
            REAL_LN | REAL_LOG10 | REAL_EXP => Flags { carry: false, zero: false, sign: true },
            REAL_TAN if self.is_largest_real(addr) => Flags { carry: false, zero: true, sign: false },
            _ => Flags { carry: false, zero: false, sign: false },
        }
    }
    /// Is the real at `addr` the largest magnitude (`FF FF FF 7F FF` with either sign,
    /// the firmware's result after an overflow or a division by zero)?
    fn is_largest_real(&self, addr: u16) -> bool {
        (0..5u16).all(|i| (self.ram(addr + i) | if i == 3 { 0x80 } else { 0 }) == 0xFF)
    }
    /// `RANDOMIZE RESET` (`BD97`).
    pub fn randomize_reset(&mut self) {
        if self.patched(RANDOMIZE_RESET, Regs::default()).is_some() { return; }
        self.cpc.random_reset()
    }
    /// `RANDOMIZE SEED` (`BD9A`): `HL` = real.
    pub fn randomize_seed(&mut self, addr: u16) {
        if self.patched(RANDOMIZE_SEED, Regs { hl: addr, ..Regs::default() }).is_some() { return; }
        self.cpc.random_seed_from_real(addr)
    }
    /// `RND` (`BD9D`): next random number at `HL`.
    pub fn rnd(&mut self, addr: u16) {
        if self.patched(RND, Regs { hl: addr, ..Regs::default() }).is_some() { return; }
        self.cpc.random_new(addr);
    }
    /// `RND LAST` (`BDA0`).
    pub fn rnd_last(&mut self, addr: u16) {
        if self.patched(RND_LAST, Regs { hl: addr, ..Regs::default() }).is_some() { return; }
        self.cpc.random_to_real(addr);
    }

    // ---- integers ----------------------------------------------------------------------

    /// `INT ABS SPLIT` (`BDA3`): `(HL = |hl|, B = sign byte)`.
    pub fn int_abs_split(&mut self, hl: u16) -> (u16, u8) {
        if let Some(r) = self.patched(INT_ABS_SPLIT, Regs { hl, ..Regs::default() }) { return (r.hl, hi(r.bc)); }
        maths::int_abs_split(hl)
    }
    /// `INT UNSIGNED SPLIT` (`BDA6`): `B` = 0.
    pub fn int_unsigned_split(&mut self, hl: u16) -> u8 {
        if let Some(r) = self.patched(INT_UNSIGNED_SPLIT, Regs { hl, ..Regs::default() }) { return hi(r.bc); }
        maths::int_unsigned_split()
    }
    /// `INT APPLY SIGN` (`BDA9`): `(HL, carry = fits)`.
    pub fn int_apply_sign(&mut self, hl: u16, b: u8) -> (u16, bool) {
        if let Some(r) = self.patched(INT_APPLY_SIGN, Regs { hl, bc: pair(b, 0), ..Regs::default() }) { return (r.hl, r.carry()); }
        let (v, ok, _) = maths::int_apply_sign(hl, b);
        (v, ok)
    }
    /// The two-operand integer operations `HL op DE` returning `(HL, carry)`:
    /// `BDAC` add, `BDB2` reverse subtract, `BDB5` multiply.
    pub fn int_op(&mut self, entry: u16, hl: u16, de: u16) -> (u16, bool) {
        if let Some(r) = self.patched(entry, Regs { hl, de, ..Regs::default() }) { return (r.hl, r.carry()); }
        match entry {
            INT_ADD => maths::int_add(hl, de),
            INT_REVERSE_SUB => maths::int_sub_reverse(hl, de),
            INT_MULTIPLY => maths::int_mul(hl, de),
            _ => unreachable!("not an integer op entry: {entry:04X}"),
        }
    }
    /// `INT DIVIDE` / `INT MODULUS` (`BDB8`/`BDBB`): `None` = division by zero (carry
    /// clear, zero set), else `(HL, carry = fits)`.
    pub fn int_divide(&mut self, entry: u16, hl: u16, de: u16) -> Option<(u16, bool)> {
        if let Some(r) = self.patched(entry, Regs { hl, de, ..Regs::default() }) {
            return if !r.carry() && r.af & F_ZERO != 0 { None } else { Some((r.hl, r.carry())) };
        }
        if entry == INT_DIVIDE { maths::int_div(hl, de) } else { maths::int_mod(hl, de) }
    }
    /// `INT MULTIPLY UNSIGNED` (`BDBE`): `(HL, carry = overflow)`.
    pub fn int_mul_unsigned(&mut self, hl: u16, de: u16) -> (u16, bool) {
        if let Some(r) = self.patched(INT_MULTIPLY_UNSIGNED, Regs { hl, de, ..Regs::default() }) { return (r.hl, r.carry()); }
        maths::int_mul_unsigned(hl, de)
    }
    /// `INT DIVIDE UNSIGNED` (`BDC1`): `(HL = quotient, DE = remainder)`.
    pub fn int_div_unsigned(&mut self, hl: u16, de: u16) -> (u16, u16) {
        if let Some(r) = self.patched(INT_DIVIDE_UNSIGNED, Regs { hl, de, ..Regs::default() }) { return (r.hl, r.de); }
        maths::int_div_unsigned(hl, de)
    }
    /// `INT COMPARE` (`BDC4`): `A` = `FF`/0/1.
    pub fn int_compare(&mut self, hl: u16, de: u16) -> u8 {
        if let Some(r) = self.patched(INT_COMPARE, Regs { hl, de, ..Regs::default() }) { return a_of(&r); }
        maths::int_compare(hl, de).0
    }
    /// `INT NEGATE` (`BDC7`): `(HL, carry = ok)`.
    pub fn int_negate(&mut self, hl: u16) -> (u16, bool) {
        if let Some(r) = self.patched(INT_NEGATE, Regs { hl, ..Regs::default() }) { return (r.hl, r.carry()); }
        maths::int_negate(hl)
    }
    /// `INT SIGN` (`BDCA`): `A` = `FF`/0/1 (see the firmware's quirk for multiples of 256).
    pub fn int_sign(&mut self, hl: u16) -> u8 {
        if let Some(r) = self.patched(INT_SIGN, Regs { hl, ..Regs::default() }) { return a_of(&r); }
        maths::int_sign(hl).0
    }
}
