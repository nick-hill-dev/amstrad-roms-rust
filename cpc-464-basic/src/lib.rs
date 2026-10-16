//! # cpc464-basic
//!
//! A readable Rust reconstruction of **Locomotive BASIC 1.0**, the 16 KiB upper ROM of
//! the Amstrad CPC464 (the image normally called `BASIC1-0.ROM`, ROM 0, mapped at
//! `C000-FFFF`).
//!
//! It is a companion to [`cpc464_os`]: the interpreter runs *on top of* the Rust port of
//! the firmware, exactly as the ROM runs on top of the lower ROM.  Every routine of the
//! ROM was disassembled (Z80) and re-expressed as a Rust function.  Each function
//! documents
//!
//! * the ROM address it was recovered from,
//! * its original register calling convention (`Entry:` / `Exit:`) where that matters,
//! * any behaviour that is surprising or only makes sense in terms of the Z80 code.
//!
//! ## Layout of the ROM
//!
//! | ROM range     | Contents                                                              | Rust module        |
//! |---------------|-----------------------------------------------------------------------|--------------------|
//! | `C000-C005`   | ROM header (foreground ROM, version 1.0.0)                             | [`repl`]           |
//! | `C006-C19C`   | cold start, the `Ready` loop, `AUTO`, `NEW`, program/variable resets   | [`repl`]           |
//! | `C19D-C48B`   | streams, character output, screen commands, ESC/break handling         | [`streams`]        |
//! | `C48C-C528`   | graphics commands (`ORIGIN`, `CLG`, `DRAW`, `PLOT`, `MOVE`, `TEST`)    | [`graphics`]       |
//! | `C529-C806`   | `FOR`/`NEXT`, `IF`, `GOTO`/`GOSUB`/`RETURN`, `WHILE`/`WEND`, `ON`      | [`control`]        |
//! | `C807-CA3A`   | events: `ON BREAK`, `ON SQ`, `AFTER`/`EVERY`, `REMAIN`, `DI`/`EI`      | [`events`]         |
//! | `CA3B-CE66`   | line input, errors and their messages, `STOP`/`END`/`CONT`/`RESUME`    | [`errors`]         |
//! | `CE67-D0BA`   | the expression evaluator, operand fetch                                | [`eval`]           |
//! | `D0BB-D2BF`   | function dispatch and tables, `ERR`/`TIME`/`HIMEM`..., `DEF FN`, `MAX`/`MIN`/`ROUND`, cassette files (`CAT`, `OPENIN`, `CLOSEOUT`...) | [`functions`] |
//! | `D2C0-D408`   | `SOUND`, `RELEASE`, `SQ`, `ENV`, `ENT`                                  | [`sound`]          |
//! | `D409-D5AD`   | `INKEY`, `JOY`, `KEY`, `SPEED`, `PI`, `DEG`/`RAD`, `^`, maths functions, `RANDOMIZE`, `RND` | [`functions`] |
//! | `D5AE-DAF7`   | variables and arrays: lookup, creation, `DIM`, `ERASE`, `DEFINT`...    | [`variables`]      |
//! | `DAF8-DD36`   | `INPUT`, `LINE INPUT`, `READ`, `DATA`, `RESTORE`                       | [`input`]          |
//! | `DD37-DEBA`   | the statement interpreter loop, `TRON`, the statement table            | [`exec`]           |
//! | `DEBB-E675`   | tokeniser, keyword tables, de-tokeniser                                 | [`tokens`]         |
//! | `E0F7-E162`, `E676-EC9E` | `LIST`; program store: line insertion/search, line-number references, `DELETE`, `RENUM`, text scanning, `RUN`, `LOAD`, `CHAIN`, `MERGE`, `SAVE` | [`program`] |
//! | `ECA3-F157`   | number parsing and formatting, `BIN$`/`HEX$` digits                     | [`numbers`]        |
//! | `F158-F4C3`   | `PEEK`/`POKE`, `INP`/`OUT`/`WAIT`, `CALL`, RSX, `ZONE`, `PRINT [USING]`, `WRITE` | [`mod@print`] |
//! | `F4C4-F7CA`   | memory layout: `HIMEM`, `MEMORY`, cassette buffers, `SYMBOL`           | [`memory`]         |
//! | `F7CB-FCB2`   | strings: literals, the descriptor stack, string functions, garbage collection | [`strings`] |
//! | `FCB3-FFFF`   | operators, type conversion of the accumulator, small helpers           | [`arith`]          |
//!
//! ## Modelling decisions
//!
//! * **Memory.**  All of BASIC's state lives in RAM at the original addresses (see
//!   [`vars`]) - the program, the variables, the strings, BASIC's own stack, the numeric
//!   accumulator.  Nothing is cached in Rust fields that the ROM keeps in RAM, so a
//!   program that PEEKs and POKEs BASIC's workspace still works.
//! * **The text pointer.**  While interpreting, the Z80's `HL` points at the current
//!   character of the program text.  That register is [`Basic::ptr`]; routines that
//!   "advance `HL`" advance `ptr`.  `HL` used for anything else becomes a local variable.
//! * **Flags.**  The carry / zero flags that routines use to report results become
//!   `bool` / `Option` return values.
//! * **Non-local exits.**  The ROM leaves a statement in four drastic ways: `JP CA94`
//!   (raise an error), `JP C064` (back to `Ready`, resetting `SP`), `JP DD74`/`JP DD93`
//!   (continue interpreting somewhere else, abandoning the current statement).  These are
//!   the variants of [`Exit`], returned as the error half of [`Res`] and propagated with
//!   `?` up to [`Basic::run`], which does what the jump target does.
//! * **Firmware.**  BASIC reaches the firmware only through the RAM jumpblock
//!   (`BB00-BDCA`).  Every call goes through [`fw`]: if the jumpblock entry still holds the
//!   value `JUMP RESTORE` puts there, the Rust implementation in [`cpc464_os`] runs;
//!   if a program has patched the entry, the patched code is called through
//!   [`Hardware::call`] with the registers the ROM would pass.
//! * **The Z80 stack.**  BASIC keeps its own control stack (`FOR`, `GOSUB`, `WHILE`...)
//!   in RAM ([`vars::STACK_PTR`]); that is reproduced exactly.  The Z80 stack the ROM uses
//!   for temporaries is not modelled: Rust locals take its place.  The only place where
//!   the Z80 stack is visible to the outside is `CALL`, which passes its parameters on it;
//!   see [`mod@print`].
//! * **Banking.**  While BASIC runs, the upper ROM (BASIC itself) is visible at
//!   `C000-FFFF` and the lower ROM is off.  Reads made by the ROM through `HL` therefore see
//!   BASIC's own tables there ([`Basic::rd`]).  The firmware, called through the
//!   jumpblock, runs with the lower ROM on and the upper ROM off; that is the state the
//!   Rust firmware is left in.  (A program that changes the ROM configuration with `OUT`
//!   is not modelled.)
//! * **Line input.**  The only way BASIC reads a line is the firmware line editor
//!   (`BD3A`).  A host can intercept that with [`Basic::line_input`] - this is how the
//!   tests feed programs to the interpreter, let time pass ([`LineInput::Ticks`]), and
//!   tell [`Basic::run`] to return.
//! * **Firmware results.**  Where the Rust firmware reports less than the ROM leaves in
//!   the registers and BASIC depends on it (the error flags of the maths routines), the
//!   wrappers in [`fw`] reconstruct it; see [`Basic::real_function`].

use cpc464_os::{Cpc, Hardware};

pub mod arith;
pub mod control;
pub mod errors;
pub mod eval;
pub mod events;
pub mod exec;
pub mod functions;
pub mod fw;
pub mod graphics;
pub mod input;
pub mod memory;
pub mod numbers;
pub mod print;
pub mod program;
pub mod repl;
pub mod sound;
pub mod streams;
pub mod strings;
pub mod tokens;
pub mod variables;
pub mod vars;

/// A non-local transfer of control: the reason a statement did not simply return.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exit {
    /// `JP CA94` with `E` = error number: raise BASIC error `n` (see [`errors`]).
    Error(u8),
    /// `JP C064`: back to the `Ready` loop (`END`, a finished direct command, a message
    /// already printed ...).
    Ready,
    /// `JP C096`: back to direct mode without printing `Ready` (after `AUTO`, an edited
    /// line, ...).
    Direct,
    /// `JP C056` with `HL` = line number: edit that line (an error in a program line
    /// when the error is a *Syntax error*).
    EditLine(u16),
    /// `JP CAA4`: report the error already stored in `ERR` (`ERROR`, `RESUME` ...).
    ReportError,
    /// `JP DD74` with `HL`: continue interpreting at this text address.
    Continue(u16),
    /// `JP DD93` with `HL`: continue interpreting at the start of the line whose header
    /// (its two link bytes) starts at this address; a zero link means "end of program".
    ContinueLine(u16),
    /// The host asked [`Basic::run`] to return (see [`Basic::line_input`]).  Not a ROM
    /// concept.
    Stop,
}

/// Result type of almost every routine: `Err` is a non-local exit.
pub type Res<T = ()> = Result<T, Exit>;

/// What the [`Basic::line_input`] hook returns instead of running the line editor.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LineInput {
    /// The user typed this text and pressed ENTER.
    Line(Vec<u8>),
    /// The user pressed ESC.
    Escape,
    /// Make [`Basic::run`] return now.
    Stop,
    /// Let this many 1/300 s interrupts happen (time passing while the user types), then
    /// ask again.  Events that become due are processed as soon as a program runs.
    Ticks(u32),
}

/// The host's line-input hook: it gets the text already in the edit buffer (for `EDIT`,
/// `AUTO` or an error line) and returns what was "typed".
pub type LineInputHook = Box<dyn FnMut(&[u8]) -> LineInput>;

/// The interpreter: the firmware machine plus BASIC's ROM image and its one register.
pub struct Basic<H: Hardware> {
    /// The firmware (lower ROM) and the whole machine: RAM, hardware.
    pub cpc: Cpc<H>,
    /// The 16 KiB BASIC ROM image.  Only tables and messages are ever read from it.
    pub rom: Box<[u8; 0x4000]>,
    /// The text pointer: the Z80's `HL` while interpreting.
    pub ptr: u16,
    /// Optional replacement for the firmware line editor (see [`LineInput`]).
    pub line_input: Option<LineInputHook>,
    /// Set when BASIC hands the machine to another program (`MC BOOT PROGRAM` after
    /// `RUN"file"` of a binary): where it would have jumped.  [`Basic::run`] returns then.
    pub boot_target: Option<cpc464_os::machine::StartTarget>,
}

impl<H: Hardware> Basic<H> {
    /// Build an interpreter around the two ROM images.  Panics unless both are 16 KiB.
    pub fn new(os_rom: &[u8], basic_rom: &[u8], hw: H) -> Self {
        assert_eq!(basic_rom.len(), 0x4000, "BASIC ROM image must be exactly 16384 bytes");
        let mut rom = Box::new([0u8; 0x4000]);
        rom.copy_from_slice(basic_rom);
        Basic { cpc: Cpc::new(os_rom, hw), rom, ptr: 0, line_input: None, boot_target: None }
    }

    // ---- memory, as the BASIC ROM sees it ---------------------------------------------

    /// CPU read as BASIC's code sees it: its own ROM at `C000-FFFF`, RAM below (the
    /// lower ROM is always off while BASIC runs).
    ///
    /// The firmware's view is kept separately in [`Cpc::banking`]: there the lower ROM is
    /// on and the upper ROM off, as the jumpblock's `LOW JUMP` arranges for every
    /// firmware call (the Rust firmware reads its own tables and the screen memory
    /// through that state).
    #[inline]
    pub fn rd(&self, a: u16) -> u8 {
        if a >= 0xC000 { self.rom[(a - 0xC000) as usize] } else { self.cpc.ram[a as usize] }
    }
    #[inline]
    pub fn rd16(&self, a: u16) -> u16 { self.rd(a) as u16 | (self.rd(a.wrapping_add(1)) as u16) << 8 }
    /// Writes always go to RAM.
    #[inline]
    pub fn wr(&mut self, a: u16, v: u8) { self.cpc.wr8(a, v) }
    #[inline]
    pub fn wr16(&mut self, a: u16, v: u16) { self.cpc.wr16(a, v) }
    /// RAM read ignoring the ROMs (`RST 4`, `RAM LAM`).
    #[inline]
    pub fn ram(&self, a: u16) -> u8 { self.cpc.rd8(a) }
    #[inline]
    pub fn ram16(&self, a: u16) -> u16 { self.cpc.rd16(a) }

    /// Byte of BASIC's own ROM by CPU address (`C000-FFFF`).
    #[inline]
    pub fn rom_byte(&self, a: u16) -> u8 { self.rom[(a - 0xC000) as usize] }
    #[inline]
    pub fn rom_word(&self, a: u16) -> u16 { self.rom_byte(a) as u16 | (self.rom_byte(a + 1) as u16) << 8 }

    /// Fill `len` bytes from `a` with `v`.
    pub fn fill(&mut self, a: u16, len: u16, v: u8) {
        for i in 0..len { self.wr(a.wrapping_add(i), v); }
    }
    /// Z80 `LDIR`: copy `len` bytes upwards from `src` to `dst` (byte by byte, so an
    /// overlapping move towards higher addresses smears, exactly like the Z80).
    pub fn ldir(&mut self, src: u16, dst: u16, len: u16) {
        for i in 0..len {
            let v = self.rd(src.wrapping_add(i));
            self.wr(dst.wrapping_add(i), v);
        }
    }
    /// Z80 `LDDR`: copy `len` bytes downwards; `src`/`dst` are the addresses of the *last*
    /// byte.
    pub fn lddr(&mut self, src: u16, dst: u16, len: u16) {
        for i in 0..len {
            let v = self.rd(src.wrapping_sub(i));
            self.wr(dst.wrapping_sub(i), v);
        }
    }

    // ---- the text pointer -------------------------------------------------------------

    /// The character under the text pointer (`LD A,(HL)`).
    #[inline]
    pub fn cur(&self) -> u8 { self.rd(self.ptr) }
}

/// `true` for the bytes that end a statement in tokenised text: `00` (end of line) and
/// `01` (the tokenised `:`).
#[inline]
pub fn is_end(c: u8) -> bool { c < 2 }
