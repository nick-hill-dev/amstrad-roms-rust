//! # Cold start and the `Ready` loop  (ROM `C000 .. C19C`)
//!
//! ## The ROM header
//!
//! ```text
//!   C000  80          ROM type: foreground
//!   C001  01 00 00    mark 1, version 0, modification 0
//!   C004  C04C        address of the name table: "BASIC" (last letter + 0x80), 0
//!   C006  ...         entry point
//! ```
//!
//! ## The `Ready` loop (`C064`)
//!
//! Every way back to direct mode (`END`, an error, a finished direct command) ends in
//! `JP C064`, which resets the Z80 stack and:
//!
//! 1. resets the string descriptor stack and the `DEF FN` frames, selects stream 0;
//! 2. holds the sound if a program was running, disarms the break key, tidies up the
//!    screen (new line if the cursor is not in column 1);
//! 3. erases a protected program;
//! 4. after a *Syntax error* in a program line, offers that line for editing;
//! 5. prints `Ready` and reads lines: a line starting with a number is stored in the
//!    program, anything else is tokenised into the buffer at [`MEMORY_BOTTOM`] and
//!    executed at once (direct mode).
//!
//! [`Basic::run`] drives this loop and the other non-local jump targets (see [`Exit`]).

use crate::vars::*;
use crate::program::Stored;
use crate::{Basic, Exit, Res};
use cpc464_os::machine::StartTarget;
use cpc464_os::Hardware;
use std::convert::Infallible;

/// A routine that never returns normally (it always ends in a non-local jump).
pub type Never = Res<Infallible>;

/// `" BASIC 1.0\n\n"` at `C03F`.
const BANNER: u16 = 0xC03F;
/// `"Ready\n"` at `C0CC`.
const READY: u16 = 0xC0CC;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // The driver
    // ===============================================================================

    /// Power the machine on and run BASIC: the firmware's cold start, then BASIC's.
    /// Returns only when [`Basic::line_input`] answers [`crate::LineInput::Stop`] (or a
    /// program is booted with `RUN"file"` of a binary, see [`Basic::boot_target`]).
    pub fn power_on_and_run(&mut self) {
        let target = self.cpc.power_on();
        self.run(target);
    }

    /// Enter BASIC as the firmware does after `MC START PROGRAM` (`target` gives the free
    /// memory in `DE`..`HL`) and keep interpreting until the host stops it.
    pub fn run(&mut self, target: StartTarget) {
        // BASIC runs with its ROM selected; the firmware's banking state is the one its
        // own routines see when BASIC calls them (lower ROM on, upper ROM off - see
        // `Basic::rd`).
        self.cpc.banking.upper_rom_number = target.rom;
        self.cpc.banking.lower_rom = true;
        self.cpc.banking.upper_rom = false;
        let r = self.cpc.banking.mode_rom_register;
        self.cpc.banking.mode_rom_register = (r & !0x0C) | 0x08;
        let mut next = never(self.cold_start(target.de, target.hl));
        loop {
            next = match next {
                Exit::Stop => return,
                Exit::Ready => never(self.ready()),
                Exit::Direct => never(self.direct_mode()),
                Exit::EditLine(line) => never(self.edit_error_line(line)),
                Exit::Error(e) => never(self.raise(e)),
                Exit::ReportError => never(self.report_error()),
                Exit::Continue(p) => never(self.interpret(p)),
                Exit::ContinueLine(p) => never(self.interpret_line(p)),
            };
        }
    }

    // ===============================================================================
    // Cold start   (C006)
    // ===============================================================================

    /// ROM `C006`: initialise BASIC.  `de`/`hl` = lowest / highest free byte.
    fn cold_start(&mut self, de: u16, hl: u16) -> Never {
        let (de, hl) = self.kl_rom_walk(de, hl);           // initialise background ROMs
        if !self.init_memory(de, hl) {                    // F4C4: too little memory?
            // JP C,0000 - reset the machine.  The host gets control back.
            return Err(Exit::Stop);
        }
        // The nine RAM hooks AC01..AC1B become RET.
        self.wr(EXT_FLAG, 0);
        for a in 0xAC01..=0xAC1Bu16 { self.wr(a, 0xC9); }
        self.print_banner(BANNER)?;                       // C337
        self.wr(EXT_FLAG, 0);
        self.set_direct_mode();                           // DDCB
        self.clear_error();                               // CA84
        self.randomize_reset();                           // BD97
        self.auto_off();                                  // C0D3
        self.new_program();                               // C13E
        self.symbol_after(0x00F0)?;                       // F706: SYMBOL AFTER 240
        Err(Exit::Ready)
    }

    // ===============================================================================
    // The Ready loop   (C064)
    // ===============================================================================

    /// ROM `C064`: back to direct mode.
    fn ready(&mut self) -> Never {
        self.call_hook(HOOK_READY);                       // CALL AC01
        // LD SP,C000 - the Z80 stack is reset; nothing to do here.
        self.reset_temporaries();                         // C162
        if self.current_line_number().is_some() {         // DDD6: a program was running
            self.sound_hold();
        }
        self.km_disarm_break();
        self.text_tidy();                                 // C386
        if self.rd(PROTECTED) != 0 { self.new_program(); } // a protected program is erased
        if self.rd(ERROR_NUMBER) == 2 {
            // After a Syntax error in a program line, offer the line for editing.
            self.wr(ERROR_NUMBER, 0);
            if let Some(line) = self.error_line_number() {
                return Err(Exit::EditLine(line));
            }
        }
        self.print_string(READY)?;                        // C090
        self.direct_mode()
    }

    /// `EDIT line` (ROM `C052`): edit a program line (falls into `C056`).
    pub fn stmt_edit(&mut self) -> Res {
        let line = self.read_line_number_constant()?;     // CEE1
        if !crate::is_end(self.cur()) { return Ok(()); }  // RET NZ: syntax error follows
        Err(Exit::EditLine(line))
    }

    /// ROM `C056`: show program line `line` in the line editor; ENTER stores (or executes)
    /// the edited text, ESC goes back to the `Ready` loop.  (Used after a *Syntax error*
    /// and by `EDIT`.)
    fn edit_error_line(&mut self, line: u16) -> Never {
        // LD SP,C000
        let at = self.find_line_or_error(line)?;          // E79A
        self.detokenise_line(at)?;                        // E163: into the input buffer
        if self.edit_input_buffer()? {                    // CA43
            self.store_or_execute(INPUT_BUFFER)           // C0B8
        } else {
            Err(Exit::Ready)                              // JR C064 (ESC)
        }
    }

    /// ROM `C096`: the direct-mode input loop (after `Ready`).
    fn direct_mode(&mut self) -> Never {
        loop {
            self.set_direct_mode();                       // DDCB
            loop {
                // C099: AUTO numbering?
                if self.rd(AUTO_ACTIVE) == 0 { break; }
                let Some(line) = self.auto_input()? else { return Err(Exit::Ready); };
                if self.rd(INPUT_BUFFER) == 0 { continue; } // empty line: offer the number again
                self.store_line(INPUT_BUFFER, line)?;     // E6D2
                self.after_program_change();              // C17A
            }
            // C0B0: read a line (ESC simply asks again).
            while !self.read_input_line()? {}
            self.new_line()?;                             // C34E
            match self.store_or_execute(INPUT_BUFFER) {
                Err(Exit::Direct) => continue,            // JR C096
                other => return other,
            }
        }
    }

    /// ROM `C0B8`: a line has been typed into the buffer at `text`.  If it starts with a
    /// line number it is stored in the program (and the loop continues with `JR C096`,
    /// reported as [`Exit::Direct`]); otherwise it is tokenised and executed.
    fn store_or_execute(&mut self, text: u16) -> Never {
        match self.store_numbered_line(text)? {           // E6BC
            Stored::Line => {
                self.after_program_change();              // C17A
                Err(Exit::Direct)
            }
            Stored::Nothing => Err(Exit::Direct),         // just a number, no such line
            Stored::Direct(start) => {
                // C0C2: a direct command.
                let tokens = self.tokenise(start)?;        // DEBB
                self.arm_break();                         // C453
                Err(Exit::Continue(tokens.wrapping_sub(1)))   // DEC HL; JP DD74
            }
        }
    }

    // ===============================================================================
    // AUTO   (C0D3 .. C12A)
    // ===============================================================================

    /// ROM `C0D3`: switch `AUTO` off.
    pub fn auto_off(&mut self) { self.wr(AUTO_ACTIVE, 0); }

    /// ROM `C0D6`: switch `AUTO` on, next line number `line`.
    fn auto_on(&mut self, line: u16) {
        self.wr16(AUTO_LINE, line);
        self.wr(AUTO_ACTIVE, 0xFF);
    }

    /// `AUTO [start][,step]` (ROM `C0DF`).  Both default to 10.  Does not return to the
    /// statement loop: it discards the statement's return address and enters the input
    /// loop at `C096` directly.
    pub fn stmt_auto(&mut self) -> Res {
        let mut start = 10;
        let c = self.cur();
        if !crate::is_end(c) && c != b',' {
            start = self.read_line_number_constant()?;    // CEE1
        }
        let mut step = 10;
        if self.comma() { step = self.read_line_number_constant()?; }
        self.check_end()?;                                // DD4A
        self.wr16(AUTO_STEP, step);
        self.auto_on(start);
        // POP BC (the statement return address); JP C096.
        Err(Exit::Direct)
    }

    /// ROM `C102`: one `AUTO` line: print the number (with `*` if that line exists
    /// already), read the rest of the line.  `None` if ESC was pressed (`AUTO` is then
    /// off); otherwise the line number used (the buffer holds the text).
    fn auto_input(&mut self) -> Res<Option<u16>> {
        let line = self.rd16(AUTO_LINE);
        self.print_number(line)?;                         // EE79
        let exists = self.find_line(line).1;      // E7A3
        self.print_char(if exists { b'*' } else { b' ' })?;
        self.auto_off();                                  // C0D3
        if !self.read_input_line()? { return Ok(None); }
        self.new_line()?;
        // Next number; AUTO stays off if it would overflow 65535.
        let (next, overflow) = self.rd16(AUTO_STEP).overflowing_add(line);
        if !overflow { self.auto_on(next); }
        Ok(Some(line))
    }

    // ===============================================================================
    // NEW, CLEAR and the reset routines   (C12B .. C19C)
    // ===============================================================================

    /// `NEW` (ROM `C12B`).
    pub fn stmt_new(&mut self) -> Res {
        if !crate::is_end(self.cur()) { return Ok(()); }  // RET NZ: syntax error follows
        self.new_program();
        Err(Exit::Ready)
    }

    /// `CLEAR` (ROM `C132`): forget all variables, close files, reset everything that
    /// depends on the program running.
    pub fn stmt_clear(&mut self) -> Res {
        self.clear_variables_and_strings();               // C18C
        self.close_files_and_reset_maths();               // C15B
        self.after_program_change();                      // C17A
        Ok(())
    }

    /// ROM `C13E`: erase the program and all of memory between [`MEMORY_BOTTOM`] and
    /// [`HIMEM`], then reset everything.
    pub fn new_program(&mut self) {
        let bottom = self.rd16(MEMORY_BOTTOM);
        let himem = self.rd16(HIMEM);
        let len = himem.wrapping_sub(bottom);
        // LD (HL),0 / LDIR: bottom .. himem inclusive.
        self.wr(bottom, 0);
        self.ldir(bottom, bottom.wrapping_add(1), len);
        self.wr(PROTECTED, 0);
        self.empty_program();                             // E676
        self.clear_variables_and_strings();               // C18C
        self.reset_program_state();                       // C16B
        self.close_files_and_reset_maths();               // C15B
    }

    /// ROM `C15B`: abandon the cassette files, radians, then [`Self::reset_temporaries`].
    fn close_files_and_reset_maths(&mut self) {
        self.abandon_files();                             // D2AD
        self.reset_angle_mode();                          // C15E
    }

    /// ROM `C15E`: `RAD`, then [`Self::reset_temporaries`].  (Also the tail of `RUN`.)
    pub fn reset_angle_mode(&mut self) {
        self.set_angle_mode(0);
        self.reset_temporaries();
    }

    /// ROM `C162`: empty the string descriptor stack and the `DEF FN` frames; streams 0.
    pub fn reset_temporaries(&mut self) {
        self.reset_descriptor_stack();                    // FBB3
        self.clear_fn_frames();                           // D9FD
        self.select_stream_0();                           // C19D
    }

    /// ROM `C16B`: `TROFF`, `AUTO` off, `ZONE 13`, an empty program, no variables, then
    /// [`Self::after_program_change`].
    pub fn reset_program_state(&mut self) {
        self.stmt_troff_core();                           // DDE6
        self.auto_off();                                  // C0D3
        self.reset_zone();                                // F1F2
        self.empty_program();                             // E676
        self.no_variables();                              // D5B1
        self.after_program_change();
    }

    /// ROM `C17A`: what must happen whenever the program changes or is (re)started:
    /// no `ON ERROR`, no `CONT`, events and sound reset, BASIC stack empty, no `DEF FN`s,
    /// `RESTORE`.
    pub fn after_program_change(&mut self) {
        self.on_error_off();                              // CBD9
        self.cont_off();                                  // CBAB
        self.reset_events();                              // C8ED
        self.reset_stack();                               // F58E
        self.clear_fn_list();                             // D5D2
        self.restore_data_start();                        // DCE5
    }

    /// ROM `C18C`: empty the string space, forget all variables and arrays, make every
    /// letter `DEFREAL`, and drop the cached variable offsets in the program text.
    pub fn clear_variables_and_strings(&mut self) {
        self.empty_string_space();                        // F5CA
        self.clear_variable_lists();                      // D5AE
        self.defreal_all();                               // D5FC
        self.forget_variable_offsets();                   // E989
    }

    /// Call one of the RAM hooks at `AC01..AC19` if an extension has patched it (the ROM
    /// always calls it; an unpatched hook is a plain `RET`).
    pub fn call_hook(&mut self, hook: u16) {
        if self.ram(hook) != 0xC9 {
            let mut r = cpc464_os::Regs { hl: self.ptr, ..Default::default() };
            self.cpc.hw.call(hook, &mut r);
        }
    }
}

/// Unwrap a routine that cannot return normally.
fn never(r: Never) -> Exit {
    match r {
        Ok(n) => match n {},
        Err(e) => e,
    }
}
