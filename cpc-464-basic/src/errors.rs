//! # Line input, errors, `STOP`/`END`/`CONT`/`ON ERROR`/`RESUME`  (ROM `CA3B .. CE66`)
//!
//! ## Raising an error  (`CA94`)
//!
//! Any routine raises error `n` with `LD E,n : JP CA94` ([`Exit::Error`]).  The error
//! number, the current line ([`ERROR_LINE`], as a line address) and the statement
//! ([`ERROR_STATEMENT`]) are recorded and `CONT` is pointed at the statement.  Then
//! ([`Basic::report_error`], `CAA4`) BASIC's stack is put back to what it was when the
//! statement started, the string temporaries and `DEF FN` frames are dropped, and:
//!
//! * in a program, with `ON ERROR GOTO` set and not already in the handler: continue at
//!   the handler's line (`ERR`/`ERL` tell what happened; `RESUME` goes back);
//! * otherwise: print the message (and ` in <line>`) on stream 0 and go back to `Ready`.
//!
//! ## Soft errors
//!
//! *Division by zero* and *Overflow* in arithmetic only print their message and carry
//! on (with the firmware's result, the largest number) - unless an `ON ERROR GOTO` is
//! active, in which case they are real errors ([`Basic::division_by_zero_error`],
//! [`Basic::overflow_error`]).
//!
//! ## The messages
//!
//! The 30 messages are zero-terminated strings at `CC5B`; message 0 (and any number
//! above 30, or an empty entry) is "Unknown error".

use crate::repl::Never;
use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// The message table.
const MESSAGES: u16 = 0xCC5B;
/// "Overflow" (message 6).
const MSG_OVERFLOW: u16 = 0xCCB9;
/// "Division by zero" (message 11).
const MSG_DIVISION: u16 = 0xCD13;
/// "Undefined line ".
const MSG_UNDEFINED_LINE: u16 = 0xCB23;
/// "Break".
const MSG_BREAK: u16 = 0xCB4F;
/// " in ".
const MSG_IN: u16 = 0xCB55;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Line input   (CA3B .. CA83)
    // ===============================================================================

    /// ROM `CA3B`: read a line from the keyboard into the (emptied) input buffer.
    /// `false` if ESC was pressed.
    pub fn read_input_line(&mut self) -> Res<bool> {
        self.wr(INPUT_BUFFER, 0);
        self.edit_line(INPUT_BUFFER)                      // BD3A
    }

    /// ROM `CA43`: edit the text already in the input buffer, then a new line.
    pub fn edit_input_buffer(&mut self) -> Res<bool> {
        let entered = self.edit_line(INPUT_BUFFER)?;
        self.new_line()?;                                 // C34E (flags preserved)
        Ok(entered)
    }

    /// ROM `CA4C`: read a line of a cassette file into the input buffer.  `CR` ends it
    /// (and becomes the terminating 0); a `LF` at the start of a line is ignored, and a
    /// `CR` straight after a `LF` (or another such `CR`) is dropped.  *Line too long*
    /// after 255 characters; ESC is a *Break*.  `false` = end of file (or a read error).
    pub fn read_file_line(&mut self) -> Res<bool> {
        let mut p = INPUT_BUFFER;
        let mut count: u8 = 1;
        let mut after_lf = false;
        loop {
            let c = match self.cas_in_char() {
                Ok(c) => c,
                Err(e) if e.zero => return self.do_break().map(|n| match n {}),   // CB6B
                Err(_) => return Ok(false),
            };
            self.wr(p, c);
            if c == 0x0D {
                if after_lf { continue; }
                self.wr(p, 0);
                return Ok(true);
            }
            after_lf = false;
            if c == 0x0A {
                if count == 1 { continue; }
                after_lf = true;
            }
            if count == 0 { return Err(Exit::Error(0x17)); }
            p = p.wrapping_add(1);
            count = count.wrapping_add(1);
        }
    }

    // ===============================================================================
    // Raising errors   (CA84 .. CAE9)
    // ===============================================================================

    /// ROM `CA84`: no error (`ERR` = 0; `ERL` = the current line).
    pub fn clear_error(&mut self) { self.set_error_number(0); }

    /// ROM `CA85`: `ERR` = `e`, and the current line becomes `ERL`.
    fn set_error_number(&mut self, e: u8) {
        self.wr(ERROR_NUMBER, e);
        let line = self.current_line();
        self.wr16(ERROR_LINE, line);
    }

    /// `ERROR n` (ROM `CA8F`): raise error `n` (1..255).
    pub fn stmt_error(&mut self) -> Res {
        let e = self.eval_nonzero_byte()?;                // CE6D
        if !crate::is_end(self.cur()) { return Ok(()); }  // RET NZ: syntax error follows
        Err(Exit::Error(e))
    }

    /// ROM `CA94`: raise error `e` (the extension hook `AC04` sees it first).
    pub fn raise(&mut self, e: u8) -> Never {
        self.call_hook(HOOK_ERROR);
        self.set_error_number(e);
        let stmt = self.rd16(STATEMENT_PTR);
        self.wr16(ERROR_STATEMENT, stmt);
        self.save_cont(stmt);                             // CBB0
        self.report_error()
    }

    /// ROM `CAA4`: handle the error in [`ERROR_NUMBER`]: jump to the `ON ERROR GOTO`
    /// handler, or print the message and return to `Ready`.
    pub fn report_error(&mut self) -> Never {
        // LD SP,C000
        let sp = self.rd16(STATEMENT_STACK);
        self.set_stack(sp);
        self.reset_descriptor_stack();                    // FBB3
        self.clear_fn_frames();                           // D9FD
        let in_program = self.error_line_number().is_some();   // CADF
        let handler = self.rd16(ON_ERROR_LINE);
        if in_program && handler != 0 && self.rd(IN_ERROR_HANDLER) == 0 {
            self.wr(IN_ERROR_HANDLER, 0xFF);
            return Err(Exit::ContinueLine(handler));      // JP DD93
        }
        self.wr(IN_ERROR_HANDLER, 0);
        let msg = self.error_message(self.rd(ERROR_NUMBER));   // CC45
        let erl = self.rd16(ERROR_LINE);
        self.set_current_line(erl);
        self.print_message(msg)?;                         // CB36
        Err(Exit::Ready)
    }

    /// ROM `CADF`: the number of the line the last error happened in (`ERL`), `None` in
    /// direct mode.
    pub fn error_line_number(&self) -> Option<u16> {
        self.line_number_at(self.rd16(ERROR_LINE))
    }

    /// ROM `CAEA`: *Division by zero* in arithmetic.
    pub fn division_by_zero_error(&mut self) -> Res { self.soft_error(MSG_DIVISION, 0x0B) }

    /// ROM `CAF3`: *Overflow* in arithmetic.
    pub fn overflow_error(&mut self) -> Res { self.soft_error(MSG_OVERFLOW, 0x06) }

    /// ROM `CAFA`: a real error `e` if `ON ERROR GOTO` is active, else print the message
    /// on stream 0 and continue.
    fn soft_error(&mut self, msg: u16, e: u8) -> Res {
        if self.rd16(ON_ERROR_LINE) != 0 { return Err(Exit::Error(e)); }
        let old = self.set_output_stream(0);              // C1A2
        self.print_string(msg)?;
        self.new_line()?;
        self.set_output_stream(old);
        Ok(())
    }

    // ===============================================================================
    // Messages   (CB18 .. CB59, CC45)
    // ===============================================================================

    /// ROM `CB18`: `Undefined line <n> in <current line>` (`RENUM`).
    pub fn undefined_line_message(&mut self, n: u16) -> Res {
        self.text_tidy();                                 // C386
        self.print_string(MSG_UNDEFINED_LINE)?;
        self.print_number(n)?;                            // EE79
        self.print_in_line()                              // CB40
    }

    /// ROM `CB33`: `Break [in <line>]`.
    fn print_break(&mut self) -> Res { self.print_message(MSG_BREAK) }

    /// ROM `CB36`: on stream 0, on a fresh line: the message at `msg`, then ` in <line>`
    /// when a program is running.
    fn print_message(&mut self, msg: u16) -> Res {
        self.select_stream_0();                           // C19D
        self.text_tidy();
        self.print_string(msg)?;
        self.print_in_line()
    }

    /// ROM `CB40`: ` in <current line>` unless in direct mode.
    fn print_in_line(&mut self) -> Res {
        let Some(n) = self.current_line_number() else { return Ok(()) };
        self.print_string(MSG_IN)?;
        self.print_number(n)
    }

    /// ROM `CC45`: the address of the message for error `e`.
    fn error_message(&self, e: u8) -> u16 {
        if e >= 0x1F || e == 0 { return MESSAGES; }
        let mut p = MESSAGES;
        for _ in 0..e {
            loop {
                let c = self.rd(p);
                p += 1;
                if c == 0 { break; }
            }
        }
        if self.rd(p) == 0 { MESSAGES } else { p }
    }

    // ===============================================================================
    // STOP, END, the end of the program, CONT   (CB5A .. CBD8)
    // ===============================================================================

    /// `STOP` (ROM `CB5A`): `Break in <line>`; `CONT` continues after it.
    pub fn stmt_stop(&mut self) -> Res {
        if !crate::is_end(self.cur()) { return Ok(()); }
        self.print_break()?;
        let p = self.ptr;
        self.save_cont_after(p);                          // CB93
        Err(Exit::Ready)
    }

    /// `END` (ROM `CB65`): close the files, back to direct mode (`CONT` continues after
    /// it).
    pub fn stmt_end(&mut self) -> Res {
        if !crate::is_end(self.cur()) { return Ok(()); }
        let p = self.ptr;
        self.save_cont_after(p);
        self.end_and_close().map(|n| match n {})
    }

    /// ROM `CB6B`: report *Break* (ESC twice): `CONT` restarts the interrupted statement.
    pub fn do_break(&mut self) -> Never {
        self.print_break()?;
        let stmt = self.rd16(STATEMENT_PTR);
        self.save_cont(stmt);
        Err(Exit::Ready)
    }

    /// ROM `CB76`: the program ran off its end.  *No RESUME* if that happened inside an
    /// error handler.
    pub fn end_of_program(&mut self) -> Never {
        if self.current_line_number().is_none() {
            self.set_direct_mode();
            return Err(Exit::Ready);
        }
        self.cont_off();
        if self.rd(IN_ERROR_HANDLER) != 0 { return Err(Exit::Error(0x13)); }
        self.end_and_close()
    }

    /// ROM `CB87`: `CLOSEIN`, `CLOSEOUT`, direct mode, `Ready`.
    fn end_and_close(&mut self) -> Never {
        self.close_in()?;                                 // D298
        self.close_out()?;                                // D2A1
        self.set_direct_mode();
        Err(Exit::Ready)
    }

    /// ROM `CB93`: point `CONT` after the statement that ends at `p` (the next statement,
    /// or the next line; nothing at the end of the program).
    fn save_cont_after(&mut self, p: u16) {
        if self.current_line_number().is_none() { return; }
        if self.rd(p) == 1 { return self.save_cont(p); }
        if self.rd16(p.wrapping_add(1)) == 0 { return self.cont_off(); }
        let line = p.wrapping_add(3);
        self.set_current_line(line);
        self.save_cont(line.wrapping_add(1));
    }

    /// ROM `CBAB`: `CONT` is not possible.
    pub fn cont_off(&mut self) { self.wr16(CONT_STATEMENT, 0); }

    /// ROM `CBB0`: `CONT` continues at `p` in the current line (only while a program
    /// runs).
    fn save_cont(&mut self, p: u16) {
        if self.current_line_number().is_none() { return; }
        let line = self.current_line();
        self.wr16(CONT_LINE, line);
        self.wr16(CONT_STATEMENT, p);
    }

    /// `CONT` (ROM `CBC0`): *Cannot CONTinue* unless possible.
    pub fn stmt_cont(&mut self) -> Res {
        if !crate::is_end(self.cur()) { return Ok(()); }
        let p = self.rd16(CONT_STATEMENT);
        if p == 0 { return Err(Exit::Error(0x11)); }
        let line = self.rd16(CONT_LINE);
        self.set_current_line(line);
        self.sound_continue();
        Err(Exit::Continue(p))
    }

    // ===============================================================================
    // ON ERROR, RESUME   (CBD9 .. CC44)
    // ===============================================================================

    /// ROM `CBD9`: no `ON ERROR GOTO`, not in a handler.
    pub fn on_error_off(&mut self) {
        self.wr(IN_ERROR_HANDLER, 0);
        self.wr16(ON_ERROR_LINE, 0);
    }

    /// `ON ERROR GOTO line` (ROM `CBE5`, reached through `ON`).
    pub fn on_error_goto(&mut self) -> Res {
        self.next();
        self.expect(0xA0)?;                               // GOTO
        let n = self.read_line_number_constant()?;        // CEE1
        let at = self.find_line_or_error(n)?;             // E79A
        self.wr16(ON_ERROR_LINE, at);
        Ok(())
    }

    /// `ON ERROR GOTO 0` (ROM `CBF8`): switch the handler off; inside the handler, report
    /// the error being handled.
    pub fn stmt_on_error_goto_0(&mut self) -> Res {
        self.wr16(ON_ERROR_LINE, 0);
        if self.rd(IN_ERROR_HANDLER) == 0 { return Ok(()); }
        Err(Exit::ReportError)
    }

    /// `RESUME`, `RESUME NEXT`, `RESUME line` (ROM `CC03`): leave the error handler and
    /// continue at the statement that failed, the one after it, or a line.
    pub fn stmt_resume(&mut self) -> Res {
        let c = self.cur();
        if crate::is_end(c) {
            let stmt = self.resume_state()?;              // CC2B
            return Err(Exit::Continue(stmt));
        }
        if c == 0xB0 {
            // RESUME NEXT
            if !crate::is_end(self.next()) { return Ok(()); }
            let stmt = self.resume_state()?;
            self.ptr = stmt.wrapping_add(1);
            self.skip_to_end_of_statement();              // E8EF
            return Ok(());
        }
        let line = self.line_operand()?;                  // E767
        self.check_end()?;
        self.resume_state()?;
        Err(Exit::ContinueLine(line.wrapping_add(1)))
    }

    /// ROM `CC2B`: *Unexpected RESUME* outside a handler; otherwise leave it (`ERR` = 0),
    /// make `ERL` the current line again and return the failed statement.
    fn resume_state(&mut self) -> Res<u16> {
        if self.rd(IN_ERROR_HANDLER) == 0 { return Err(Exit::Error(0x14)); }
        self.wr(ERROR_NUMBER, 0);
        self.wr(IN_ERROR_HANDLER, 0);
        let erl = self.rd16(ERROR_LINE);
        self.set_current_line(erl);
        Ok(self.rd16(ERROR_STATEMENT))
    }
}
