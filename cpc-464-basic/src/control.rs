//! # `FOR`/`NEXT`, `IF`, `GOTO`/`GOSUB`/`RETURN`, `WHILE`/`WEND`, `ON`  (ROM `C529 .. C806`, `C9C5 .. CA3A`)
//!
//! ## Frames on BASIC's stack
//!
//! Every frame ends with its own size, so the stack can be walked downwards from
//! [`STACK_PTR`]; the zero byte at the bottom ([`Basic::reset_stack`]) stops the walk.
//!
//! ```text
//!   GOSUB (6):        kind(1) return text(2) line(2) 06
//!   WHILE (7):        WHILE line(2) text after WEND(2) condition text(2) 07
//!   FOR integer (16): variable(2) limit(2) step(2) sign(1) loop text(2) FOR line(2)
//!                     text after NEXT(2) NEXT position(2) 10
//!   FOR real (22):    variable(2) limit(5) step(5) sign(1) ... as above ... 16
//! ```
//!
//! The `GOSUB` kind is 0 for `GOSUB`, 1 for an `ON SQ`/`AFTER`/`EVERY` event and 2 for
//! `ON BREAK GOSUB` (see [`crate::events`]).
//!
//! ## How a loop is identified
//!
//! `FOR` scans the program for its `NEXT` *before* running (counting nested `FOR`s;
//! `NEXT a,b` closes several), and the text position of that `NEXT`'s variable is the
//! loop's identity: `NEXT` finds its frame by looking for its own text position.
//! Likewise `WHILE` finds its `WEND` and is identified by the position after it.  So a
//! loop must be closed by the `NEXT` that follows it in the text - jumping to another
//! `NEXT` gives *Unexpected NEXT*.  A new `FOR` (`WHILE`) of a loop that is already on
//! the stack drops the old frame and everything above it.  `FOR` walks past `WHILE`
//! frames, `WHILE` past `FOR` frames; both stop at a `GOSUB` frame.  `RETURN` drops
//! everything above its frame.
//!
//! `FOR` ends by jumping into the `NEXT` code with "do not add the step" set
//! ([`FOR_STEP_FLAG`]), so the first test is done exactly like the later ones and a loop
//! that does not run at all continues after its `NEXT`.
//!
//! Quirk: that flag stays clear for the rest of a multi-variable `NEXT`: with
//! `FOR i=1 TO 3:FOR j=1 TO 0:NEXT j,i` the outer loop's step is never added.

use crate::vars::*;
use crate::{is_end, Basic, Exit, Res};
use cpc464_os::Hardware;

/// The tokens these statements look for.
const TOK_NEXT: u8 = 0xB0;
const TOK_FOR: u8 = 0x9E;
const TOK_WHILE: u8 = 0xD6;
const TOK_WEND: u8 = 0xD5;
const TOK_GOTO: u8 = 0xA0;
const TOK_GOSUB: u8 = 0x9F;
const TOK_THEN: u8 = 0xEB;
const TOK_TO: u8 = 0xEC;
const TOK_STEP: u8 = 0xE6;
const TOK_EQUALS: u8 = 0xEF;
const TOK_ERROR: u8 = 0x9C;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // FOR   (C529 .. C5F5)
    // ===============================================================================

    /// `FOR var = start TO limit [STEP step]` (ROM `C529`).
    pub fn stmt_for(&mut self) -> Res {
        let (var, t) = self.find_simple_variable()?;      // D6B3
        let text = self.ptr;
        let (next_pos, for_line) = self.find_next()?;     // C9C5 (current line = NEXT's)
        self.wr16(FOR_VARIABLE, next_pos);
        if let Some((start, _)) = self.find_for_frame(next_pos) { self.set_stack(start); }
        // The NEXT's variable, if it names one, must be ours.  (It is looked up - and
        // created - with the NEXT's line as current line, for the error message.)
        self.ptr = next_pos;
        let mut next_var = 0;
        if !is_end(self.cur()) { next_var = self.find_variable()?.0; }   // D686
        let after_next = self.ptr;
        if next_var != 0 && next_var != var { return Err(Exit::Error(1)); }
        let next_line = self.current_line();
        self.set_current_line(for_line);
        let size = match t {
            T_REAL => 0x16,
            T_INT => 0x10,
            _ => return Err(Exit::Error(0x0D)),
        };
        let frame = self.stack_alloc(size)?;
        self.wr16(frame, var);
        self.ptr = text;
        self.expect(TOK_EQUALS)?;
        self.eval()?;
        self.convert_acc_to(t)?;                          // FED7
        self.copy_acc_to(FOR_SCRATCH);
        self.expect(TOK_TO)?;
        self.eval()?;
        self.convert_acc_to(t)?;
        let mut p = frame + 2;
        self.copy_acc_to(p);                              // the limit
        p += t as u16;
        self.set_acc_int(1);                              // FF0D: default STEP 1
        if self.cur() == TOK_STEP {
            self.next();
            self.eval()?;
        }
        self.convert_acc_to(t)?;
        self.copy_acc_to(p);                              // the step
        p += t as u16;
        let sign = self.acc_sign()?;                      // FDA3
        self.wr(p, sign);
        p += 1;
        self.check_end()?;                                // DD4A
        let body = self.ptr;
        let line = self.current_line();
        let key = self.rd16(FOR_VARIABLE);
        self.wr16(p, body);
        self.wr16(p + 2, line);
        self.wr16(p + 4, after_next);
        self.wr16(p + 6, key);
        self.wr(p + 8, size);
        self.copy_value(FOR_SCRATCH, var);                // FF66: the start value
        self.wr(FOR_STEP_FLAG, 0);
        self.set_current_line(next_line);
        self.next_loop(key)                               // JR C600
    }

    // ===============================================================================
    // NEXT   (C5FB .. C6C6)
    // ===============================================================================

    /// `NEXT [var[,var...]]` (ROM `C5FB`).
    pub fn stmt_next(&mut self) -> Res {
        self.wr(FOR_STEP_FLAG, 0xFF);
        self.next_loop(self.ptr)
    }

    /// ROM `C600`: find the loop whose `NEXT` position is `key` (*Unexpected NEXT* if
    /// none), add the step, test.  Not finished: continue after the `FOR`.  Finished: drop
    /// the frame, continue after this `NEXT` (or with the next variable of the list).
    fn next_loop(&mut self, mut key: u16) -> Res {
        loop {
            let Some((frame, end)) = self.find_for_frame(key) else { return Err(Exit::Error(1)) };
            self.set_stack(end);
            let size = self.rd(end - 1);
            let (finished, sign_at) = self.step_and_test(frame, size)?;   // C661
            if !finished {
                let body = self.rd16(sign_at + 1);
                let line = self.rd16(sign_at + 3);
                self.set_current_line(line);
                self.ptr = body;
                return Ok(());
            }
            let after = self.rd16(sign_at + 5);
            self.set_stack(frame);
            self.ptr = after;
            if !self.comma() { return Ok(()); }
            key = self.ptr;
        }
    }

    /// ROM `C661`: add the step to the variable (unless [`FOR_STEP_FLAG`] is 0) and
    /// compare it with the limit.  `(finished, address of the sign byte)`: finished when
    /// `sign(value - limit)` equals the sign of the step.
    fn step_and_test(&mut self, frame: u16, size: u8) -> Res<(bool, u16)> {
        let var = self.rd16(frame);
        let limit = frame + 2;
        let stepping = self.rd(FOR_STEP_FLAG) != 0;
        let (c, sign_at) = if size == 0x10 {
            let mut v = self.rd16(var);
            if stepping {
                let step = self.rd16(frame + 4);
                let (s, ok) = self.int_op(crate::fw::INT_ADD, step, v);
                if !ok { return Err(Exit::Error(6)); }
                v = s;
                self.wr16(var, v);
            }
            let limit = self.rd16(limit);
            (self.int_compare(v, limit), frame + 6)
        } else {
            self.set_acc_from(var, T_REAL);               // FF4B
            if stepping {
                self.op_add(frame + 7, T_REAL)?;          // FCCC
                self.copy_acc_to(var);
            }
            (self.compare_numbers(limit, T_REAL)?, frame + 12)   // FD09
        };
        Ok((c == self.rd(sign_at), sign_at))
    }

    /// ROM `C632`: the `FOR` frame whose `NEXT` position is `key`: `(start, end)`.  `WHILE`
    /// frames are walked past; a `GOSUB` frame or the bottom of the stack ends the search.
    fn find_for_frame(&self, key: u16) -> Option<(u16, u16)> {
        let mut end = self.rd16(STACK_PTR);
        loop {
            let size = self.rd(end.wrapping_sub(1));
            let start = end.wrapping_sub(size as u16);
            match size {
                0x07 => {}
                0x10 | 0x16 => if self.rd16(end.wrapping_sub(3)) == key { return Some((start, end)); },
                _ => return None,
            }
            end = start;
        }
    }

    // ===============================================================================
    // IF, GOTO, GOSUB, RETURN   (C6C7 .. C746)
    // ===============================================================================

    /// `IF condition THEN|GOTO ... [ELSE ...]` (ROM `C6C7`).  After `THEN` (or `ELSE`)
    /// comes a line number (`GOTO`) or a statement.
    pub fn stmt_if(&mut self) -> Res {
        self.eval()?;
        if self.cur() != TOK_GOTO { self.expect(TOK_THEN)?; }
        if self.acc_sign()? == 0 && !self.skip_to_else() { return Ok(()); }   // E89F
        let c = self.cur();
        if is_end(c) { return Ok(()); }
        if c == 0x1E || c == 0x1D {
            let target = self.line_operand()?;            // E767
            self.ptr = target;
            return Ok(());
        }
        self.execute_statement()                          // DDAB
    }

    /// `GOTO line` (ROM `C6E8`): continue at the line (the text pointer is put on the
    /// byte before it, the previous line's end marker).
    pub fn stmt_goto(&mut self) -> Res {
        let target = self.line_operand()?;                // E767
        self.ptr = target;
        Ok(())
    }

    /// `GOSUB line` (ROM `C6ED`).
    pub fn stmt_gosub(&mut self) -> Res {
        let target = self.line_operand()?;
        self.skip_to_end_of_statement();                  // E8EF
        let ret = self.ptr;
        self.push_gosub_frame(0, ret)?;
        self.ptr = target;
        Ok(())
    }

    /// ROM `C6F6`: push a `GOSUB` frame: `kind`, the text to return to, the current line.
    pub fn push_gosub_frame(&mut self, kind: u8, ret: u16) -> Res {
        let f = self.stack_alloc(6)?;
        self.wr(f, kind);
        self.wr16(f + 1, ret);
        let line = self.current_line();
        self.wr16(f + 3, line);
        self.wr(f + 5, 6);
        Ok(())
    }

    /// `RETURN` (ROM `C70F`): drop everything above the newest `GOSUB` frame and return
    /// (*Unexpected RETURN* if there is none).  Event `GOSUB`s return through
    /// [`crate::events`].
    pub fn stmt_return(&mut self) -> Res {
        if !is_end(self.cur()) { return Ok(()); }         // RET NZ: syntax error follows
        let f = self.find_gosub_frame()?;                 // C72E
        self.set_stack(f);
        let kind = self.rd(f);
        let ret = self.rd16(f + 1);
        let line = self.rd16(f + 3);
        self.set_current_line(line);
        self.ptr = ret;
        match kind {
            0 => Ok(()),
            1 => self.return_from_event(ret),             // C8A4
            _ => self.return_from_break(ret),             // C8B6
        }
    }

    /// ROM `C72E`: the start of the newest `GOSUB` frame, walking past all other frames.
    fn find_gosub_frame(&self) -> Res<u16> {
        let mut end = self.rd16(STACK_PTR);
        loop {
            let size = self.rd(end.wrapping_sub(1));
            let start = end.wrapping_sub(size as u16);
            if size == 6 { return Ok(start); }
            if size == 0 { return Err(Exit::Error(3)); }
            end = start;
        }
    }

    // ===============================================================================
    // WHILE, WEND   (C747 .. C7E2)
    // ===============================================================================

    /// `WHILE condition` (ROM `C747`).
    pub fn stmt_while(&mut self) -> Res {
        let cond = self.ptr;
        let (after_wend, wend_line) = self.find_wend()?;  // CA18
        self.wr16(WHILE_SCRATCH, wend_line);
        if let Some(f) = self.find_while_frame(after_wend) { self.set_stack(f); }
        let f = self.stack_alloc(7)?;
        let line = self.current_line();
        self.wr16(f, line);
        self.wr16(f + 2, after_wend);
        self.wr16(f + 4, cond);
        self.wr(f + 6, 7);
        self.ptr = cond;
        self.while_test(after_wend)
    }

    /// `WEND` (ROM `C776`): back to the condition of the loop it closes (*Unexpected
    /// WEND* if there is none).
    pub fn stmt_wend(&mut self) -> Res {
        if !is_end(self.cur()) { return Ok(()); }
        let Some(f) = self.find_while_frame(self.ptr) else { return Err(Exit::Error(0x1E)) };
        self.set_stack(f + 7);
        let wend_line = self.current_line();
        self.wr16(WHILE_SCRATCH, wend_line);
        let line = self.rd16(f);
        self.set_current_line(line);
        let after = self.rd16(f + 2);
        self.ptr = self.rd16(f + 4);
        self.while_test(after)
    }

    /// ROM `C7A0`: evaluate the condition; true: run the body; false: drop the frame and
    /// continue after the `WEND`.
    fn while_test(&mut self, after_wend: u16) -> Res {
        self.eval()?;
        if self.acc_sign()? != 0 { return Ok(()); }
        let line = self.rd16(WHILE_SCRATCH);
        self.set_current_line(line);
        self.pop_stack(7);
        self.ptr = after_wend;
        Ok(())
    }

    /// ROM `C7B8`: the `WHILE` frame identified by `key` (the text after its `WEND`),
    /// walking past `FOR` frames.
    fn find_while_frame(&self, key: u16) -> Option<u16> {
        let mut end = self.rd16(STACK_PTR);
        loop {
            let size = self.rd(end.wrapping_sub(1));
            let start = end.wrapping_sub(size as u16);
            match size {
                0x10 | 0x16 => {}
                0x07 => if self.rd16(end.wrapping_sub(5)) == key { return Some(start); },
                _ => return None,
            }
            end = start;
        }
    }

    // ===============================================================================
    // ON   (C7E3 .. C806)
    // ===============================================================================

    /// `ON n GOTO|GOSUB line,line...` (ROM `C7E3`); `ON ERROR GOTO` is handled in
    /// [`crate::errors`].  `n` = 0 or larger than the list: continue with the next
    /// statement.
    pub fn stmt_on(&mut self) -> Res {
        if self.cur() == TOK_ERROR { return self.on_error_goto(); }   // CBE5
        let mut n = self.eval_byte()?;
        let token = self.cur();
        if token != TOK_GOTO {
            self.expect(TOK_GOSUB)?;
            self.ptr = self.ptr.wrapping_sub(1);
        }
        loop {
            n = n.wrapping_sub(1);
            if n == 0 { return self.execute_token(token); }   // DDAB
            self.next();
            self.read_line_number_constant()?;            // CEE1
            if self.cur() != b',' { return Ok(()); }
        }
    }

    // ===============================================================================
    // Finding the NEXT / WEND   (C9C5 .. CA3A)
    // ===============================================================================

    /// ROM `C9C5`: from the `FOR` statement, find the `NEXT` that closes it: the first
    /// `NEXT` (or variable of a `NEXT a,b`) not taken by a nested `FOR`.  *NEXT missing*
    /// if the program ends first.  Returns `(position of that NEXT's variable, the FOR's
    /// line)`; the current line is left at the `NEXT`'s.  The variables of a `NEXT` closing
    /// nested loops are looked up (and created) on the way.
    fn find_next(&mut self) -> Res<(u16, u16)> {
        let mut line = self.current_line();
        self.ptr = self.ptr.wrapping_sub(1);
        let mut depth: u8 = 1;
        loop {
            line = self.find_next_statement(line, 0x1A)?; // E923
            let at = self.ptr;
            let t = self.next();
            if t != TOK_NEXT {
                self.ptr = at;
                if t == TOK_FOR { depth = depth.wrapping_add(1); }
                continue;
            }
            let for_line = self.current_line();
            self.set_current_line(line);
            depth = depth.wrapping_sub(1);
            if depth != 0 {
                if !is_end(self.next()) {
                    loop {
                        self.find_variable()?;            // D686
                        if !self.comma() { break; }
                        depth = depth.wrapping_sub(1);    // DJNZ
                        if depth == 0 { break; }
                    }
                }
                self.ptr = self.ptr.wrapping_sub(1);
                if depth != 0 {
                    self.set_current_line(for_line);
                    continue;
                }
            }
            self.next();
            return Ok((self.ptr, for_line));
        }
    }

    /// ROM `CA18`: from the `WHILE` statement, find the matching `WEND` (*WEND missing*
    /// if none): `(text after it, its line)`.
    fn find_wend(&mut self) -> Res<(u16, u16)> {
        self.ptr = self.ptr.wrapping_sub(1);
        let mut line = self.current_line();
        let mut depth: u8 = 1;
        loop {
            line = self.find_next_statement(line, 0x1D)?; // E923
            let at = self.ptr;
            let t = self.next();
            self.ptr = at;
            if t == TOK_WHILE { depth = depth.wrapping_add(1); continue; }
            if t != TOK_WEND { continue; }
            depth = depth.wrapping_sub(1);
            if depth != 0 { continue; }
            self.next();
            self.next();
            return Ok((self.ptr, line));
        }
    }
}
