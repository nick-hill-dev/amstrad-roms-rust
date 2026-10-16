//! # The statement interpreter  (ROM `DD37 .. DEBA`)
//!
//! ## Text scanning
//!
//! The text pointer usually rests on a character that has already been looked at.  The
//! basic step, [`Basic::next`] (`DD3F`), moves to the next non-space character and
//! reports the end of a statement (`00` = end of line, `01` = `:`).  The other helpers
//! are built on it:
//!
//! | ROM    | here                     | does                                                   |
//! |--------|--------------------------|--------------------------------------------------------|
//! | `DD37` | [`Basic::expect`]        | the current character must be the given one (inline byte after the `CALL`), then `next` |
//! | `DD4A` | [`Basic::check_end`]     | the statement must end here, else *Syntax error*        |
//! | `DD51` | `is_end(cur())`          | does the statement end here?                            |
//! | `DD55` | [`Basic::comma`]         | is there a comma (spaces skipped)? if so step over it   |
//! | `DD61` | [`Basic::skip_white`]    | skip spaces, TABs and line feeds (for untokenised text) |
//!
//! ## The loop  (`DD74`)
//!
//! For every statement: remember where it starts ([`STATEMENT_PTR`], and BASIC's stack
//! pointer in [`STATEMENT_STACK`]), process pending events, fetch the first token and run
//! the statement.  After it the text pointer must be on `:` (next statement), on `00`
//! (next line, `DD93`) or the statement was badly formed (*Syntax error*).  A line is
//! `length(2) number(2) text 00`; a zero length ends the program.
//!
//! Statement handlers are entered with the text pointer on the first non-space character
//! after the keyword.

use crate::repl::Never;
use crate::vars::*;
use crate::{is_end, Basic, Exit, Res};
use cpc464_os::Hardware;

/// The statement table: handler addresses of the tokens `80..DC`.
pub const STATEMENT_TABLE: u16 = 0xDE01;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Scanning   (DD37 .. DD70)
    // ===============================================================================

    /// ROM `DD3F`: move to the next non-space character and return it.  (The ROM also
    /// returns Z for the end of a statement, `00` or `01`: [`is_end`].)
    pub fn next(&mut self) -> u8 {
        loop {
            self.ptr = self.ptr.wrapping_add(1);
            let c = self.cur();
            if c != b' ' { return c; }
        }
    }

    /// ROM `DD37`: the current character must be `c` (else *Syntax error*); step past it.
    pub fn expect(&mut self, c: u8) -> Res<u8> {
        if self.cur() != c { return Err(Exit::Error(2)); }
        Ok(self.next())
    }

    /// ROM `DD4A`: the statement must end at the current character.
    pub fn check_end(&self) -> Res {
        if is_end(self.cur()) { Ok(()) } else { Err(Exit::Error(2)) }
    }

    /// ROM `DD55`: skip spaces from the current character; if it is a comma step over it
    /// (and any spaces after it) and return `true`.
    pub fn comma(&mut self) -> bool {
        self.ptr = self.ptr.wrapping_sub(1);
        if self.next() != b',' { return false; }
        self.next();
        true
    }

    /// ROM `DD61`: skip spaces, TABs and line feeds, starting with the current character.
    pub fn skip_white(&mut self) -> u8 {
        loop {
            let c = self.cur();
            if c != b' ' && c != 0x09 && c != 0x0A { return c; }
            self.ptr = self.ptr.wrapping_add(1);
        }
    }

    // ===============================================================================
    // The loop   (DD71 .. DDC8)
    // ===============================================================================

    /// ROM `DD74`: interpret from text address `p` (the byte before the next statement:
    /// a `:`, or the high byte of a line number).
    pub fn interpret(&mut self, mut p: u16) -> Never {
        loop {
            let sp = self.rd16(STACK_PTR);
            self.wr16(STATEMENT_STACK, sp);
            self.wr16(STATEMENT_PTR, p);
            self.ptr = p;
            if self.kl_poll_synchronous() {
                self.process_events()?;                   // C807
            }
            let c = self.next();
            if !is_end(c) { self.execute_statement()?; }  // DDAB
            match self.cur() {
                1 => { p = self.ptr; continue; }
                0 => {}
                _ => return Err(Exit::Error(2)),          // DDC6
            }
            p = self.start_line(self.ptr.wrapping_add(1))?;
        }
    }

    /// ROM `DD93`: start interpreting the line whose header is at `p`.
    pub fn interpret_line(&mut self, p: u16) -> Never {
        let p = self.start_line(p)?;
        self.interpret(p)
    }

    /// ROM `DD93`: enter the line at `p`: a zero length is the end of the program
    /// (`CB76`); otherwise it becomes the current line (traced if `TRON`) and the address
    /// to continue from is returned.
    fn start_line(&mut self, p: u16) -> Res<u16> {
        if self.rd16(p) == 0 { return self.end_of_program().map(|n| match n {}); }
        let line = p.wrapping_add(2);
        self.wr16(CURRENT_LINE, line);
        if self.rd(TRACE) != 0 { self.trace_line()?; }    // DDEB
        Ok(line.wrapping_add(1))
    }

    /// ROM `DDAB`: execute the statement whose first character is at the text pointer.
    /// Not a keyword: an assignment (or `|` for an RSX).  Keywords `80..DC` go through the
    /// table at [`STATEMENT_TABLE`]; others are offered to the extension hook `AC07` and
    /// are a *Syntax error*.
    pub fn execute_statement(&mut self) -> Res {
        let t = self.cur();
        self.execute_token(t)
    }

    /// ROM `DDAB` proper: execute statement `t` (`A`); for a keyword the text pointer is
    /// first moved to the next character (`ON` enters here with the pointer before the
    /// line number and `t` = `GOTO`/`GOSUB`).
    pub fn execute_token(&mut self, t: u8) -> Res {
        if t < 0x80 { return self.implicit_let(); }      // D64F
        if t >= 0xDD {
            self.call_hook(HOOK_BAD_STATEMENT);           // DDC3
            return Err(Exit::Error(2));
        }
        self.next();
        match t {
            0x80 => self.stmt_after(),
            0x81 => self.stmt_auto(),
            0x82 => self.stmt_border(),
            0x83 => self.stmt_call(),
            0x84 => self.stmt_cat(),
            0x85 => self.stmt_chain(),
            0x86 => self.stmt_clear(),
            0x87 => self.stmt_clg(),
            0x88 => self.stmt_closein(),
            0x89 => self.stmt_closeout(),
            0x8A => self.stmt_cls(),
            0x8B => self.stmt_cont(),
            0x8C => self.stmt_data(),
            0x8D => self.stmt_def(),
            0x8E => self.stmt_defint(),
            0x8F => self.stmt_defreal(),
            0x90 => self.stmt_defstr(),
            0x91 => self.stmt_deg(),
            0x92 => self.stmt_delete(),
            0x93 => self.stmt_dim(),
            0x94 => self.stmt_draw(),
            0x95 => self.stmt_drawr(),
            0x96 => self.stmt_edit(),
            0x97 => self.stmt_skip_rest(),                // ELSE: like REM
            0x98 => self.stmt_end(),
            0x99 => self.stmt_ent(),
            0x9A => self.stmt_env(),
            0x9B => self.stmt_erase(),
            0x9C => self.stmt_error(),
            0x9D => self.stmt_every(),
            0x9E => self.stmt_for(),
            0x9F => self.stmt_gosub(),
            0xA0 => self.stmt_goto(),
            0xA1 => self.stmt_if(),
            0xA2 => self.stmt_ink(),
            0xA3 => self.stmt_input(),
            0xA4 => self.stmt_key(),
            0xA5 => self.stmt_let(),
            0xA6 => self.stmt_line(),
            0xA7 => self.stmt_list(),
            0xA8 => self.stmt_load(),
            0xA9 => self.stmt_locate(),
            0xAA => self.stmt_memory(),
            0xAB => self.stmt_merge(),
            0xAC => self.stmt_mid(),
            0xAD => self.stmt_mode(),
            0xAE => self.stmt_move(),
            0xAF => self.stmt_mover(),
            0xB0 => self.stmt_next(),
            0xB1 => self.stmt_new(),
            0xB2 => self.stmt_on(),
            0xB3 => self.stmt_on_break(),
            0xB4 => self.stmt_on_error_goto_0(),
            0xB5 => self.stmt_on_sq(),
            0xB6 => self.stmt_openin(),
            0xB7 => self.stmt_openout(),
            0xB8 => self.stmt_origin(),
            0xB9 => self.stmt_out(),
            0xBA => self.stmt_paper(),
            0xBB => self.stmt_pen(),
            0xBC => self.stmt_plot(),
            0xBD => self.stmt_plotr(),
            0xBE => self.stmt_poke(),
            0xBF => self.stmt_print(),
            0xC0 => self.stmt_skip_rest(),                // '
            0xC1 => self.stmt_rad(),
            0xC2 => self.stmt_randomize(),
            0xC3 => self.stmt_read(),
            0xC4 => self.stmt_release(),
            0xC5 => self.stmt_skip_rest(),                // REM
            0xC6 => self.stmt_renum(),
            0xC7 => self.stmt_restore(),
            0xC8 => self.stmt_resume(),
            0xC9 => self.stmt_return(),
            0xCA => self.stmt_run(),
            0xCB => self.stmt_save(),
            0xCC => self.stmt_sound(),
            0xCD => self.stmt_speed(),
            0xCE => self.stmt_stop(),
            0xCF => self.stmt_symbol(),
            0xD0 => self.stmt_tag(),
            0xD1 => self.stmt_tagoff(),
            0xD2 => { self.stmt_troff_core(); Ok(()) }
            0xD3 => { self.wr(TRACE, 0xFF); Ok(()) }       // TRON (DDE2)
            0xD4 => self.stmt_wait(),
            0xD5 => self.stmt_wend(),
            0xD6 => self.stmt_while(),
            0xD7 => self.stmt_width(),
            0xD8 => self.stmt_window(),
            0xD9 => self.stmt_write(),
            0xDA => self.stmt_zone(),
            0xDB => self.stmt_di(),
            _ => self.stmt_ei(),                          // 0xDC
        }
    }

    // ===============================================================================
    // Current line, TRON   (DDCB .. DE00)
    // ===============================================================================

    /// ROM `DDCB`: direct mode (no current line).
    pub fn set_direct_mode(&mut self) { self.wr16(CURRENT_LINE, 0); }

    /// ROM `DDCE`: set the current line (address of its number).
    pub fn set_current_line(&mut self, line: u16) { self.wr16(CURRENT_LINE, line); }

    /// ROM `DDD2`: the current line's address (0 in direct mode).
    pub fn current_line(&self) -> u16 { self.rd16(CURRENT_LINE) }

    /// ROM `DDD6`: the number of the current line, `None` in direct mode.
    pub fn current_line_number(&self) -> Option<u16> { self.line_number_at(self.rd16(CURRENT_LINE)) }

    /// ROM `DDD9`: the line number stored at `addr`, `None` for address 0.
    pub fn line_number_at(&self, addr: u16) -> Option<u16> {
        if addr == 0 { None } else { Some(self.rd16(addr)) }
    }

    /// `TROFF` (ROM `DDE6`).
    pub fn stmt_troff_core(&mut self) { self.wr(TRACE, 0); }

    /// ROM `DDEB`: `TRON` output: `[line]`.
    fn trace_line(&mut self) -> Res {
        self.print_char(b'[')?;
        let n = self.rd16(self.rd16(CURRENT_LINE));
        self.print_number(n)?;                            // EE79
        self.print_char(b']')
    }

    /// `REM`, `'` and `ELSE` (ROM `E8F3`): skip the rest of the line.
    pub fn stmt_skip_rest(&mut self) -> Res {
        self.skip_to_end_of_line();
        Ok(())
    }
}
