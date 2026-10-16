//! # Functions, `DEF FN`, cassette files, keys, maths  (ROM `D0BB .. D2BF`, `D409 .. D5AD`)
//!
//! ## Function dispatch
//!
//! A function is the token `FF` followed by its number (`D080`, in [`crate::eval`]).
//! The ROM jumps to an address taken from one of two tables (`D190` for `00..1D` and
//! `71..7F`, `D0CA` for `40..48`); [`Basic::call_function`] maps each address to its Rust
//! routine.  `00..1D` find their single argument already evaluated in the accumulator;
//! `71..7F` read their own arguments and the closing bracket; `40..48` have none.
//!
//! ## `DEF FN`
//!
//! `DEF FNname[(params)]=expression` stores the text position after the name in a
//! two-byte "variable" of the `DEF FN` list - nothing else is checked until the function
//! is called.  `FNname(args)` (see [`Basic::fn_user`]) builds a frame with the parameters
//! (each assigned its argument, converted to the parameter's type), makes it visible and
//! evaluates the expression in the definition's text, then converts the result to the
//! function name's type.
//!
//! ## Maths errors
//!
//! A failing firmware maths routine returns carry clear; zero means *Division by zero*,
//! sign means *Overflow* (both soft errors, see [`crate::errors`]), otherwise *Improper
//! argument*.

use crate::fw::*;
use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;
use cpc464_os::maths::Flags;

/// "Random number seed ? ".
const MSG_SEED: u16 = 0xD543;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Dispatch
    // ===============================================================================

    /// Run the function routine at ROM address `routine` (from one of the function
    /// tables).
    pub fn call_function(&mut self, routine: u16) -> Res {
        match routine {
            // 71..7F: own arguments
            0xF8BA => self.fn_bin(),
            0xF8EA => self.fn_dec(),
            0xF8C4 => self.fn_hex(),
            0xFAA1 => self.fn_instr(),
            0xF93C => self.fn_left(),
            0xD1EE => self.max_min(1),
            0xD1EA => self.max_min(0xFF),
            0xC276 => self.fn_pos(),
            0xF943 => self.fn_right(),
            0xD219 => self.fn_round(),
            0xFA36 => self.fn_string(),
            0xC4E9 => self.fn_test(),
            0xC4EE => self.fn_testr(),
            0xC262 => self.fn_vpos(),
            // 00..1D: argument in the accumulator
            0xFD85 => self.fn_abs(),
            0xFA10 => self.fn_asc(),
            0xD53E => self.maths_function(REAL_ATN),
            0xFA16 => self.fn_chr(),
            0xFE8D => self.acc_to_int().map(|_| ()),
            0xD534 => self.maths_function(REAL_COS),
            0xFEEC => self.acc_to_real(),
            0xD520 => self.maths_function(REAL_EXP),
            0xFDE8 => self.fn_fix(),
            0xFC2D => self.fn_fre(),
            0xD409 => self.fn_inkey(),
            0xF16D => self.fn_inp(),
            0xFDED => self.fn_int(),
            0xD423 => self.fn_joy(),
            0xFA0A => self.fn_len(),
            0xD52A => self.maths_function(REAL_LN),
            0xD525 => self.maths_function(REAL_LOG10),
            0xF834 => self.fn_lower(),
            0xF158 => self.fn_peek(),
            0xC99F => self.fn_remain(),
            0xFF02 => self.fn_sgn(),
            0xD52F => self.maths_function(REAL_SIN),
            0xFA57 => self.fn_space(),
            0xD329 => self.fn_sq(),
            0xD4EF => self.maths_function(REAL_SQRT),
            0xF91E => self.fn_str(),
            0xD539 => self.maths_function(REAL_TAN),
            0xFEC2 => self.acc_to_unsigned().map(|_| ()),
            0xF842 => self.fn_upper(),
            0xFA77 => self.fn_val(),
            // 40..48: no argument
            0xC417 => self.fn_eof(),
            0xD0DC => self.fn_err(),
            0xD0F4 => self.fn_himem(),
            0xFA24 => self.fn_inkey_string(),
            0xD4DB => self.fn_pi(),
            0xD584 => self.fn_rnd(),
            0xD0E5 => self.fn_time(),
            0xD107 => self.fn_xpos(),
            0xD10E => self.fn_ypos(),
            // CEAB, the unused function 7E: Improper argument.
            _ => Err(Exit::Error(5)),
        }
    }

    // ===============================================================================
    // Functions without arguments   (D0DC .. D116)
    // ===============================================================================

    /// `ERR` (ROM `D0DC`).
    fn fn_err(&mut self) -> Res {
        let e = self.rd(ERROR_NUMBER);
        self.set_acc_int(e as u16);                       // FF0A
        Ok(())
    }

    /// `TIME` (ROM `D0E5`): the 32-bit frame counter (1/300 s) as a real.
    fn fn_time(&mut self) -> Res {
        let (high, low) = self.kl_time_please();
        self.set_acc_u32(high, low);                      // FE7C
        Ok(())
    }

    /// `ERL` (ROM `D0EE`), reached through the operand table.
    pub fn fn_erl(&mut self) -> Res {
        let n = self.error_line_number().unwrap_or(0);    // CADF
        self.set_acc_unsigned_real(n);                    // FE60
        Ok(())
    }

    /// `HIMEM` (ROM `D0F4`).
    fn fn_himem(&mut self) -> Res {
        let h = self.rd16(HIMEM);
        self.set_acc_unsigned_real(h);
        Ok(())
    }

    /// `@variable` (ROM `D0FA`): the address of a variable's value (*Improper argument*
    /// if it does not exist yet).
    pub fn fn_address_of(&mut self) -> Res {
        let (found, addr, _) = self.find_variable_for_read()?;   // D690
        if !found { return Err(Exit::Error(5)); }         // CEAB
        self.set_acc_unsigned_real(addr);
        Ok(())
    }

    /// `XPOS` (ROM `D107`).
    fn fn_xpos(&mut self) -> Res {
        let (x, _) = self.gra_ask_cursor();
        self.set_acc_int(x);
        Ok(())
    }

    /// `YPOS` (ROM `D10E`).
    fn fn_ypos(&mut self) -> Res {
        let (_, y) = self.gra_ask_cursor();
        self.set_acc_int(y);
        Ok(())
    }

    // ===============================================================================
    // DEF FN and FN   (D117 .. D18F)
    // ===============================================================================

    /// `DEF FNname[(params)]=expression` (ROM `D117`).  *Invalid direct command* in
    /// direct mode.
    pub fn stmt_def(&mut self) -> Res {
        self.expect(0xE4)?;                               // FN
        if self.current_line_number().is_none() { return Err(Exit::Error(0x0C)); }
        let (addr, _) = self.find_fn()?;                  // D6A2
        let text = self.ptr;
        self.wr16(addr, text);
        self.skip_to_end_of_statement();                  // E8EF
        Ok(())
    }

    /// `FNname[(args)]` (ROM `D130`), reached through the operand table.
    pub fn fn_user(&mut self) -> Res {
        let (addr, t) = self.find_fn()?;
        let mut call = self.ptr;
        let mut def = self.rd16(addr);
        if def == 0 { return Err(Exit::Error(0x12)); }
        self.push_fn_frame()?;                            // DA07
        if self.rd(def) == b'(' {
            self.ptr = def;
            self.next();
            def = self.ptr;
            self.ptr = call;
            self.expect(b'(')?;
            call = self.ptr;
            loop {
                self.ptr = def;
                let value = self.add_fn_parameter()?;     // DA4B
                let pt = self.acc_type();
                def = self.ptr;
                self.ptr = call;
                self.eval()?;
                self.assign(value, pt)?;                  // D666
                if !self.comma() { break; }
                call = self.ptr;
                self.ptr = def;
                self.expect(b',')?;
                def = self.ptr;
            }
            self.expect(b')')?;
            call = self.ptr;
            self.ptr = def;
            self.expect(b')')?;
        } else {
            self.ptr = def;
        }
        self.activate_fn_frame();                         // DA27
        self.expect(0xEF)?;                               // '='
        self.eval()?;
        if !crate::is_end(self.cur()) { return Err(Exit::Error(2)); }   // D07B
        self.pop_fn_frame();                              // DA30
        if self.acc_is_string() { self.keep_string_result()?; }   // FB49
        self.ptr = call;
        self.convert_acc_to(t)                            // FED7
    }

    // ===============================================================================
    // MAX, MIN, ROUND   (D1EA .. D245)
    // ===============================================================================

    /// `MAX(a,b...)` (`keep` = 1) and `MIN` (`FF`) (ROM `D1EE`/`D1EA`).
    fn max_min(&mut self, keep: u8) -> Res {
        self.eval()?;
        while self.comma() {
            let t = self.push_acc()?;                     // FF53
            self.eval()?;
            let left = self.pop_stack(t);
            let c = self.compare_numbers(left, t)?;       // FD09
            if c != 0 && c != keep { self.copy_value(left, ACC); }   // FF4E
        }
        self.expect(b')')?;
        Ok(())
    }

    /// `ROUND(x[,digits])` (ROM `D219`): digits -39..39.
    fn fn_round(&mut self) -> Res {
        self.eval()?;
        let t = self.push_acc()?;
        let digits = if self.comma() { self.eval_int()? } else { 0 };
        self.expect(b')')?;
        if digits.wrapping_add(0x27) >= 0x4F { return Err(Exit::Error(5)); }
        let src = self.pop_stack(t);
        self.round_value(src, t, digits as u8)            // FDAF
    }

    // ===============================================================================
    // Cassette files   (D246 .. D2BF)
    // ===============================================================================

    /// `CAT` (ROM `D246`).
    pub fn stmt_cat(&mut self) -> Res {
        if !crate::is_end(self.cur()) { return Ok(()); }
        self.abandon_files();
        let buffer = self.output_file_buffer()?;          // F637
        let _ = self.cas_catalog(buffer);
        self.release_output_buffer()                      // F671
    }

    /// `OPENOUT name` (ROM `D256`).
    pub fn stmt_openout(&mut self) -> Res { self.open_output_file() }

    /// ROM `D256`: open the output file named by the string expression.
    pub fn open_output_file(&mut self) -> Res {
        let (name, len) = self.file_name()?;              // D273
        let buffer = self.output_file_buffer()?;
        let r = self.cas_out_open(name, len, buffer).map(|_| 0);
        self.open_result(r).map(|_| ())
    }

    /// `OPENIN name` (ROM `D25F`): the file must be an ASCII file (*File type error*).
    pub fn stmt_openin(&mut self) -> Res {
        let file_type = self.open_input_file()?.file_type;
        if file_type == 0x16 { Ok(()) } else { Err(Exit::Error(0x19)) }
    }

    /// ROM `D26A`: open the input file named by the string expression: its header's
    /// file type, load address and length.
    pub fn open_input_file(&mut self) -> Res<cpc464_os::cassette::CasOpen> {
        let (name, len) = self.file_name()?;
        let buffer = self.input_file_buffer()?;           // F632
        let r = self.cas_in_open(name, len, buffer);
        self.open_result(r)
    }

    /// ROM `D273`/`D285`: evaluate the file name; a leading `!` (removed) means "no
    /// messages" (`CAS NOISY`).  Returns `(address, length)` of the name.
    fn file_name(&mut self) -> Res<(u16, u8)> {
        let (mut len, mut addr) = self.eval_string()?;
        let mut quiet = 0;
        if len != 0 && self.rd(addr) == b'!' {
            addr = addr.wrapping_add(1);
            len -= 1;
            quiet = 0xFF;
        }
        self.cas_noisy(quiet);
        Ok((addr, len))
    }

    /// ROM `D27B`: ESC while opening is a *Break*; any other failure *File already open*.
    fn open_result<T>(&mut self, r: Result<T, cpc464_os::cassette::CasError>) -> Res<T> {
        match r {
            Ok(v) => Ok(v),
            Err(e) if e.zero => self.do_break().map(|n| match n {}),
            Err(_) => Err(Exit::Error(0x1B)),
        }
    }

    /// `CLOSEIN` (ROM `D298`).
    pub fn stmt_closein(&mut self) -> Res { self.close_in() }

    /// ROM `D298`.
    pub fn close_in(&mut self) -> Res {
        self.cas_in_close();
        self.release_input_buffer()                       // F66D
    }

    /// `CLOSEOUT` (ROM `D2A1`).
    pub fn stmt_closeout(&mut self) -> Res { self.close_out() }

    /// ROM `D2A1`: ESC while the last block is written is a *Break*.
    pub fn close_out(&mut self) -> Res {
        if let Err(e) = self.cas_out_close() {
            if e.zero { return self.do_break().map(|n| match n {}); }
        }
        self.release_output_buffer()                      // F671
    }

    /// ROM `D2AD`: abandon both files and give their buffers back.
    pub fn abandon_files(&mut self) {
        self.cas_in_abandon();
        let _ = self.release_input_buffer();
        self.cas_out_abandon();
        let _ = self.release_output_buffer();
    }

    // ===============================================================================
    // Keyboard   (D409 .. D4DA)
    // ===============================================================================

    /// `INKEY(key)` (ROM `D409`): -1 if the key is up, else the shift/control state.
    fn fn_inkey(&mut self) -> Res {
        let key = self.acc_to_int()?;
        if key >= 0x50 { return Err(Exit::Error(5)); }
        let (pressed, state) = self.km_test_key(key as u8);
        self.set_acc_int(if pressed { state as u16 } else { 0xFFFF });
        Ok(())
    }

    /// `JOY(0|1)` (ROM `D423`).
    fn fn_joy(&mut self) -> Res {
        let (j0, j1) = self.km_get_joystick();
        let n = self.acc_to_int()?;
        let v = match n { 0 => j0, 1 => j1, _ => return Err(Exit::Error(5)) };
        self.set_acc_int(v as u16);
        Ok(())
    }

    /// `KEY n,string` (ROM `D439`) or `KEY DEF` (`D456`).
    ///
    /// Quirk: the limit for `n` is meant to be loaded into `B` but goes into `A`, which is
    /// then overwritten; `B` still holds the high byte of this routine's address, so `n`
    /// may be up to `D3` and `KM SET EXPAND` alone decides (*Improper argument* if it
    /// refuses).
    pub fn stmt_key(&mut self) -> Res {
        if self.cur() == 0x8D { return self.key_def(); }
        let token = self.eval_byte_below(0xD4)?;          // D317
        self.expect(b',')?;
        let (len, addr) = self.eval_string()?;
        if !self.km_set_expand(token, len, addr) { return Err(Exit::Error(5)); }
        Ok(())
    }

    /// `KEY DEF key,repeat[,normal[,shift[,control]]]` (ROM `D456`).
    fn key_def(&mut self) -> Res {
        self.next();
        let key = self.eval_byte()?;
        if key >= 0x50 { return Err(Exit::Error(5)); }
        self.expect(b',')?;
        let repeat = self.eval_byte_below(2)?;
        self.km_set_repeat(key, if repeat & 1 != 0 { 0xFF } else { 0 });
        for entry in [KM_SET_TRANSLATE, KM_SET_SHIFT, KM_SET_CONTROL] {
            if !self.comma() { return Ok(()); }
            let code = self.eval_byte()?;
            self.km_set_table_entry(entry, key, code);
        }
        Ok(())
    }

    /// `SPEED KEY delay,repeat`, `SPEED INK first,second`, `SPEED WRITE 0|1` (ROM
    /// `D494`).
    pub fn stmt_speed(&mut self) -> Res {
        match self.cur() {
            0xA4 | 0xA2 => {
                let ink = self.cur() == 0xA2;
                self.next();
                let a = self.eval_nonzero_byte()?;
                self.expect(b',')?;
                let b = self.eval_nonzero_byte()?;
                if ink { self.scr_set_flashing(a, b); } else { self.km_set_delay(a, b); }
                Ok(())
            }
            0xD9 => {
                self.next();
                let fast = self.eval_byte_below(2)?;
                if fast == 1 { self.cas_set_speed(0x00A7, 0x32); } else { self.cas_set_speed(0x014E, 0x19); }
                Ok(())
            }
            _ => Err(Exit::Error(2)),
        }
    }

    /// ROM `D317`: a byte below `limit`, else *Improper argument*.
    pub fn eval_byte_below(&mut self, limit: u8) -> Res<u8> {
        let v = self.eval_byte()?;
        if v < limit { Ok(v) } else { Err(Exit::Error(5)) }
    }

    // ===============================================================================
    // Maths   (D4DB .. D5AD)
    // ===============================================================================

    /// `PI` (ROM `D4DB`).
    fn fn_pi(&mut self) -> Res {
        self.set_acc_type_real();
        self.real_pi(ACC);
        Ok(())
    }

    /// `DEG` (ROM `D4E7`).
    pub fn stmt_deg(&mut self) -> Res { self.set_angle_mode(0xFF); Ok(()) }

    /// `RAD` (ROM `D4EB`).
    pub fn stmt_rad(&mut self) -> Res { self.set_angle_mode(0); Ok(()) }

    /// `^` (ROM `D4F4`): left operand at `left` (type `lt`), right in the accumulator.
    pub fn op_power(&mut self, left: u16, lt: u8) -> Res {
        self.acc_to_real()?;                              // FEEC
        self.real_copy(SCRATCH_REAL, ACC);
        self.set_acc_from(left, lt);                      // FF4B
        self.acc_to_real()?;
        let f = self.real_power(ACC, SCRATCH_REAL);
        self.maths_result(f)
    }

    /// The one-argument maths functions (ROM `D50A`): `SQR` (`D4EF`), `EXP`, `LOG10`,
    /// `LOG`, `SIN`, `COS`, `TAN`, `ATN` (`D520 .. D53E`).
    fn maths_function(&mut self, entry: u16) -> Res {
        self.acc_to_real()?;
        let f = self.real_function(entry, ACC);
        self.maths_result(f)
    }

    /// ROM `D50D`: what a failing maths routine means.
    fn maths_result(&mut self, f: Flags) -> Res {
        if f.carry { return Ok(()); }
        if f.zero { return self.division_by_zero_error(); }   // CAEA
        if f.sign { return self.overflow_error(); }       // CAF3
        Err(Exit::Error(5))
    }

    /// `RANDOMIZE [seed]` (ROM `D559`): without a seed, ask for one until a number is
    /// typed (ESC is a *Break*).
    pub fn stmt_randomize(&mut self) -> Res {
        if !crate::is_end(self.cur()) {
            self.eval()?;
        } else {
            let text = self.ptr;
            loop {
                self.print_string(MSG_SEED)?;             // C341
                if !self.read_input_line()? { return self.do_break().map(|n| match n {}); }
                self.new_line()?;
                let Some(after) = self.parse_signed_number(INPUT_BUFFER)? else { continue };   // ECA3
                self.ptr = after;
                let c = self.skip_white();                // DD61
                if c == 0 { break; }
            }
            self.ptr = text;
        }
        self.acc_to_real()?;
        self.randomize_seed(ACC);
        Ok(())
    }

    /// `RND[(x)]` (ROM `D584`): `x` = 0 repeats the last number, `x` < 0 seeds the
    /// generator with `x` first.
    fn fn_rnd(&mut self) -> Res {
        if self.cur() == b'(' {
            self.next();
            self.eval()?;
            self.expect(b')')?;
            self.acc_to_real()?;
            let (_, f) = self.real_sign(ACC);
            if f.zero {
                self.rnd_last(ACC);
                return Ok(());
            }
            if f.sign { self.randomize_seed(ACC); }
        }
        self.set_acc_type_real();                         // FF16
        self.rnd(ACC);
        Ok(())
    }
}
