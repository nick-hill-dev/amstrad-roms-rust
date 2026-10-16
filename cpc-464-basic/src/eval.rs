//! # The expression evaluator  (ROM `CE67 .. D18F`)
//!
//! An expression is evaluated into the *accumulator* (`B0C1` type, `B0C2..` value, see
//! [`crate::arith`]).  The evaluator is an operator-precedence parser (`CF07`): it fetches
//! an operand, then, while the next token is a binary operator binding tighter than the
//! current priority, saves the left value on BASIC's stack, evaluates the right-hand side
//! at the operator's priority and applies the operator.
//!
//! ## Priorities
//!
//! | priority | operators                                   |
//! |----------|---------------------------------------------|
//! | `16`     | `^`                                         |
//! | `14`     | unary `-` (its operand)                     |
//! | `12`     | `*` `/`                                     |
//! | `10`     | `\`                                         |
//! | `0E`     | `MOD`                                       |
//! | `0C`     | `+` `-`                                     |
//! | `0A`     | comparisons `=` `<>` `<` `>` `<=` `>=`      |
//! | `08`     | `NOT` (its operand)                         |
//! | `06`     | `AND`                                       |
//! | `04`     | `OR`                                        |
//! | `02`     | `XOR`                                       |
//!
//! The binary operators are the tokens `F4..FD` in the order `+ - * / ^ \ AND MOD OR XOR`;
//! their priorities and routines are the table at `CF81` (a priority byte followed by a
//! `JP routine`, four bytes per operator).  The comparisons (`EE..F3`) are handled
//! separately; for numbers they borrow a fake table entry at `CFA9` whose "priority" is
//! the opcode byte `0A` of the code that follows (`LD A,(BC)`).
//!
//! String `+` is special: its right operand is a single operand, not an expression.
//!
//! ## Text pointer conventions
//!
//! [`Basic::eval`] (`CEFB`) is entered with the text pointer on the first character of
//! the expression and leaves it on the first non-space character after it (the ROM steps
//! back one byte and calls `DD3F`, so a statement handler can test the result like the
//! character fetched by `DD3F`).

use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// Base of the binary operator table (`CF81`): priority byte + `JP routine`.
const OPERATOR_TABLE: u16 = 0xCF81;
/// The fake operator entry used for numeric comparisons (`CFA9`).
const COMPARE_ENTRY: u16 = 0xCFA9;
/// The operand dispatch table (`CFF2`): count, default routine, `(char, routine)`s.
pub const OPERAND_TABLE: u16 = 0xCFF2;
/// A zero-length string descriptor in ROM (`D02B`), the value of an unknown string
/// variable.
pub const EMPTY_STRING: u16 = 0xD02B;
/// The function tables (`D190`: functions with an argument list, `D0CA`: functions
/// without).
pub const FUNCTION_TABLE: u16 = 0xD190;
pub const NO_ARG_FUNCTION_TABLE: u16 = 0xD0CA;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Typed evaluation   (CE67 .. CEAD)
    // ===============================================================================

    /// ROM `CE67`: evaluate an integer `0..255`, else *Improper argument*.
    pub fn eval_byte(&mut self) -> Res<u8> {
        let v = self.eval_int()?;
        if v >> 8 != 0 { return Err(Exit::Error(5)); }
        Ok(v as u8)
    }

    /// ROM `CE6D`: evaluate an integer `1..255`, else *Improper argument*.
    pub fn eval_nonzero_byte(&mut self) -> Res<u8> {
        let v = self.eval_int()?;
        if v == 0 || v >> 8 != 0 { return Err(Exit::Error(5)); }
        Ok(v as u8)
    }

    /// ROM `CE7C`: evaluate an integer `0..32767`, else *Improper argument*.
    pub fn eval_positive_int(&mut self) -> Res<u16> {
        let v = self.eval_int()?;
        if v & 0x8000 != 0 { return Err(Exit::Error(5)); }
        Ok(v)
    }

    /// ROM `CE86`: evaluate a 16-bit signed integer (`CINT`, *Overflow* if it does not
    /// fit).
    pub fn eval_int(&mut self) -> Res<u16> {
        self.eval()?;
        self.acc_to_int()                                 // FE8D
    }

    /// ROM `CE91`: evaluate an address: a number `-32768..65535` as an unsigned 16-bit
    /// value (`UNT`).
    pub fn eval_address(&mut self) -> Res<u16> {
        self.eval()?;
        self.acc_to_unsigned()                            // FEC2
    }

    /// ROM `CE9F`: evaluate a string; returns `(length, address)` of its text (the
    /// string is released from the descriptor stack, see [`Basic::pop_string`]).
    pub fn eval_string(&mut self) -> Res<(u8, u16)> {
        self.eval()?;
        self.pop_string()                                 // FBDA
    }

    /// ROM `CEA5`: evaluate a string and leave its descriptor in the accumulator.
    pub fn eval_string_keep(&mut self) -> Res {
        self.eval()?;
        self.require_string()                             // FF3C
    }

    /// ROM `CEB0`: a line-number range `[a][-[b]]` (for `LIST`, `DELETE`): returns
    /// `(from, to)`, defaults `1` and `65535`; a single number is a range of one line.
    /// Stops at a comma (which is consumed) or a `#`.
    pub fn line_range(&mut self) -> Res<(u16, u16)> {
        let mut from: u16 = 1;
        let mut to: u16 = 0xFFFF;
        if self.comma() || crate::is_end(self.cur()) { return Ok((from, to)); }
        let c = self.cur();
        if c == b'#' { return Ok((from, to)); }
        if c != 0xF5 {
            // a start line
            to = self.read_line_number_constant()?;       // CEE1
            from = to;
            if crate::is_end(self.cur()) || self.comma() { return Ok((from, to)); }
        }
        // CECE: "-"
        self.expect(0xF5)?;
        to = 0xFFFF;
        if crate::is_end(self.cur()) || self.comma() { return Ok((from, to)); }
        to = self.read_line_number_constant()?;
        if !crate::is_end(self.cur()) { self.comma(); }
        Ok((from, to))
    }

    /// ROM `CEE1`: a line number in the program text (token `1E` number, or `1D` the
    /// address of the line, whose number is fetched).  Anything else: *Syntax error*.
    /// The text pointer must be on the token; it is left on the next non-space character.
    pub fn read_line_number_constant(&mut self) -> Res<u16> {
        let t = self.cur();
        let v = self.rd16(self.ptr.wrapping_add(1));
        self.ptr = self.ptr.wrapping_add(2);
        let n = match t {
            0x1E => v,
            0x1D => self.rd16(v.wrapping_add(3)),
            _ => return Err(Exit::Error(2)),              // D07B
        };
        self.next();
        Ok(n)
    }

    // ===============================================================================
    // The precedence parser   (CEFB .. CFCA)
    // ===============================================================================

    /// ROM `CEFB`: evaluate the expression at the text pointer into the accumulator.
    pub fn eval(&mut self) -> Res {
        self.ptr = self.ptr.wrapping_sub(1);
        self.eval_priority(0)?;
        self.ptr = self.ptr.wrapping_sub(1);
        self.next();
        Ok(())
    }

    /// ROM `CF07`: fetch an operand and apply operators of priority above `prio`.
    /// Entered with the text pointer one byte *before* the operand; leaves it on the
    /// first character that is not part of the expression.
    pub fn eval_priority(&mut self, prio: u8) -> Res {
        self.operand()?;                                  // CFCB
        loop {
            let t = self.cur();
            if !(0xEE..0xFE).contains(&t) { return Ok(()); }
            if t >= 0xF4 {
                if t == 0xF4 && self.acc_is_string() {
                    // String concatenation: the right side is one operand.
                    let left = self.rd16(ACC);
                    self.operand()?;
                    self.require_string()?;
                    self.concatenate(left)?;              // F863
                    continue;
                }
                let entry = OPERATOR_TABLE + 4 * (t - 0xF4) as u16;
                let op_prio = self.rom_byte(entry);
                if prio >= op_prio { return Ok(()); }
                self.binary_operation(entry, op_prio, t)?;
            } else {
                // Comparisons EE..F3.
                if prio >= 0x0A { return Ok(()); }
                let relation = t - 0xED;                  // > 1, = 2, >= 3, < 4, <> 5, <= 6
                if !self.acc_is_string() {
                    let p = self.rom_byte(COMPARE_ENTRY);  // 0x0A, see above
                    self.binary_operation(COMPARE_ENTRY, p, relation)?;
                } else {
                    let left = self.rd16(ACC);
                    self.eval_priority(0x0A)?;
                    let r = self.compare_strings(left)?;  // F897
                    self.set_relation_result(r, relation); // CFAF
                }
            }
        }
    }

    /// ROM `CF42`: push the left value, evaluate the right side at `op_prio`, pop the left
    /// value and apply the operator (`JP` at `entry + 1`).  `arg` is the operator token
    /// (or the relation code for a comparison).
    fn binary_operation(&mut self, entry: u16, op_prio: u8, arg: u8) -> Res {
        let left_type = self.push_acc()?;                 // FF53
        self.eval_priority(op_prio)?;
        let left = self.pop_stack(left_type);             // F5A0
        if entry == COMPARE_ENTRY {
            // CFAA: compare, then turn into -1 / 0.
            let r = self.compare_numbers(left, left_type)?; // FD09
            self.set_relation_result(r, arg);
            return Ok(());
        }
        match arg {
            0xF4 => self.op_add(left, left_type),         // FCCC
            0xF5 => self.op_subtract(left, left_type),    // FCE1
            0xF6 => self.op_multiply(left, left_type),    // FCF5
            0xF7 => self.op_divide(left, left_type),      // FD12
            0xF8 => self.op_power(left, left_type),       // D4F4
            0xF9 => self.op_int_divide(left, left_type),  // FD37
            0xFA => self.op_and(left, left_type),         // FD58
            0xFB => self.op_mod(left, left_type),         // FD49
            0xFC => self.op_or(left, left_type),          // FD63
            _ => self.op_xor(left, left_type),            // FD6D
        }
    }

    /// ROM `CFAF`: turn a comparison result (`FF` less, `0` equal, `1` greater - of the
    /// right operand against the left, see [`Basic::compare_numbers`]) and the relation
    /// mask into the integer -1 (true) or 0 (false).
    ///
    /// The result is mapped to a bit (less 1, equal 2, greater 4) and ANDed with the
    /// relation code, whose bits are exactly the outcomes that make it true.
    fn set_relation_result(&mut self, cmp: u8, relation: u8) {
        let (a, c) = cmp.overflowing_add(1);              // ADD A,1
        let bit = a.wrapping_add(a).wrapping_add(c as u8); // ADC A,A
        let v = if bit & relation != 0 { 0xFF } else { 0 };
        self.set_acc_signed_byte(v);                      // FF05
    }

    /// `-x` (ROM `CFB9`): the operand is evaluated at priority `14`.
    fn unary_minus(&mut self) -> Res {
        self.ptr = self.ptr.wrapping_sub(1);
        self.eval_priority(0x14)?;
        self.negate_acc()                                 // FD89
    }

    /// `NOT x` (ROM `CFC2`): the operand is evaluated at priority `08`.
    fn unary_not(&mut self) -> Res {
        self.ptr = self.ptr.wrapping_sub(1);
        self.eval_priority(0x08)?;
        self.not_acc()                                    // FD77
    }

    // ===============================================================================
    // Operands   (CFCB .. D07F)
    // ===============================================================================

    /// ROM `CFCB`: one operand: a variable, a number, a string literal, a function, or
    /// something from the operand table (`-`, `+`, `(`, `NOT`, `ERL`, `FN`, `MID$`, `@`).
    pub fn operand(&mut self) -> Res {
        let c = self.next();
        if crate::is_end(c) { return Err(Exit::Error(0x16)); }   // Operand missing
        if c < 0x0E { return self.variable_operand(c); }        // D00D
        if c < 0x20 { return self.number_operand(c); }          // D02C
        if c == b'"' { return self.string_literal(); }          // F7CB
        if c == 0xFF { return self.function_operand(); }        // D080
        let routine = self.table_lookup(OPERAND_TABLE, c);      // FF93
        self.next();
        self.operand_routine(routine)
    }

    /// Run an entry of the operand table.
    fn operand_routine(&mut self, routine: u16) -> Res {
        match routine {
            0xCFB9 => self.unary_minus(),
            0xCFCE => {
                // unary plus: just fetch the next operand (the ROM re-enters CFCB after
                // its DD3F, so the pointer is on the character after '+').
                if crate::is_end(self.cur()) { return Err(Exit::Error(0x16)); }
                self.ptr = self.ptr.wrapping_sub(1);
                self.operand()
            }
            0xD070 => self.parenthesised(),
            0xCFC2 => self.unary_not(),
            0xD0EE => self.fn_erl(),
            0xD130 => self.fn_user(),
            0xF94B => self.fn_mid(),
            0xD0FA => self.fn_address_of(),
            _ => {
                // D078: let an extension have a go, then Syntax error.
                self.call_hook(HOOK_BAD_OPERAND);
                Err(Exit::Error(2))
            }
        }
    }

    /// ROM `D00D`: a variable as an operand.  A string variable's value is the address of
    /// its descriptor; numbers are copied into the accumulator.  A variable that does not
    /// exist yet reads as 0 or the empty string.
    fn variable_operand(&mut self, _token: u8) -> Res {
        let (found, addr, t) = self.find_variable_for_read()?;  // D690
        if t == T_STR {
            self.wr16(ACC, if found { addr } else { EMPTY_STRING });
            return Ok(());
        }
        if found {
            self.set_acc_from(addr, t);                   // FF4B
        } else {
            self.zero_acc();                              // FEF3
        }
        Ok(())
    }

    /// ROM `D02C`: a numeric constant.  `0E..17` are the digits 0..9, `19` an 8-bit
    /// integer, `1A..1C` 16-bit integers (decimal, binary, hex), `1D`/`1E` a line
    /// address / line number (evaluated as an unsigned real), `1F` a real.
    fn number_operand(&mut self, token: u8) -> Res {
        let k = token - 0x0E;
        if k < 0x0A {
            self.set_acc_int(k as u16);
        } else {
            self.ptr = self.ptr.wrapping_add(1);
            if k == 0x0B {
                let v = self.cur();
                self.set_acc_int(v as u16);
            } else if k < 0x0F {
                let v = self.rd16(self.ptr);
                self.ptr = self.ptr.wrapping_add(1);
                self.set_acc_int(v);
            } else if k < 0x11 {
                let mut v = self.rd16(self.ptr);
                self.ptr = self.ptr.wrapping_add(1);
                if k == 0x0F { v = self.rd16(v.wrapping_add(3)); }   // line address
                self.set_acc_unsigned_real(v);            // FE60
            } else if k == 0x11 {
                self.set_acc_from(self.ptr, T_REAL);      // FF4B
                self.ptr = self.ptr.wrapping_add(4);
            } else {
                return Err(Exit::Error(2));               // D07B
            }
        }
        self.next();
        Ok(())
    }

    /// ROM `D070`: `( expression )`.
    fn parenthesised(&mut self) -> Res {
        self.eval()?;
        self.expect(b')')?;
        Ok(())
    }

    /// ROM `D080`: a function: token `FF` followed by the function number.
    ///
    /// * `40..48` (`EOF`, `ERR`, `HIMEM`, `INKEY$`, `PI`, `RND`, `TIME`, `XPOS`, `YPOS`)
    ///   take no argument list (`RND` looks for its own);
    /// * the others need `(`; `00..1D` get their single argument evaluated here, `71..7F`
    ///   parse their own arguments and the closing bracket.
    fn function_operand(&mut self) -> Res {
        self.ptr = self.ptr.wrapping_add(1);
        let f = self.cur();
        self.next();
        if (0x40..0x49).contains(&f) {
            let routine = self.rom_word(NO_ARG_FUNCTION_TABLE + 2 * (f - 0x40) as u16);
            return self.call_function(routine);
        }
        self.expect(b'(')?;
        let c = f.wrapping_add(f).wrapping_add(0x1E);     // table offset
        if c >= 0x59 {
            self.call_hook(HOOK_BAD_FUNCTION);            // D0A9
            return Err(Exit::Error(2));
        }
        let routine = self.rom_word(FUNCTION_TABLE + c as u16);
        if c < 0x1D {
            return self.call_function(routine);
        }
        // One argument: evaluate "expression )" first; the function itself does not move
        // the text pointer.
        self.parenthesised()?;
        let p = self.ptr;
        self.call_function(routine)?;
        self.ptr = p;
        Ok(())
    }
}
