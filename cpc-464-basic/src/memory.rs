//! # Memory layout, BASIC's stack, `MEMORY`, `SYMBOL`  (ROM `F4C4 .. F7CA`)
//!
//! ## Layout
//!
//! At cold start ([`Basic::init_memory`]) BASIC gets the free memory `DE..HL` from the
//! firmware.  The bottom 303 bytes (`DE .. DE+12E`) are the buffer the tokeniser writes a
//! typed line into; the program starts after it ([`PROGRAM_BASE`]).  `HIMEM` starts at the
//! top.  Above `HIMEM`, `MEMORY` reserves user memory; the 4 KiB cassette buffer and the
//! user-defined character matrices (`SYMBOL AFTER`) are carved out *below* `HIMEM` when
//! needed, lowering it (the string space is moved down with it, see [`Basic::set_himem`]).
//!
//! ## BASIC's stack
//!
//! `AE8B .. B087` holds `FOR`, `GOSUB` and `WHILE` frames, saved operands of the
//! expression evaluator and other temporaries.  [`STACK_PTR`] points at the next free
//! byte; a zero byte at the bottom marks the end of the frames.  Each frame ends with its
//! own length... see [`crate::control`].

use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// Size of the tokenised-line buffer below the program (`012F`).
pub const TOKEN_BUFFER: u16 = 0x012F;
/// One past the last byte BASIC's stack may use.
pub const STACK_LIMIT: u16 = 0xB088;
/// Size of the cassette buffer area (input buffer at +1, output buffer at +0x801).
pub const CASSETTE_AREA: u16 = 0x1000;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Start-up   (F4C4)
    // ===============================================================================

    /// ROM `F4C4`: set up the memory layout for free memory `bottom..=top`.  `false`
    /// (carry) means there is not enough memory (the ROM then resets the machine).
    ///
    /// Quirk: a `top` inside BASIC's own workspace (`>= AC00`) skips everything and is
    /// reported as success.
    pub fn init_memory(&mut self, bottom: u16, top: u16) -> bool {
        if top >= 0xAC00 { return true; }
        self.wr16(HIMEM, top);
        self.wr16(STRINGS_TOP, top);
        self.wr16(MEMORY_TOP, top);
        self.wr16(MEMORY_BOTTOM, bottom);
        let (base, overflow) = bottom.overflowing_add(TOKEN_BUFFER);
        if overflow { return false; }
        self.wr16(PROGRAM_BASE, base);
        let (room, borrow) = top.wrapping_add(1).overflowing_sub(base);
        if borrow { return false; }
        if room >> 8 < 4 { return false; }               // at least 1 KiB
        self.wr(CASSETTE_BUFFERS, 0);
        true
    }

    // ===============================================================================
    // Moving the blocks   (F501 .. F58D)
    // ===============================================================================

    /// ROM `F501`: may `length` bytes be loaded at `start`?  Only outside BASIC's own
    /// memory (between the bottom of memory and `HIMEM`): *Memory full* otherwise.
    pub fn check_load_range(&mut self, start: u16, length: u16) -> Res {
        let bottom = self.rd16(MEMORY_BOTTOM);
        let room = self.rd16(HIMEM).wrapping_sub(bottom);
        let offset = start.wrapping_sub(bottom);
        if offset < room.wrapping_add(1) { return Err(Exit::Error(7)); }   // F73E
        let (_, carry) = offset.wrapping_sub(1).overflowing_add(length);
        if carry { return Err(Exit::Error(7)); }
        Ok(())
    }

    /// ROM `F51D`: the size of the string space.
    pub fn string_space_size(&self) -> u16 {
        self.rd16(STRINGS_TOP).wrapping_sub(self.rd16(STRINGS_BOTTOM))
    }

    /// ROM `F52C`: the program grew by `delta` (negative: shrank): move the pointers to
    /// the variables, `DEF FN`s and arrays.
    pub fn shift_after_program(&mut self, delta: u16) {
        for a in [PROGRAM_END, VARS_START] {
            let v = self.rd16(a).wrapping_add(delta);
            self.wr16(a, v);
        }
        self.shift_after_vars(delta);
    }

    /// ROM `F53A`: the simple variables grew by `delta`: move the array pointers.
    pub fn shift_after_vars(&mut self, delta: u16) {
        for a in [ARRAYS_START, ARRAYS_END] {
            let v = self.rd16(a).wrapping_add(delta);
            self.wr16(a, v);
        }
    }

    /// ROM `F549`: park the variables and arrays just below the string space (for
    /// `CHAIN MERGE`, which replaces the program but keeps the variables).  Returns
    /// `(size of the simple variables, size of everything moved)`; the variable area is
    /// then empty.
    pub fn park_arrays(&mut self) -> (u16, u16) {
        let vars_end = self.rd16(VARS_START);
        let vars_size = self.rd16(ARRAYS_START).wrapping_sub(vars_end);
        let arrays_end = self.rd16(ARRAYS_END);
        let size = arrays_end.wrapping_sub(vars_end);
        let bottom = self.rd16(STRINGS_BOTTOM);
        let mut dst = bottom;
        if size != 0 {
            self.lddr(arrays_end.wrapping_sub(1), bottom, size);
            dst = bottom.wrapping_sub(size);
        }
        self.wr16(STRINGS_BOTTOM, dst);
        self.no_variables();                              // D5B1
        (vars_size, size)
    }

    /// ROM `F571`: bring back what [`Self::park_arrays`] parked, after the new program.
    pub fn unpark_arrays(&mut self, vars_size: u16, size: u16) {
        let end = self.rd16(PROGRAM_END);
        self.wr16(VARS_START, end);
        self.wr16(ARRAYS_START, end.wrapping_add(vars_size));
        let src = self.rd16(STRINGS_BOTTOM).wrapping_add(1);
        if size != 0 { self.ldir(src, end, size); }
        self.wr16(STRINGS_BOTTOM, src.wrapping_add(size).wrapping_sub(1));
        self.wr16(ARRAYS_END, end.wrapping_add(size));
    }

    // ===============================================================================
    // BASIC's stack   (F58E .. F5C9)
    // ===============================================================================

    /// ROM `F58E`: empty BASIC's stack (just the zero end marker).
    pub fn reset_stack(&mut self) {
        self.wr16(STACK_PTR, STACK_BASE);
        let p = self.stack_alloc_unchecked(1);
        self.wr(p, 0);
    }

    /// ROM `F5A0`: drop `n` bytes from BASIC's stack; returns the new top (the address of
    /// the dropped data).
    pub fn pop_stack(&mut self, n: u8) -> u16 {
        let sp = self.rd16(STACK_PTR);
        if n == 0 { return sp; }
        let p = sp.wrapping_sub(n as u16);
        self.wr16(STACK_PTR, p);
        p
    }

    /// ROM `F5AC`: set BASIC's stack pointer.
    pub fn set_stack(&mut self, p: u16) { self.wr16(STACK_PTR, p); }

    /// ROM `F5B0`: reserve `n` bytes on BASIC's stack; returns their address.  If the
    /// stack would reach `B088` it is emptied and *Memory full* raised.
    pub fn stack_alloc(&mut self, n: u8) -> Res<u16> {
        let p = self.stack_alloc_unchecked(n);
        if self.rd16(STACK_PTR) >= STACK_LIMIT {
            self.reset_stack();
            return Err(Exit::Error(7));                   // F73E
        }
        Ok(p)
    }

    fn stack_alloc_unchecked(&mut self, n: u8) -> u16 {
        let p = self.rd16(STACK_PTR);
        self.wr16(STACK_PTR, p.wrapping_add(n as u16));
        p
    }

    // ===============================================================================
    // String space and free memory   (F5CA .. F631)
    // ===============================================================================

    /// ROM `F5CA`: an empty string space.
    pub fn empty_string_space(&mut self) {
        let top = self.rd16(STRINGS_TOP);
        self.wr16(STRINGS_BOTTOM, top);
    }

    /// ROM `F5D1`: take `n` bytes from the bottom of the string space, collecting garbage
    /// once if necessary; *String space full* otherwise.  Returns the address.
    pub fn alloc_string_space(&mut self, n: u8) -> Res<u16> {
        if let Some(a) = self.try_alloc_string_space(n) { return Ok(a); }
        self.collect_garbage();                           // FC3E
        if let Some(a) = self.try_alloc_string_space(n) { return Ok(a); }
        Err(Exit::Error(0x0E))
    }

    /// ROM `F5E6`: `Some(address)` if `n` bytes fit above the arrays.
    fn try_alloc_string_space(&mut self, n: u8) -> Option<u16> {
        let new_bottom = self.rd16(STRINGS_BOTTOM).wrapping_sub(n as u16);
        if new_bottom < self.rd16(ARRAYS_END) { return None; }
        self.wr16(STRINGS_BOTTOM, new_bottom);
        Some(new_bottom.wrapping_add(1))
    }

    /// ROM `F5F8`/`F5FB`: open a gap of `size` bytes at `at` by moving everything from `at`
    /// up to `end` (default the end of the arrays) up.  Collects garbage if the string
    /// space is in the way; *Memory full* if there is still no room.  Returns the new end.
    /// (The caller adjusts the pointers.)
    pub fn open_gap(&mut self, size: u16, at: u16, end: u16) -> Res<u16> {
        let (new_end, overflow) = end.overflowing_add(size);
        if overflow { return Err(Exit::Error(7)); }
        if self.rd16(STRINGS_BOTTOM) < new_end {
            self.collect_garbage();
            if self.rd16(STRINGS_BOTTOM) < new_end { return Err(Exit::Error(7)); }
        }
        let count = end.wrapping_sub(at);
        if count != 0 { self.lddr(end.wrapping_sub(1), new_end.wrapping_sub(1), count); }
        Ok(new_end)
    }

    /// [`Self::open_gap`] ending at [`ARRAYS_END`] (ROM `F5F8`).
    pub fn open_gap_in_vars(&mut self, size: u16, at: u16) -> Res<u16> {
        let end = self.rd16(ARRAYS_END);
        self.open_gap(size, at, end)
    }

    /// ROM `F628`: free memory between the arrays and the strings.
    pub fn free_memory(&self) -> u16 {
        self.rd16(STRINGS_BOTTOM).wrapping_sub(self.rd16(ARRAYS_END))
    }

    // ===============================================================================
    // Cassette buffers   (F632 .. F69C)
    // ===============================================================================

    /// ROM `F632`: the 2 KiB buffer for reading a file (allocating the cassette area if
    /// needed).
    pub fn input_file_buffer(&mut self) -> Res<u16> { self.file_buffer(0x00, 0x01) }

    /// ROM `F637`: the 2 KiB buffer for writing a file.
    pub fn output_file_buffer(&mut self) -> Res<u16> { self.file_buffer(0x08, 0x02) }

    /// ROM `F63A`: allocate the 4 KiB cassette area below `HIMEM` the first time a buffer
    /// is wanted; mark buffer `flag` in use; returns `area + 1 + offset * 256`.
    fn file_buffer(&mut self, offset: u8, flag: u8) -> Res<u16> {
        let mut state = self.rd(CASSETTE_BUFFERS);
        if state == 0 {
            let himem = self.allocate_below_himem(CASSETTE_AREA, 0)?;   // F743
            self.wr16(CASSETTE_BUFFER, himem);
            let top = self.rd16(MEMORY_TOP);
            self.wr16(CASSETTE_SAVED_TOP, top);
            self.wr16(MEMORY_TOP, himem);
            state = 0x04;
        }
        self.wr(CASSETTE_BUFFERS, state | flag);
        Ok(self.rd16(CASSETTE_BUFFER).wrapping_add(1).wrapping_add((offset as u16) << 8))
    }

    /// ROM `F66D`: the input buffer is no longer needed.
    pub fn release_input_buffer(&mut self) -> Res { self.release_buffer(0xFE) }
    /// ROM `F671`: the output buffer is no longer needed.
    pub fn release_output_buffer(&mut self) -> Res { self.release_buffer(0xFD) }
    /// ROM `F675`: if neither buffer is in use, give the cassette area back (only possible
    /// while `HIMEM` is still right below it).
    pub fn release_cassette_area(&mut self) -> Res { self.release_buffer(0xFF) }

    /// ROM `F677`.
    fn release_buffer(&mut self, mask: u8) -> Res {
        let state = self.rd(CASSETTE_BUFFERS) & mask;
        self.wr(CASSETTE_BUFFERS, state);
        if state != 0x04 { return Ok(()); }
        let area = self.rd16(CASSETTE_BUFFER);
        if self.free_block_at_himem(area, CASSETTE_AREA)? {   // F72E
            self.wr(CASSETTE_BUFFERS, 0);
            let top = self.rd16(CASSETTE_SAVED_TOP);
            self.wr16(MEMORY_TOP, top);
        }
        Ok(())
    }

    // ===============================================================================
    // SYMBOL   (F69D .. F72D)
    // ===============================================================================

    /// `SYMBOL char,row1[,row2...]` (ROM `F69D`), or `SYMBOL AFTER n`.
    pub fn stmt_symbol(&mut self) -> Res {
        if self.cur() == 0x80 { return self.symbol_after_statement(); }   // AFTER
        let ch = self.eval_byte()?;
        self.expect(b',')?;
        let mut rows = [0u8; 8];
        let mut i = 0;
        loop {
            rows[i] = self.eval_byte()?;
            i += 1;
            if i == 8 || !self.comma() { break; }
        }
        let (addr, user) = self.txt_get_matrix(ch);
        if !user { return Err(Exit::Error(5)); }          // F729
        for (k, r) in rows.iter().enumerate() { self.wr(addr + k as u16, *r); }
        Ok(())
    }

    /// ROM `F6CD`: `SYMBOL AFTER n` (0..256): remove the current user table (only if it is
    /// still directly below `HIMEM`, else *Improper argument*) and make a new one.
    fn symbol_after_statement(&mut self) -> Res {
        self.next();
        let n = self.eval_int()?;
        if n > 0x100 { return Err(Exit::Error(5)); }
        if let Some((first, table)) = self.txt_get_m_table() {
            let size = (0x100 - first as u16) * 8;
            if !self.free_block_at_himem(table.wrapping_sub(1), size)? { return Err(Exit::Error(5)); }
            let top = self.rd16(SYMBOL_SAVED_TOP);
            self.wr16(MEMORY_TOP, top);
            self.release_cassette_area()?;                // F675
            // No user table (DE = 256; HL is whatever was loaded last).
            self.txt_set_m_table(0x0100, top);
        }
        self.symbol_after(n)
    }

    /// ROM `F706`: allocate a user character table for characters `n..255` below
    /// `HIMEM` (not below `4000`) and install it.  `n = 256`: no table.
    pub fn symbol_after(&mut self, n: u16) -> Res {
        let count = 0x100u16.wrapping_sub(n);
        if count == 0 { return Ok(()); }
        let himem = self.allocate_below_himem(count * 8, 0x4000)?;   // F743
        let top = self.rd16(MEMORY_TOP);
        self.wr16(SYMBOL_SAVED_TOP, top);
        self.wr16(MEMORY_TOP, himem);
        self.txt_set_m_table(n, himem.wrapping_add(1));
        Ok(())
    }

    // ===============================================================================
    // HIMEM and MEMORY   (F72E .. F7CA)
    // ===============================================================================

    /// ROM `F72E`: if `HIMEM` is `at` (i.e. the block of `size` bytes above it is the
    /// last one carved out), raise `HIMEM` back over it.  `true` = done.
    fn free_block_at_himem(&mut self, at: u16, size: u16) -> Res<bool> {
        if self.rd16(HIMEM) != at { return Ok(false); }
        let new = at.wrapping_add(size);
        self.wr16(MEMORY_TOP, new);
        self.set_himem(new)?;                             // F750
        Ok(true)
    }

    /// ROM `F743`: lower `HIMEM` by `size` bytes (not below `limit`); returns the new
    /// `HIMEM` (the block is `HIMEM+1 .. old HIMEM`).
    fn allocate_below_himem(&mut self, size: u16, limit: u16) -> Res<u16> {
        let new = self.rd16(HIMEM).wrapping_sub(size);
        if new < limit { return Err(Exit::Error(7)); }
        self.set_himem(new)?;
        Ok(new)
    }

    /// `MEMORY address` (ROM `F4EF`).
    pub fn stmt_memory(&mut self) -> Res {
        self.collect_garbage();                           // FC3E
        let a = self.eval_address()?;
        self.set_himem(a)?;
        self.release_cassette_area()?;                    // F675
        self.wr16(HIMEM, a);
        Ok(())
    }

    /// ROM `F750`: move `HIMEM` to `new`, moving the string space with it.  *Memory
    /// full* if `new` is above [`MEMORY_TOP`] or leaves no room for the variables and
    /// strings.
    pub fn set_himem(&mut self, new: u16) -> Res {
        self.collect_garbage();
        if self.rd16(MEMORY_TOP) < new { return Err(Exit::Error(7)); }
        let size = self.string_space_size();
        let (need, overflow) = self.rd16(ARRAYS_END).overflowing_add(size);
        if overflow { return Err(Exit::Error(7)); }
        if need.wrapping_sub(1) >= new { return Err(Exit::Error(7)); }
        let delta = new.wrapping_sub(self.rd16(HIMEM));
        self.wr16(MOVE_DELTA, delta);
        self.for_each_string(StringVisitor::Relocate)?;   // DA74 with F7BB
        let delta = self.rd16(MOVE_DELTA);
        if delta & 0x8000 != 0 {
            // F799: down
            let bottom = self.rd16(STRINGS_BOTTOM);
            let new_bottom = bottom.wrapping_add(delta);
            let size = self.string_space_size();
            if size != 0 { self.ldir(bottom.wrapping_add(1), new_bottom.wrapping_add(1), size); }
            self.wr16(STRINGS_TOP, new_bottom.wrapping_add(size));
            self.wr16(STRINGS_BOTTOM, new_bottom);
        } else if delta != 0 {
            let top = self.rd16(STRINGS_TOP);
            let new_top = top.wrapping_add(delta);
            let size = self.string_space_size();
            if size != 0 { self.lddr(top, new_top, size); }
            self.wr16(STRINGS_TOP, new_top);
            let bottom = self.rd16(STRINGS_BOTTOM).wrapping_add(delta);
            self.wr16(STRINGS_BOTTOM, bottom);
        }
        self.wr16(HIMEM, new);
        Ok(())
    }

    /// ROM `F7BB`: a string descriptor callback for [`Self::set_himem`]: a string in the
    /// string space (above the program) moves by [`MOVE_DELTA`].  `de` = address of the
    /// descriptor's high address byte, `bc` = the string's address.
    pub fn relocate_string(&mut self, de: u16, bc: u16) {
        if self.rd16(PROGRAM_END) >= bc { return; }
        let new = self.rd16(MOVE_DELTA).wrapping_add(bc);
        self.wr16(de.wrapping_sub(1), new);
    }
}

/// The routines the ROM passes to its "for every string" walker (`DA74`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StringVisitor {
    /// `F7BB`: relocate after `MEMORY`.
    Relocate,
    /// `FC9C`: garbage collection: find the highest string not yet moved.
    FindHighest,
    /// `FB2E`: copy strings that live outside the string space into it (`CHAIN`).
    CopyIn,
}
