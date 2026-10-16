//! # Variables and arrays  (ROM `D5AE .. DAF7`)
//!
//! ## In the program text
//!
//! The tokeniser turns a variable name into
//!
//! ```text
//!   token  offset(2)  name...   (last character + 0x80)
//! ```
//!
//! where the token gives the type: `02` `%`, `03` `$`, `04` `!` written after the name,
//! or `0B`/`0C`/`0D` for a name without a suffix - its type comes from the
//! `DEFINT`/`DEFSTR`/`DEFREAL` table.  The first time such a reference is looked up, the
//! token is rewritten to `0B` (integer), `0C` (string) or `0D` (real) according to the
//! table at that moment.  The offset starts as 0; once the variable has been found it
//! caches the variable's position, so later executions of the same reference skip the
//! search.  (`CLEAR`, `ERASE`, editing the program ... zero the offsets again, see
//! [`Basic::forget_variable_offsets`].)
//!
//! ## In memory
//!
//! Simple variables and `DEF FN`s live between [`VARS_START`] and [`ARRAYS_START`];
//! arrays between [`ARRAYS_START`] and [`ARRAYS_END`].  Each lives on a singly linked
//! list: one list per initial letter ([`VAR_HEADS`]), one for the `DEF FN`s
//! ([`FN_HEAD`]), one per array type ([`ARRAY_HEADS`]).  New entries go at the front of
//! their list.  A link (and a cached offset) is relative to `VARS_START - 1` or
//! `ARRAYS_START - 1`, so the areas can move without fixing the links; 0 ends a list.
//!
//! ```text
//!   simple variable:  link(2) name... type-1  value(type bytes)
//!   DEF FN:           link(2) name... 0x40|1  address of the definition(2)
//!   array:            link(2) name... type-1  length(2) dims(1) size(2)*dims  elements
//! ```
//!
//! Names are stored with bit 5 of every character cleared (upper case for letters, which
//! also changes digits and `.` - harmless, as only the comparison matters) and the last
//! character marked with bit 7.  An array dimension holds its number of elements (`DIM
//! a(10)` stores 11); the dimensions are stored last subscript first.  The element offset
//! is computed with `INT MULTIPLY UNSIGNED` without an overflow check.
//!
//! ## While looking a name up
//!
//! The name is first copied (with bit 5 cleared) onto BASIC's stack and [`VAR_NAME`]
//! points at the copy.  The copy is dropped again when the routine that started the
//! lookup returns ([`Basic::with_name`]); an error leaves it (the error handler resets
//! the stack anyway).
//!
//! ## `DEF FN` parameters
//!
//! Calling `FN` pushes a frame on BASIC's stack ([`Basic::push_fn_frame`]); each
//! parameter becomes a little variable entry inside the frame.  While the function body
//! is evaluated ([`FN_ACTIVE_FRAME`]), every simple-variable search looks at the frame's
//! parameters first.

use crate::memory::StringVisitor;
use crate::vars::*;
use crate::{Basic, Exit, Res};
use cpc464_os::Hardware;

/// The first step of a lookup ([`Basic::scan_name`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Name {
    /// The reference in the text already carries this cached offset.
    Cached(u16),
    /// Not cached: the name has been copied to [`VAR_NAME`].  `token` is the address of
    /// the reference (where the offset will be cached), `letter` its first letter.
    New { token: u16, letter: u8 },
}

/// The "letter" whose list head is the `DEF FN` list ([`FN_HEAD`] = `AD4E + 2 * 5B`).
const FN_LETTER: u8 = 0x5B;

impl<H: Hardware> Basic<H> {
    // ===============================================================================
    // Clearing   (D5AE .. D5D8)
    // ===============================================================================

    /// ROM `D5AE`: empty all lists and the variable area.
    pub fn clear_variable_lists(&mut self) {
        self.fill(VAR_HEADS, 0x36, 0);                    // D5BE: letters and DEF FN
        self.clear_array_lists();
        self.no_variables();
    }

    /// ROM `D5B1`: no variables or arrays: all three areas start (and end) at the end of
    /// the program.
    pub fn no_variables(&mut self) {
        let end = self.rd16(PROGRAM_END);
        self.wr16(VARS_START, end);
        self.wr16(ARRAYS_START, end);
        self.wr16(ARRAYS_END, end);
    }

    /// ROM `D5C6`: empty the three array lists.
    fn clear_array_lists(&mut self) { self.fill(ARRAY_HEADS, 6, 0); }

    /// ROM `D5D2`: forget all `DEF FN`s.
    pub fn clear_fn_list(&mut self) { self.wr16(FN_HEAD, 0); }

    /// ROM `D5DB`: the list head of letter `c` (`5B` = the `DEF FN` list) and the base its
    /// links are relative to.
    fn var_list_head(&self, c: u8) -> (u16, u16) {
        let head = 0xAD4Eu16.wrapping_add(c.wrapping_add(c) as u16);
        (head, self.rd16(VARS_START).wrapping_sub(1))
    }

    /// ROM `D5EA`: the list head of arrays of type `t` and their link base.
    fn array_list_head(&self, t: u8) -> (u16, u16) {
        let head = ARRAY_HEADS.wrapping_add((t & 3).wrapping_sub(1).wrapping_mul(2) as u16);
        (head, self.rd16(ARRAYS_START).wrapping_sub(1))
    }

    // ===============================================================================
    // DEFINT, DEFSTR, DEFREAL   (D5FC .. D64E)
    // ===============================================================================

    /// ROM `D5FC`: every letter defaults to real.
    pub fn defreal_all(&mut self) {
        let _ = self.set_default_types(b'A', b'Z', T_REAL);
    }

    /// ROM `D601`: letters `first..=last` default to type `t`; *Syntax error* if the range
    /// is backwards.
    fn set_default_types(&mut self, first: u8, last: u8, t: u8) -> Res {
        let Some(n) = last.checked_sub(first) else { return Err(Exit::Error(2)) };
        let top = SCRATCH_REAL + last as u16;
        for i in 0..=n as u16 { self.wr(top - i, t); }
        Ok(())
    }

    /// `DEFSTR` (ROM `D614`).
    pub fn stmt_defstr(&mut self) -> Res { self.def_types(T_STR) }
    /// `DEFINT` (ROM `D618`).
    pub fn stmt_defint(&mut self) -> Res { self.def_types(T_INT) }
    /// `DEFREAL` (ROM `D61C`).
    pub fn stmt_defreal(&mut self) -> Res { self.def_types(T_REAL) }

    /// ROM `D61E`: `letter[-letter][,...]`.
    fn def_types(&mut self, t: u8) -> Res {
        loop {
            // (FF71 hands back the letter in upper case.)
            let first = crate::arith::to_upper(self.cur());
            if !crate::arith::is_letter(first) { return Err(Exit::Error(2)); }
            let mut last = first;
            if self.next() == b'-' {
                last = crate::arith::to_upper(self.next());
                if !crate::arith::is_letter(last) { return Err(Exit::Error(2)); }
                self.next();
            }
            self.set_default_types(first, last, t)?;
            if !self.comma() { return Ok(()); }
        }
    }

    // ===============================================================================
    // Assignment   (D64F .. D685)
    // ===============================================================================

    /// ROM `D64F`: a statement that does not start with a keyword: `|name` (an RSX) or an
    /// assignment.
    pub fn implicit_let(&mut self) -> Res {
        if self.cur() == b'|' { return self.call_rsx(); }  // F1A0
        self.stmt_let()
    }

    /// `[LET] variable = expression` (ROM `D654`).
    pub fn stmt_let(&mut self) -> Res {
        let (addr, t) = self.find_variable()?;             // D686
        self.expect(0xEF)?;                               // '='
        self.eval()?;
        self.assign(addr, t)
    }

    /// ROM `D666`: store the accumulator in the variable of type `t` at `addr`, converting
    /// it first if necessary.
    pub fn assign(&mut self, addr: u16, t: u8) -> Res {
        if self.acc_type() != t { self.convert_acc_to(t)?; }   // FED7
        self.assign_same_type(addr)
    }

    /// ROM `D66F`: store the accumulator (already of the right type) at `addr`.  A string
    /// is first made safe to keep (copied into the string space if it lives in the
    /// program text or is a temporary, see [`Basic::string_for_assignment`]).
    pub fn assign_same_type(&mut self, addr: u16) -> Res {
        if !self.acc_is_string() {
            self.copy_acc_to(addr);                       // FF62
            return Ok(());
        }
        let src = self.string_for_assignment()?;          // FB59
        self.copy_value(src, addr);                       // FF66
        Ok(())
    }

    /// `DIM name(dims)[,...]` (ROM `D67D`).
    pub fn stmt_dim(&mut self) -> Res {
        loop {
            self.dim_one()?;                              // D7B5
            if !self.comma() { return Ok(()); }
        }
    }

    // ===============================================================================
    // Finding variables   (D686 .. D6DD)
    // ===============================================================================

    /// ROM `D686`: the variable (or array element) at the text pointer, created if it
    /// does not exist: `(address of its value, type)`.
    pub fn find_variable(&mut self) -> Res<(u16, u8)> {
        self.with_name(|s, name| {
            let addr = match s.array_or_cached(name)? {    // D7DB
                Some(a) => a,
                None => s.find_or_create_simple(name)?,   // D6B8
            };
            Ok((addr, s.acc_type()))
        })
    }

    /// ROM `D690`: like [`Self::find_variable`] for reading: a simple variable that does
    /// not exist is not created: `(found, address, type)`.  (Array elements are always
    /// found - an undimensioned array is created with 11 elements per subscript.)
    pub fn find_variable_for_read(&mut self) -> Res<(bool, u16, u8)> {
        self.with_name(|s, name| {
            if let Some(a) = s.array_or_cached(name)? { return Ok((true, a, s.acc_type())); }
            let Name::New { token, letter } = name else { unreachable!() };
            let (head, base) = s.var_list_head(letter);
            let found = s.search_with_params(token, head, base);  // D6DE
            Ok((found.is_some(), found.unwrap_or(token), s.acc_type()))
        })
    }

    /// ROM `D6A2`: the `DEF FN` variable named at the text pointer (created if missing):
    /// `(address of its value - the definition's address, type)`.
    pub fn find_fn(&mut self) -> Res<(u16, u8)> {
        self.with_name(|s, name| {
            let addr = match name {
                Name::Cached(off) => s.rd16(VARS_START).wrapping_sub(1).wrapping_add(off),
                Name::New { token, .. } => {
                    let (head, base) = s.var_list_head(FN_LETTER);     // D5D9
                    match s.search_with_params(token, head, base) {
                        Some(a) => a,
                        None => {
                            // D73D: a two-byte entry flagged 0x40 in its type byte.
                            let a = s.create_variable(T_INT, token, head, base)?;
                            let f = s.rd(a - 1) | 0x40;
                            s.wr(a - 1, f);
                            a
                        }
                    }
                }
            };
            Ok((addr, s.acc_type()))
        })
    }

    /// ROM `D6B3`: a simple variable (no subscripts: the `FOR` variable), created if
    /// missing.
    pub fn find_simple_variable(&mut self) -> Res<(u16, u8)> {
        self.with_name(|s, name| {
            let addr = match name {
                Name::Cached(off) => s.rd16(VARS_START).wrapping_sub(1).wrapping_add(off),
                Name::New { .. } => s.find_or_create_simple(name)?,
            };
            Ok((addr, s.acc_type()))
        })
    }

    /// ROM `D6D6`: step over a variable reference (and its subscripts) without looking it
    /// up; returns its type.  (`INPUT` checks the syntax of its list this way.)
    pub fn skip_variable(&mut self) -> Res<u8> {
        self.with_name(|s, _| {
            s.skip_subscripts()?;                         // E8C1
            Ok(s.acc_type())
        })
    }

    /// ROM `D6B8`: search the list of the name's letter; create the variable if missing.
    fn find_or_create_simple(&mut self, name: Name) -> Res<u16> {
        let Name::New { token, letter } = name else { unreachable!() };
        let (head, base) = self.var_list_head(letter);
        match self.search_with_params(token, head, base) {
            Some(a) => Ok(a),
            None => {
                let t = self.acc_type();
                self.create_variable(t, token, head, base)   // D749
            }
        }
    }

    /// ROM `D6DE`: search for the name in [`VAR_NAME`]: first among the parameters of the
    /// active `DEF FN` frame, then in the list at `head`.  Found in the list, the offset is
    /// cached in the reference at `token`.  Returns the address of the value.
    fn search_with_params(&mut self, token: u16, head: u16, base: u16) -> Option<u16> {
        let frame = self.rd16(FN_ACTIVE_FRAME);
        if frame != 0 {
            if let Some((_, value)) = self.search_list(frame.wrapping_add(2), 0) {
                return Some(value);                       // D704: not cached
            }
        }
        let (_, value) = self.search_list(head, base)?;
        self.cache_offset(token, value, base);            // D76D
        Some(value)
    }

    /// ROM `D708`: search the list whose head is at `head` (links relative to `base`) for
    /// the name at [`VAR_NAME`] with the type in [`ACC_TYPE`]: `(entry, value address)`.
    fn search_list(&self, head: u16, base: u16) -> Option<(u16, u16)> {
        let mut link_at = head;
        loop {
            let link = self.rd16(link_at);
            if link == 0 { return None; }
            let entry = link.wrapping_add(base);
            if let Some(value) = self.entry_matches(entry) { return Some((entry, value)); }
            link_at = entry;
        }
    }

    /// The comparison of `D710..D72D`: the entry's name equals the name at [`VAR_NAME`]
    /// and its type byte (low 3 bits) is `ACC_TYPE - 1`.  Returns the value address.
    fn entry_matches(&self, entry: u16) -> Option<u16> {
        let mut e = entry.wrapping_add(2);
        let mut n = self.rd16(VAR_NAME);
        loop {
            let c = self.rd(e);
            if c != self.rd(n) { return None; }
            n = n.wrapping_add(1);
            e = e.wrapping_add(1);
            if c & 0x80 != 0 { break; }
        }
        if (self.acc_type().wrapping_sub(1) ^ self.rd(e)) & 7 != 0 { return None; }
        Some(e.wrapping_add(1))
    }

    /// ROM `D731`: the address after the name of the entry at `entry` (its type byte).
    fn skip_entry_name(&self, entry: u16) -> u16 {
        let mut p = entry.wrapping_add(2);
        loop {
            let c = self.rd(p);
            p = p.wrapping_add(1);
            if c & 0x80 != 0 { return p; }
        }
    }

    /// ROM `D749`: create a simple variable of type `t` (its size) at the end of the
    /// simple-variable area, zeroed, at the front of the list at `head`, and cache it in
    /// the reference at `token`.  Returns the value address.
    fn create_variable(&mut self, t: u8, token: u16, head: u16, base: u16) -> Res<u16> {
        let (name_len, total) = self.entry_size(t);       // D777
        let at = self.rd16(ARRAYS_START);
        self.open_gap_in_vars(total, at)?;                // F5F8
        self.shift_after_vars(total);                     // F53A
        self.write_entry_header(at, name_len);            // D78A
        let end = at.wrapping_add(total);
        let value = end.wrapping_sub(t as u16);
        self.fill(value, t as u16, 0);
        self.link_entry(head, at, base);                  // D7A5
        self.cache_offset(token, value, base);            // D76D
        Ok(value)
    }

    /// ROM `D76D`: cache `value - base` in the offset field of the reference at `token`.
    fn cache_offset(&mut self, token: u16, value: u16, base: u16) {
        self.wr16(token.wrapping_add(1), value.wrapping_sub(base));
    }

    /// ROM `D777`: `(length of the name at VAR_NAME, size of an entry with a fixed part of
    /// a bytes)`.  The size is computed in 8 bits: `a + 3 + name length`.
    fn entry_size(&self, a: u8) -> (u8, u16) {
        let mut c = a.wrapping_add(3);
        let mut n = self.rd16(VAR_NAME);
        let mut len = 0u8;
        loop {
            c = c.wrapping_add(1);
            len = len.wrapping_add(1);
            let ch = self.rd(n);
            n = n.wrapping_add(1);
            if ch & 0x80 != 0 { return (len, c as u16); }
        }
    }

    /// ROM `D78A`: write the name and the type byte of a new entry at `at`; returns the
    /// address after the type byte.
    fn write_entry_header(&mut self, at: u16, name_len: u8) -> u16 {
        let name = self.rd16(VAR_NAME);
        let dst = at.wrapping_add(2);
        self.ldir(name, dst, name_len as u16);
        let tb = dst.wrapping_add(name_len as u16);
        let t = self.acc_type().wrapping_sub(1);
        self.wr(tb, t);
        tb.wrapping_add(1)
    }

    /// ROM `D7A5`: put the entry at `entry` at the front of the list at `head`.
    fn link_entry(&mut self, head: u16, entry: u16, base: u16) {
        let old = self.rd16(head);
        self.wr16(head, entry.wrapping_sub(base));
        self.wr16(entry, old);
    }

    // ===============================================================================
    // Arrays   (D7B5 .. D905)
    // ===============================================================================

    /// ROM `D7B5`: one array of a `DIM` statement.  *Array already dimensioned* if it
    /// exists.
    fn dim_one(&mut self) -> Res {
        self.with_name(|s, _| {
            let c = s.cur();
            if c != b'(' && c != b'[' { return Err(Exit::Error(2)); }
            let dims = s.subscripts()?;                   // D85A
            let (head, base) = s.array_list_head(s.acc_type());
            if s.search_list(head, base).is_some() { return Err(Exit::Error(0x0A)); }
            s.create_array(true, dims)?;                  // D88A
            Ok(())
        })
    }

    /// ROM `D7DB`: after the name: an array element if a bracket follows, the cached
    /// variable if the reference had an offset, `None` if the list must be searched.
    fn array_or_cached(&mut self, name: Name) -> Res<Option<u16>> {
        let c = self.cur();
        if c == b'(' || c == b'[' { return self.array_element(name).map(Some); }
        Ok(match name {
            Name::Cached(off) => Some(self.rd16(VARS_START).wrapping_sub(1).wrapping_add(off)),
            Name::New { .. } => None,
        })
    }

    /// ROM `D7F1`: evaluate the subscripts and find the element (creating the array with
    /// 11 elements per dimension if it does not exist).  *Subscript out of range* for a
    /// wrong number of subscripts or a subscript too large.
    fn array_element(&mut self, name: Name) -> Res<u16> {
        let dims = self.subscripts()?;
        let text = self.ptr;
        let header = match name {
            Name::Cached(off) => {
                let h = self.rd16(ARRAYS_START).wrapping_sub(1).wrapping_add(off);
                if dims != self.rd(h) { return Err(Exit::Error(9)); }
                h
            }
            Name::New { token, .. } => {
                let (head, base) = self.array_list_head(self.acc_type());
                match self.search_list(head, base) {
                    Some((_, value)) => {
                        let h = value.wrapping_add(2);
                        self.cache_offset(token, h, base);
                        if dims != self.rd(h) { return Err(Exit::Error(9)); }
                        h
                    }
                    None => {
                        let h = self.create_array(false, dims)?;
                        let base = self.rd16(ARRAYS_START).wrapping_sub(1);
                        self.cache_offset(token, h, base);
                        h
                    }
                }
            }
        };
        let addr = self.element_address(header)?;          // D825
        self.ptr = text;
        Ok(addr)
    }

    /// ROM `D825`: the address of the element whose subscripts are on BASIC's stack (last
    /// subscript on top), in the array whose dimension count is at `header`.
    fn element_address(&mut self, header: u16) -> Res<u16> {
        let dims = self.rd(header);
        let mut p = header.wrapping_add(1);
        let mut index: u16 = 0;
        for _ in 0..dims {
            let size = self.rd16(p);
            let at = self.pop_stack(2);
            let sub = self.rd16(at);
            if sub >= size { return Err(Exit::Error(9)); }
            let (m, _) = self.int_mul_unsigned(index, size);
            index = m.wrapping_add(sub);
            p = p.wrapping_add(2);
        }
        let (m, _) = self.int_mul_unsigned(self.acc_type() as u16, index);
        Ok(p.wrapping_add(m))
    }

    /// ROM `D85A`: `(subscript, ...)` or `[...]` (either bracket closes either): each
    /// subscript is pushed on BASIC's stack.  Returns how many there were.  The variable's
    /// type in [`ACC_TYPE`] survives the evaluation.
    fn subscripts(&mut self) -> Res<u8> {
        self.next();
        let t = self.acc_type();
        let mut n = 0u8;
        loop {
            let v = self.eval_positive_int()?;            // CE7C
            let slot = self.stack_alloc(2)?;
            self.wr16(slot, v);
            n = n.wrapping_add(1);
            if !self.comma() { break; }
        }
        let c = self.cur();
        if c != b')' && c != b']' { return Err(Exit::Error(2)); }
        self.next();
        self.wr(ACC_TYPE, t);
        Ok(n)
    }

    /// ROM `D88A`: create an array of the type in [`ACC_TYPE`] with `dims` dimensions at
    /// the end of the array area.  `dimmed`: the sizes (subscript + 1) are on BASIC's
    /// stack, otherwise each is 11.  Returns the address of the dimension count.
    /// *Subscript out of range* if it would exceed 64 KiB.
    fn create_array(&mut self, dimmed: bool, dims: u8) -> Res<u16> {
        self.wr(VAR_SCRATCH, if dimmed { 0xFF } else { 0 });
        let fixed = dims.wrapping_add(dims).wrapping_add(3);
        let (name_len, total) = self.entry_size(fixed);
        let at = self.rd16(ARRAYS_END);
        self.open_gap(total, at, at)?;                    // F5F8 (nothing to move)
        let length_at = self.write_entry_header(at, name_len);
        let header = length_at.wrapping_add(2);
        let mut product = self.acc_type() as u16;
        self.wr(header, dims);
        let mut p = header.wrapping_add(1);
        let mut k = dims;
        loop {
            let size = if self.rd(VAR_SCRATCH) == 0 {
                0x000B
            } else {
                let s = self.pop_stack(2);
                self.rd16(s).wrapping_add(1)
            };
            self.wr16(p, size);
            p = p.wrapping_add(2);
            let (m, overflow) = self.int_mul_unsigned(product, size);
            if overflow { return Err(Exit::Error(9)); }
            product = m;
            k = k.wrapping_sub(1);
            if k == 0 { break; }                          // DJNZ
        }
        let end = self.open_gap(product, p, p)?;          // F5FB (nothing to move)
        self.wr16(ARRAYS_END, end);
        self.fill(end.wrapping_sub(product), product, 0);
        let length = (dims as u16 * 2 + 1).wrapping_add(product);
        self.wr16(length_at, length);
        let (head, base) = self.array_list_head(self.acc_type());
        self.link_entry(head, at, base);
        Ok(header)
    }

    // ===============================================================================
    // Reading a name   (D906 .. D998)
    // ===============================================================================

    /// Run `f` on the name at the text pointer ([`Self::scan_name`]).  If the name had to
    /// be copied onto BASIC's stack, the copy is dropped and [`VAR_NAME`] restored after
    /// `f` - the ROM arranges this by planting `D92B` as a return address.
    pub fn with_name<T>(&mut self, f: impl FnOnce(&mut Self, Name) -> Res<T>) -> Res<T> {
        let saved = self.rd16(VAR_NAME);
        let name = self.scan_name()?;
        let r = f(self, name)?;
        if let Name::New { .. } = name {
            // D92B
            let copy = self.rd16(VAR_NAME);
            self.set_stack(copy);
            self.wr16(VAR_NAME, saved);
        }
        Ok(r)
    }

    /// ROM `D906`: read the variable reference at the text pointer: set [`ACC_TYPE`] to
    /// its type and step over it (the pointer ends on the next non-space character).
    pub fn scan_name(&mut self) -> Res<Name> {
        self.reference_type()?;                           // D97F
        let token = self.ptr;
        let offset = self.rd16(token.wrapping_add(1));
        if offset != 0 {
            self.ptr = token.wrapping_add(2);
            loop {
                self.ptr = self.ptr.wrapping_add(1);
                if self.cur() & 0x80 != 0 { break; }
            }
            self.next();
            return Ok(Name::Cached(offset));
        }
        let letter = self.copy_name()?;                   // D939
        Ok(Name::New { token, letter })
    }

    /// ROM `D939`: the reference at the text pointer has no cached offset: settle its
    /// type (a name without suffix takes the `DEF` type of its letter, and the token is
    /// rewritten to record it), copy the name onto BASIC's stack (at most 40 characters,
    /// else *Syntax error*), point [`VAR_NAME`] at the copy and step over the name.
    /// Returns the first letter (upper case).
    fn copy_name(&mut self) -> Res<u8> {
        let token = self.ptr;
        let t = self.rd(token);
        let mut src = token.wrapping_add(3);
        let letter = self.rd(src) & !0x20;
        if t >= 0x0B {
            let ty = self.rd(DEFAULT_TYPES.wrapping_sub(1).wrapping_add((letter & 0x1F) as u16));
            self.wr(ACC_TYPE, ty);
            self.wr(token, if ty == T_REAL { 0x0D } else { ty.wrapping_add(9) });
        }
        let mut dst = self.stack_alloc(0x28)?;
        self.wr16(VAR_NAME, dst);
        let mut room = 0x29u8;
        loop {
            room -= 1;
            if room == 0 { return Err(Exit::Error(2)); }
            let c = self.rd(src) & 0xDF;
            src = src.wrapping_add(1);
            self.wr(dst, c);
            dst = dst.wrapping_add(1);
            if c & 0x80 != 0 { break; }
        }
        self.set_stack(dst);
        self.ptr = src.wrapping_sub(1);
        self.next();
        Ok(letter)
    }

    /// ROM `D97F`: the type of the reference token at the text pointer (`02`/`0B`
    /// integer, `03`/`0C` string, `04`/`0D` real), stored in [`ACC_TYPE`].  Anything
    /// else is a *Syntax error*.
    fn reference_type(&mut self) -> Res<u8> {
        let mut a = self.cur();
        if a >= 0x0B { a = a.wrapping_add(0xF7); }
        let t = match a {
            4 => T_REAL,
            2 | 3 => a,
            _ => return Err(Exit::Error(2)),
        };
        self.wr(ACC_TYPE, t);
        Ok(t)
    }

    // ===============================================================================
    // ERASE   (D999 .. D9FC)
    // ===============================================================================

    /// ROM `D999`: rebuild the three array lists by walking the array area (after `ERASE`
    /// has closed a gap).  Each array is put at the front of its list, so the lists end up
    /// in reverse memory order.
    fn relink_arrays(&mut self) {
        self.clear_array_lists();
        let end = self.rd16(ARRAYS_END);
        let mut entry = self.rd16(ARRAYS_START);
        while entry != end {
            let tb = self.skip_entry_name(entry);
            let t = (self.rd(tb) & 7) + 1;
            let length_at = tb.wrapping_add(1);
            let (head, base) = self.array_list_head(t);
            self.link_entry(head, entry, base);
            entry = length_at.wrapping_add(2).wrapping_add(self.rd16(length_at));
        }
    }

    /// `ERASE name[,name...]` (ROM `D9C0`).
    pub fn stmt_erase(&mut self) -> Res {
        self.forget_variable_offsets();                   // E989
        loop {
            self.erase_one()?;                            // D9CC
            if !self.comma() { return Ok(()); }
        }
    }

    /// ROM `D9CC`: remove one array; *Improper argument* if it does not exist.
    fn erase_one(&mut self) -> Res {
        self.with_name(|s, _| {
            let (head, base) = s.array_list_head(s.acc_type());
            let Some((entry, value)) = s.search_list(head, base) else { return Err(Exit::Error(5)) };
            let end = value.wrapping_add(2).wrapping_add(s.rd16(value));
            let count = s.rd16(ARRAYS_END).wrapping_sub(end);
            if count != 0 { s.ldir(end, entry, count); }
            s.wr16(ARRAYS_END, entry.wrapping_add(count));
            s.relink_arrays();
            Ok(())
        })
    }

    // ===============================================================================
    // DEF FN parameter frames   (D9FD .. DA73)
    // ===============================================================================

    /// ROM `D9FD`: no `DEF FN` frames.
    pub fn clear_fn_frames(&mut self) {
        self.wr16(FN_ACTIVE_FRAME, 0);
        self.wr16(FN_FRAMES, 0);
    }

    /// ROM `DA07`: push a new (not yet active) frame on BASIC's stack:
    /// `previous frame(2), parameter list head(2), previously active frame(2)`.
    pub fn push_fn_frame(&mut self) -> Res {
        let active = self.rd16(FN_ACTIVE_FRAME);
        let prev = self.rd16(FN_FRAMES);
        let f = self.stack_alloc(6)?;
        self.wr16(FN_FRAMES, f);
        self.wr16(f, prev);
        self.wr16(f + 2, 0);
        self.wr16(f + 4, active);
        Ok(())
    }

    /// ROM `DA27`: make the newest frame the one whose parameters are visible.
    pub fn activate_fn_frame(&mut self) {
        let f = self.rd16(FN_FRAMES);
        self.wr16(FN_ACTIVE_FRAME, f);
    }

    /// ROM `DA30`: drop the newest frame (and everything above it on BASIC's stack).
    pub fn pop_fn_frame(&mut self) {
        let f = self.rd16(FN_FRAMES);
        self.set_stack(f);
        let prev = self.rd16(f);
        self.wr16(FN_FRAMES, prev);
        let active = self.rd16(f + 4);
        self.wr16(FN_ACTIVE_FRAME, active);
    }

    /// ROM `DA4B`: add the parameter named at the text pointer to the newest frame: a
    /// variable entry built on BASIC's stack (`link, name, type byte, value`).  Returns
    /// the address of its value.  The name copy stays (it is part of the entry), and
    /// [`VAR_NAME`] keeps pointing at it.
    pub fn add_fn_parameter(&mut self) -> Res<u16> {
        let slot = self.stack_alloc(2)?;
        self.reference_type()?;                           // D97F
        self.copy_name()?;                                // D939
        let frame = self.rd16(FN_FRAMES);
        self.link_entry(frame.wrapping_add(2), slot, 0);
        let t = self.acc_type();
        let tb = self.stack_alloc(t.wrapping_add(1))?;
        self.wr(tb, t.wrapping_sub(1));
        Ok(tb.wrapping_add(1))
    }

    // ===============================================================================
    // Visiting every string   (DA74 .. DAF7)
    // ===============================================================================

    /// ROM `DA74`: call `visitor` for every string variable and string array element with
    /// a non-empty value: the parameters of all `DEF FN` frames, the 26 letter lists, then
    /// the string arrays.  The visitor gets the address of the descriptor's address high
    /// byte and the string's address.
    pub fn for_each_string(&mut self, visitor: StringVisitor) -> Res {
        let mut frame = self.rd16(FN_FRAMES);
        while frame != 0 {
            let next = self.rd16(frame);
            self.visit_list(frame.wrapping_add(2), 0, visitor)?;
            frame = next;
        }
        for i in 0..26u8 {
            let (head, base) = self.var_list_head(b'A' + i);
            self.visit_list(head, base, visitor)?;
        }
        // DA99: the string arrays.
        let (mut link_at, _) = self.array_list_head(T_STR);
        loop {
            let link = self.rd16(link_at);
            if link == 0 { return Ok(()); }
            let entry = self.rd16(ARRAYS_START).wrapping_sub(1).wrapping_add(link);
            let length_at = self.skip_entry_name(entry).wrapping_add(1);
            let end = length_at.wrapping_add(2).wrapping_add(self.rd16(length_at));
            let dims_at = length_at.wrapping_add(2);
            let mut d = dims_at.wrapping_add(1).wrapping_add(self.rd(dims_at) as u16 * 2);
            while d != end {
                self.visit_descriptor(d, visitor)?;       // DAE7
                d = d.wrapping_add(3);
            }
            link_at = entry;
        }
    }

    /// ROM `DACE`: the string entries of the list at `head`.
    fn visit_list(&mut self, head: u16, base: u16, visitor: StringVisitor) -> Res {
        let mut link_at = head;
        loop {
            let link = self.rd16(link_at);
            if link == 0 { return Ok(()); }
            let entry = link.wrapping_add(base);
            let tb = self.skip_entry_name(entry);
            if self.rd(tb) & 7 == T_STR - 1 { self.visit_descriptor(tb.wrapping_add(1), visitor)?; }
            link_at = entry;
        }
    }

    /// ROM `DAE7`: call the visitor for the descriptor at `d` unless the string is empty.
    fn visit_descriptor(&mut self, d: u16, visitor: StringVisitor) -> Res {
        if self.rd(d) == 0 { return Ok(()); }
        let addr = self.rd16(d.wrapping_add(1));
        let de = d.wrapping_add(2);
        match visitor {
            StringVisitor::Relocate => self.relocate_string(de, addr),     // F7BB
            StringVisitor::FindHighest => self.gc_find_highest(de, addr),  // FC9C
            StringVisitor::CopyIn => return self.copy_string_in(de, addr), // FB2E
        }
        Ok(())
    }
}
