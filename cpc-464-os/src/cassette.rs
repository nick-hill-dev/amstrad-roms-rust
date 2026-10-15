//! Cassette manager (ROM `2370`-`2A91`).
//!
//! Three layers, from top to bottom:
//!
//! 1. **Streams** (`CAS IN OPEN` ... `CAS OUT CLOSE`): a 2 KiB buffer per direction, a
//!    64-byte file header, and the "read the next block when the buffer runs dry" logic.
//! 2. **Blocks** (ROM `253F`-`2688`): find the right block on the tape, print the
//!    `Loading FRED block 1` / `Saving ...` / `Press PLAY then any key:` messages,
//!    retry on errors, handle ESC.
//! 3. **Signals** (`CAS READ` / `CAS WRITE` / `CAS CHECK`, ROM `2836`-`2A8C`): turn
//!    bytes into timed pulses on the PPI cassette output bit and back again.
//!
//! ## Tape format
//!
//! A *block* on tape is: a leader of 2049 one-bits, one zero-bit (the sync bit), a
//! marker byte (`2C` = header, `16` = data) and then the data in 256-byte *segments*,
//! each followed by the two bytes of its inverted CRC-16 (polynomial `1021`, initial
//! value `FFFF`, high byte first).  The last segment is padded with zeros to 256 bytes.
//! After the data come 33 one-bits as a trailer.  A file is a sequence of such blocks,
//! each a 64-byte header block followed by up to 2 KiB of data.
//!
//! ## Header layout (64 bytes; also the layout of the copies at `B807` and `B84C`)
//!
//! | Offset | Size | Meaning                                   |
//! |-------:|-----:|-------------------------------------------|
//! | `00`   | 16   | file name, padded with `00`               |
//! | `10`   | 1    | block number (starts at 1)                |
//! | `11`   | 1    | last-block flag (`FF` on the final block) |
//! | `12`   | 1    | file type (`16` = default / ASCII)        |
//! | `13`   | 2    | length of the data in this block          |
//! | `15`   | 2    | load address of this block                |
//! | `17`   | 1    | first-block flag (`FF` on block 1)        |
//! | `18`   | 2    | logical length of the whole file          |
//! | `1A`   | 2    | entry address                             |
//!
//! ## Timing model (read this before trusting the signal layer)
//!
//! The original produces and measures pulses by *counting Z80 instructions*: the
//! `R` (refresh) register increments on every opcode fetch and is used as a stop-watch.
//! A Rust port cannot count instructions.  The port therefore models `R` as "no time
//! has elapsed since the last edge" (`R = 0`), so every wait lasts its full nominal
//! length.  Each wait is reported to the host through [`Hardware::delay`], and each
//! pass of the edge-polling loop is one `Hardware::inp` call on port `F5xx`, which the
//! host must treat as about 57 T-states.  With a host that honours that, the write and
//! read halves are consistent; the figures are *not* cycle-exact against the real ROM.

use crate::hw::{Hardware, PPI_A, PPI_B, PPI_C, PPI_CTL};
use crate::Cpc;

// ---- variables (all addresses are the ROM's own) ----------------------------------
/// `B800`: non-zero = silent (no prompts, no `Loading` lines).  See [`Cpc::cas_noisy`].
const NOISY: u16 = 0xB800;
/// `B801`: set to `FF` when a message had to wrap onto a new line.
const MSG_WRAPPED: u16 = 0xB801;

/// `B802`: input stream state.  `0` closed, `1` open, `2` reading characters,
/// `3` reading directly, `4` aborted by ESC, `5` cataloguing.
const IN_STATE: u16 = 0xB802;
/// `B803`: address of the 2 KiB input buffer.
const IN_BUFFER: u16 = 0xB803;
/// `B805`: address of the next character in the buffer.
const IN_PTR: u16 = 0xB805;
/// `B807`: copy of the header of the file being read.
const IN_HEADER: u16 = 0xB807;

/// `B847`: output stream state (same codes as input; `4` = aborted).
const OUT_STATE: u16 = 0xB847;
/// `B848`: address of the 2 KiB output buffer (or, for direct output, the next source).
const OUT_BUFFER: u16 = 0xB848;
/// `B84A`: address where the next character will be stored.
const OUT_PTR: u16 = 0xB84A;
/// `B84C`: header of the file being written.
const OUT_HEADER: u16 = 0xB84C;

/// `B88C`: scratch header, receives each header block read from tape.
const SCRATCH_HEADER: u16 = 0xB88C;

/// `B8CC`: last tape operation (`1` read, `2` write, `0` none) so the user is prompted only
/// when it changes.
const LAST_OP: u16 = 0xB8CC;
/// `B8CD`: marker byte of the block being read / written (`2C` or `16`).
const MARKER: u16 = 0xB8CD;
/// `B8CE`: current polarity pattern for edge detection; `B8CF`: 0/1 pulse-length threshold.
const PATTERN: u16 = 0xB8CE;
const THRESHOLD: u16 = 0xB8CF;
/// `B8D0`: delay used for the next written bit.  `B8D1` / `B8D2`: speed constants set by
/// [`Cpc::cas_set_speed`] (pre-compensation, bit-0 half-period).
const WRITE_DELAY: u16 = 0xB8D0;
const SPEED_PRECOMP: u16 = 0xB8D1;
const SPEED_HALF: u16 = 0xB8D2;
/// `B8D3`: running CRC of the segment being read or written.
const CRC: u16 = 0xB8D3;

// ---- header field offsets ----------------------------------------------------------
const H_BLOCK: u16 = 0x10;
const H_LAST: u16 = 0x11;
const H_TYPE: u16 = 0x12;
const H_LENGTH: u16 = 0x13;
const H_LOAD: u16 = 0x15;
const H_FIRST: u16 = 0x17;
const H_LOGICAL: u16 = 0x18;
const H_ENTRY: u16 = 0x1A;

/// Marker byte of a header block.
const MARK_HEADER: u8 = 0x2C;
/// Marker byte of a data block.
const MARK_DATA: u8 = 0x16;
/// Size of the stream buffers.
const BLOCK_SIZE: u16 = 0x800;

/// Why a cassette call failed: the values the ROM leaves in the A register and the zero
/// flag (carry is always clear on failure).
///
/// * ESC pressed: `a == 0`, `zero == true` ([`CasError::is_escape`]).
/// * Stream in the wrong state: `a` is the old state XOR 1 (open: the state itself).
/// * End of file: `a` is the last-block flag (`FF`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CasError {
    pub a: u8,
    pub zero: bool,
}

impl CasError {
    /// The user pressed ESC (the stream is left in state 4 until it is closed).
    pub fn is_escape(&self) -> bool { self.a == 0 && self.zero }
    const ESCAPE: CasError = CasError { a: 0, zero: true };
    fn nz(a: u8) -> CasError { CasError { a, zero: false } }
}

/// Result of [`Cpc::cas_in_open`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CasOpen {
    /// Address of the buffer holding the file header (`B807`).
    pub header: u16,
    /// Load address recorded in the header (`DE`).
    pub load_address: u16,
    /// Logical length of the file (`BC`).
    pub logical_length: u16,
    /// File type (`A`).
    pub file_type: u8,
}

/// The message table at ROM `27C5`.  Each message is a NUL-terminated list of *words*;
/// the last letter of a word has bit 7 set, and a byte with bit 7 set at the start of a
/// word is a reference to another message.  Decoded:
///
/// | # | text                                | # | text            |
/// |--:|-------------------------------------|--:|-----------------|
/// | 0 | `Press`                             | 8 | `Found  `       |
/// | 1 | `PLAY then any key:`                | 9 | `Loading`       |
/// | 2 | `error`                             | 10| `Saving`        |
/// | 3 | `Press PLAY then any key:`          | 11| (empty)         |
/// | 4 | `Press REC and PLAY then any key:`  | 12| `Ok`            |
/// | 5 | `Read error`                        | 13| `block`         |
/// | 6 | `Write error`                       | 14| `Unnamed file   `|
/// | 7 | `Rewind tape`                       |   |                 |
const MESSAGES: [u8; 113] = [
    0x50, 0x72, 0x65, 0x73, 0xF3, 0x00,
    0x50, 0x4C, 0x41, 0xD9, 0x74, 0x68, 0x65, 0xEE, 0x61, 0x6E, 0xF9, 0x6B, 0x65, 0x79, 0xBA, 0x00,
    0x65, 0x72, 0x72, 0x6F, 0xF2, 0x00,
    0x80, 0x81, 0x00,
    0x80, 0x52, 0x45, 0xC3, 0x61, 0x6E, 0xE4, 0x81, 0x00,
    0x52, 0x65, 0x61, 0xE4, 0x82, 0x00,
    0x57, 0x72, 0x69, 0x74, 0xE5, 0x82, 0x00,
    0x52, 0x65, 0x77, 0x69, 0x6E, 0xE4, 0x74, 0x61, 0x70, 0xE5, 0x00,
    0x46, 0x6F, 0x75, 0x6E, 0x64, 0x20, 0xA0, 0x00,
    0x4C, 0x6F, 0x61, 0x64, 0x69, 0x6E, 0xE7, 0x00,
    0x53, 0x61, 0x76, 0x69, 0x6E, 0xE7, 0x00,
    0x00,
    0x4F, 0xEB, 0x00,
    0x62, 0x6C, 0x6F, 0x63, 0xEB, 0x00,
    0x55, 0x6E, 0x6E, 0x61, 0x6D, 0x65, 0xE4, 0x66, 0x69, 0x6C, 0x65, 0x20, 0x20, 0x20, 0xA0, 0x00,
];

/// Which of the three byte-level routines the data loop of `CAS READ` / `CAS CHECK` /
/// `CAS WRITE` runs (ROM `28B8`, `28C7`, `28F7`).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Transfer {
    /// Read from tape into memory.
    Read,
    /// Compare tape with memory.
    Check,
    /// Write memory to tape.
    Write,
}

impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Set-up                                                         (2370 .. 2391)
    // ===============================================================================

    /// `CAS INITIALISE` (`BC65`, ROM `2370`): close both streams, enable messages and select
    /// 1000 baud.
    pub fn cas_initialise(&mut self) {
        self.cas_in_abandon();
        self.cas_out_abandon();
        self.cas_noisy(0);
        self.cas_set_speed(0x014D, 0x19);
    }

    /// `CAS SET SPEED` (`BC68`, ROM `237F`).
    ///
    /// `half_zero` is the length in µs of half a zero bit (333 = 1000 baud, 167 = 2000
    /// baud) and `precomp` the pre-compensation in µs (25 for either speed).  Both are
    /// converted to the units of the pulse delay loops (4 µs) and stored.
    pub fn cas_set_speed(&mut self, half_zero: u16, precomp: u8) {
        let scaled = half_zero.wrapping_shl(6); // six `ADD HL,HL`
        let precomp = precomp.rotate_right(2) & 0x3F; // two `RRCA`, `AND 3F`
        self.wr16(SPEED_PRECOMP, (scaled & 0xFF00) | precomp as u16);
    }

    /// `CAS NOISY` (`BC6B`, ROM `238E`): `0` = print prompts and file names, non-zero =
    /// stay silent.
    pub fn cas_noisy(&mut self, flag: u8) { self.wr8(NOISY, flag); }

    // ===============================================================================
    // Opening and closing streams                                    (2392 .. 2434)
    // ===============================================================================

    /// `CAS IN OPEN` (`BC77`, ROM `2392`): open a file for reading and read its first block.
    ///
    /// `name` / `name_len` locate the file name in memory (an empty name accepts the
    /// first file on the tape); `buffer` is a 2 KiB buffer.  Fails if the stream is already
    /// open or the user presses ESC at the prompt.
    pub fn cas_in_open(&mut self, name: u16, name_len: u8, buffer: u16) -> Result<CasOpen, CasError> {
        let header = self.cas_open_stream(IN_STATE, name, name_len, buffer)?;
        self.cas_next_block()?;
        Ok(CasOpen {
            header,
            load_address: self.rd16(IN_HEADER + H_LOAD),
            logical_length: self.rd16(IN_HEADER + H_LOGICAL),
            file_type: self.rd8(IN_HEADER + H_TYPE),
        })
    }

    /// `CAS OUT OPEN` (`BC8C`, ROM `23AB`): open a file for writing.  Returns the address of
    /// the header buffer (`B84C`), so the caller may set the type, length and entry address
    /// before writing.  Nothing is written to tape yet.
    pub fn cas_out_open(&mut self, name: u16, name_len: u8, buffer: u16) -> Result<u16, CasError> {
        self.cas_open_stream(OUT_STATE, name, name_len, buffer)
    }

    /// ROM `23AF`: shared part of the two OPEN calls.  `desc` is `B802` or `B847`.
    fn cas_open_stream(&mut self, desc: u16, name: u16, len: u8, buffer: u16) -> Result<u16, CasError> {
        let state = self.rd8(desc);
        if state != 0 {
            return Err(CasError::nz(state));
        }
        self.wr8(desc, 1); // state: open
        self.wr16(desc + 1, buffer); // buffer address
        self.wr16(desc + 3, buffer); // current pointer
        let header = desc + 5;
        for i in 0..0x40 {
            self.wr8(header + i, 0);
        }
        // Copy the name (at most 16 characters), converting a-z to upper case.
        let n = len.min(16) as u16;
        for i in 0..n {
            let c = Self::cas_upper(self.rd8(name.wrapping_add(i)));
            self.wr8(header + i, c);
        }
        // Trailing spaces become NULs.
        for i in (0..n).rev() {
            if self.rd8(header + i) != 0x20 { break; }
            self.wr8(header + i, 0);
        }
        self.wr8(header + H_BLOCK, 1); // first block is number 1
        self.wr8(header + H_TYPE, 0x16); // default file type
        self.wr8(header + H_FIRST, 0xFF); // "this is the first block" (IX+28 decremented from 0)
        Ok(header)
    }

    /// `CAS IN CLOSE` (`BC7A`, ROM `23FC`): close the input stream (does nothing if it is
    /// closed).  Returns the address of the buffer it was using.
    pub fn cas_in_close(&mut self) -> u16 {
        if self.rd8(IN_STATE) == 0 { return self.rd16(IN_BUFFER); }
        self.cas_in_abandon()
    }

    /// `CAS IN ABANDON` (`BC7D`, ROM `2401`): forget the input stream at once.  Returns the
    /// address of its buffer.
    pub fn cas_in_abandon(&mut self) -> u16 { self.cas_abandon(IN_STATE, 1) }

    /// `CAS OUT CLOSE` (`BC8F`, ROM `2415`): write out the partly filled last block (with the
    /// last-block flag set) and close the stream.  If the stream was aborted by ESC it is
    /// abandoned without writing.
    pub fn cas_out_close(&mut self) -> Result<u16, CasError> {
        let state = self.rd8(OUT_STATE);
        if state == 4 {
            return Ok(self.cas_out_abandon());
        }
        if state == 0 {
            return Err(CasError::nz(0xFF)); // `ADD A,FF` leaves A = FF, carry clear
        }
        self.wr8(OUT_HEADER + H_LAST, 0xFF);
        if self.rd16(OUT_HEADER + H_LENGTH) != 0 {
            self.cas_write_block()?;
        }
        Ok(self.cas_out_abandon())
    }

    /// `CAS OUT ABANDON` (`BC92`, ROM `242E`): forget the output stream at once.  Returns
    /// the address of its buffer.
    pub fn cas_out_abandon(&mut self) -> u16 { self.cas_abandon(OUT_STATE, 2) }

    /// ROM `2406`: mark a stream closed and, if the tape was last used in this direction,
    /// forget that (so the next tape operation prompts again).
    fn cas_abandon(&mut self, desc: u16, op: u8) -> u16 {
        self.wr8(desc, 0);
        if self.rd8(LAST_OP) == op {
            self.wr8(LAST_OP, 0);
        }
        self.rd16(desc + 1)
    }

    // ===============================================================================
    // Character I/O                                                  (2435 .. 249A)
    // ===============================================================================

    /// `CAS IN CHAR` (`BC80`, ROM `2435`): the next character of the file.
    ///
    /// When the 2 KiB buffer is empty the next block is read from tape first.  At the end of
    /// the file the error has `a == 0xFF`; if the user pressed ESC it is
    /// [`CasError::is_escape`].
    pub fn cas_in_char(&mut self) -> Result<u8, CasError> {
        self.cas_claim(IN_STATE, 2)?;
        if self.rd16(IN_HEADER + H_LENGTH) == 0 {
            self.cas_next_block()?;
        }
        let remaining = self.rd16(IN_HEADER + H_LENGTH).wrapping_sub(1);
        self.wr16(IN_HEADER + H_LENGTH, remaining);
        let ptr = self.rd16(IN_PTR);
        let ch = self.rd8(ptr); // RST 20h: RAM LAM
        self.wr16(IN_PTR, ptr.wrapping_add(1));
        Ok(ch)
    }

    /// `CAS OUT CHAR` (`BC95`, ROM `245B`): append a character to the file; a full 2 KiB
    /// buffer is written to tape first.
    pub fn cas_out_char(&mut self, ch: u8) -> Result<(), CasError> {
        self.cas_claim(OUT_STATE, 2)?;
        if self.rd16(OUT_HEADER + H_LENGTH) >= BLOCK_SIZE {
            self.cas_write_block()?;
        }
        let len = self.rd16(OUT_HEADER + H_LENGTH);
        self.wr16(OUT_HEADER + H_LENGTH, len.wrapping_add(1));
        let ptr = self.rd16(OUT_PTR);
        self.wr8(ptr, ch);
        self.wr16(OUT_PTR, ptr.wrapping_add(1));
        Ok(())
    }

    /// ROM `248B` / `248E`: claim a stream for use in mode `mode` (2 = characters,
    /// 3 = direct).  A freshly opened stream (state 1) switches to `mode`; one already in
    /// that mode is fine; anything else is an error (`a` = old state XOR 1).
    fn cas_claim(&mut self, desc: u16, mode: u8) -> Result<(), CasError> {
        let state = self.rd8(desc);
        if state == mode { return Ok(()); }
        if state != 1 { return Err(CasError::nz(state ^ 1)); }
        self.wr8(desc, mode);
        Ok(())
    }

    /// `CAS TEST EOF` (`BC89`, ROM `2496`): `Ok(ch)` if there is another character (it is
    /// pushed back so the next [`Self::cas_in_char`] returns it again), `Err` at the end.
    pub fn cas_test_eof(&mut self) -> Result<u8, CasError> {
        let ch = self.cas_in_char()?;
        self.cas_return();
        Ok(ch)
    }

    /// `CAS RETURN` (`BC86`, ROM `249A`): put the character just read back.
    pub fn cas_return(&mut self) {
        let n = self.rd16(IN_HEADER + H_LENGTH).wrapping_add(1);
        self.wr16(IN_HEADER + H_LENGTH, n);
        let p = self.rd16(IN_PTR).wrapping_sub(1);
        self.wr16(IN_PTR, p);
    }

    // ===============================================================================
    // Direct (whole-file) I/O                                        (24AB .. 2527)
    // ===============================================================================

    /// `CAS IN DIRECT` (`BC83`, ROM `24AB`): read the rest of the file straight into memory
    /// at `dest`.  Returns the entry address from the header of the last block.
    ///
    /// The first block was already read into the 2 KiB buffer by `CAS IN OPEN`; it is
    /// moved to `dest` first, so `dest` may overlap that buffer.
    pub fn cas_in_direct(&mut self, dest: u16) -> Result<u16, CasError> {
        self.cas_claim(IN_STATE, 3)?;
        self.wr16(IN_HEADER + H_LOAD, dest);
        // ROM 24CF: move the data already in the buffer to its destination.  Copy forwards
        // if the destination is below the buffer, otherwise backwards from the last byte.
        let buffer = self.rd16(IN_BUFFER);
        let len = self.rd16(IN_HEADER + H_LENGTH);
        if dest < buffer {
            self.kl_ldir(buffer, dest, len);
        } else {
            self.kl_lddr(buffer.wrapping_add(len).wrapping_sub(1), dest.wrapping_add(len).wrapping_sub(1), len);
        }
        loop {
            // ROM 24B9: the next block goes right after the previous one.
            let next = self.rd16(IN_HEADER + H_LOAD).wrapping_add(self.rd16(IN_HEADER + H_LENGTH));
            self.wr16(IN_HEADER + H_LOAD, next);
            match self.cas_next_block() {
                Ok(()) => continue,
                // `L_253F` returned NZ: the last block had already been read -> success.
                Err(e) if !e.zero => return Ok(self.rd16(SCRATCH_HEADER + H_ENTRY)),
                Err(e) => return Err(e), // ESC
            }
        }
    }

    /// `CAS OUT DIRECT` (`BC98`, ROM `24EA`): write `length` bytes from `start` as a whole
    /// file of type `file_type` with the given entry address.
    ///
    /// Full 2 KiB blocks are written at once; the final partial block is written by
    /// [`Self::cas_out_close`].
    pub fn cas_out_direct(&mut self, start: u16, length: u16, entry: u16, file_type: u8) -> Result<(), CasError> {
        self.cas_claim(OUT_STATE, 3)?;
        self.wr8(OUT_HEADER + H_TYPE, file_type);
        self.wr16(OUT_HEADER + H_LOGICAL, length);
        self.wr16(OUT_HEADER + H_ENTRY, entry);
        let mut src = start;
        let mut left = length;
        loop {
            self.wr16(OUT_BUFFER, src);
            self.wr16(OUT_HEADER + H_LENGTH, left);
            if left <= BLOCK_SIZE { return Ok(()); } // the rest is flushed by CAS OUT CLOSE
            self.wr16(OUT_HEADER + H_LENGTH, BLOCK_SIZE);
            left -= BLOCK_SIZE;
            let next = src.wrapping_add(BLOCK_SIZE);
            self.cas_write_block()?;
            src = next;
        }
    }

    /// `CAS CATALOG` (`BC9B`, ROM `2528`): list the files on the tape (until ESC), using
    /// `buffer` (2 KiB) as scratch space.  The listing is printed on the text screen.
    pub fn cas_catalog(&mut self, buffer: u16) -> Result<u16, CasError> {
        let state = self.rd8(IN_STATE);
        if state != 0 { return Err(CasError::nz(state)); }
        self.wr8(IN_STATE, 5);
        self.wr16(IN_BUFFER, buffer);
        self.cas_noisy(0);
        while self.cas_read_block().is_ok() {}
        Ok(self.cas_in_abandon())
    }

    // ===============================================================================
    // Block level                                                    (253F .. 2760)
    // ===============================================================================

    /// ROM `253F`: read the next block of the input file, unless the last one was already
    /// read (error with `a` = the last-block flag).
    fn cas_next_block(&mut self) -> Result<(), CasError> {
        let last = self.rd8(IN_HEADER + H_LAST);
        if last != 0 { return Err(CasError::nz(last)); }
        self.cas_read_block()
    }

    /// ROM `2544`: prompt, then find and read one block (header + data) that belongs to
    /// the file being read.  Read errors print `Read error x` and try again; only ESC
    /// (or a stream that was aborted) fails.
    fn cas_read_block(&mut self) -> Result<(), CasError> {
        let mut code: u8 = if self.cas_motor_prompt(0x83, 1) { 0xFF } else { 0 };
        loop {
            if code == 0xFF {
                // L_254C: read a header block into the scratch header and see whether
                // it is the one we want.
                match self.cas_read(SCRATCH_HEADER, 0x40, MARK_HEADER) {
                    Err(a) => code = a,
                    Ok(()) => {
                        let (zero, carry) = self.cas_match_header();
                        if !zero {
                            // Not the wanted block: say what was found and go on.
                            self.cas_file_line(SCRATCH_HEADER, 0x88);
                            if carry {
                                self.cas_error_line(0x87, 0xFF); // "Rewind tape"
                            }
                            continue;
                        }
                        self.cas_file_line(SCRATCH_HEADER, if carry { 0x8B } else { 0x89 });
                        match self.cas_read_data_block() {
                            Ok(()) => return self.cas_finish_block(),
                            Err(a) => code = a,
                        }
                    }
                }
            }
            // L_25A8: an error.  A = 0 is ESC; anything else is reported and retried.
            if code == 0 {
                self.wr8(IN_STATE, 4);
                self.cas_stop_motor_keep();
                return Err(CasError::ESCAPE);
            }
            self.cas_error_line(0x85, code);
            code = 0xFF;
        }
    }

    /// ROM `2564`-`259D`: read the data block that follows an accepted header.
    fn cas_read_data_block(&mut self) -> Result<(), u8> {
        let len = self.rd16(SCRATCH_HEADER + H_LENGTH);
        let mut dest = self.rd16(IN_HEADER + H_LOAD);
        if self.rd8(IN_STATE) != 3 {
            // Buffered reading: the data must fit the 2 KiB buffer.
            if len > BLOCK_SIZE { return Err(4); }
            dest = self.rd16(IN_BUFFER);
            self.wr16(IN_PTR, dest);
        }
        self.cas_read(dest, len, MARK_DATA)?;
        // Account for the block in the header copy.
        let n = self.rd8(IN_HEADER + H_BLOCK).wrapping_add(1);
        self.wr8(IN_HEADER + H_BLOCK, n);
        self.wr8(IN_HEADER + H_LAST, self.rd8(SCRATCH_HEADER + H_LAST));
        self.wr8(IN_HEADER + H_FIRST, 0);
        self.wr16(IN_HEADER + H_LENGTH, len);
        Ok(())
    }

    /// ROM `25A0`-`260D`: after a good block, print `Ok` while cataloguing and stop the
    /// motor.
    fn cas_finish_block(&mut self) -> Result<(), CasError> {
        if self.rd8(IN_STATE) == 5 {
            self.cas_message_line(0x8C);
        }
        self.cas_stop_motor_keep();
        Ok(())
    }

    /// ROM `25C5`: does the header just read in `B88C` belong to the wanted file?
    /// Returns the `(zero, carry)` flags the ROM returns:
    ///
    /// * `(true, true)`: cataloguing – every header is of interest.
    /// * `(true, false)`: accept this block.
    /// * `(false, false)`: skip it (wrong name, or not the first block when looking for one).
    /// * `(false, true)`: right file, but a *later* block than expected (tape must be rewound).
    fn cas_match_header(&mut self) -> (bool, bool) {
        if self.rd8(IN_STATE) == 5 { return (true, true); }
        if self.rd8(IN_HEADER + H_FIRST) == 0 {
            // Already inside the file: name and block number must match.
            if !self.cas_names_equal() { return (false, false); }
            let expected = self.rd8(IN_HEADER + H_BLOCK);
            let actual = self.rd8(SCRATCH_HEADER + H_BLOCK);
            return (expected == actual, expected < actual);
        }
        // Looking for the first block of the file.
        if self.rd8(SCRATCH_HEADER + H_FIRST) != 0xFF { return (false, false); }
        if self.rd8(IN_HEADER) != 0 && !self.cas_names_equal() { return (false, false); }
        for i in 0..0x40 {
            let b = self.rd8(SCRATCH_HEADER + i);
            self.wr8(IN_HEADER + i, b);
        }
        (true, false)
    }

    /// ROM `25F3`: compare the 16-character names in `B807` and `B88C`, ignoring case.
    fn cas_names_equal(&self) -> bool {
        (0..16).all(|i| Self::cas_upper(self.rd8(IN_HEADER + i)) == Self::cas_upper(self.rd8(SCRATCH_HEADER + i)))
    }

    /// ROM `27B6`: lower case → upper case, everything else unchanged.
    fn cas_upper(c: u8) -> u8 {
        if (0x61..0x7B).contains(&c) { c - 0x20 } else { c }
    }

    /// ROM `2614`: write the buffered block (header, then data) to tape.  Write errors
    /// print `Write error x` and the block is written again; ESC fails.
    fn cas_write_block(&mut self) -> Result<(), CasError> {
        let mut code: u8 = 0xFF; // 0xFF = "no error, go ahead"
        if !self.cas_motor_prompt(0x84, 2) {
            code = 0;
        } else {
            self.cas_file_line(OUT_HEADER, 0x8A);
            if !self.cas_pause_if_set(OUT_HEADER + H_FIRST) { code = 0; }
        }
        loop {
            if code == 0xFF {
                // L_262C: the data comes from the start of the buffer; record where that is.
                let src = self.rd16(OUT_BUFFER);
                self.wr16(OUT_PTR, src);
                self.wr16(OUT_HEADER + H_LOAD, src);
                let result = self.cas_write(OUT_HEADER, 0x40, MARK_HEADER).and_then(|()| {
                    let len = self.rd16(OUT_HEADER + H_LENGTH);
                    self.cas_write(src, len, MARK_DATA)
                });
                match result {
                    Err(a) => code = a,
                    Ok(()) => {
                        if self.cas_pause_if_set(OUT_HEADER + H_LAST) {
                            // Block done: reset the buffer, bump the block number.
                            self.wr16(OUT_HEADER + H_LENGTH, 0);
                            let n = self.rd8(OUT_HEADER + H_BLOCK).wrapping_add(1);
                            self.wr8(OUT_HEADER + H_BLOCK, n);
                            self.wr8(OUT_HEADER + H_FIRST, 0);
                            self.cas_stop_motor_keep();
                            return Ok(());
                        }
                        code = 0;
                    }
                }
            }
            // L_2666: error handling.
            if code == 0 {
                self.wr8(OUT_STATE, 4);
                self.cas_stop_motor_keep();
                return Err(CasError::ESCAPE);
            }
            self.cas_error_line(0x86, code);
            code = 0xFF;
        }
    }

    /// ROM `2688`: if the flag byte at `flag` is non-zero, pause for about three seconds
    /// (the gap between the first block's header and data / after the last block).
    /// Returns `false` if ESC was pressed.
    fn cas_pause_if_set(&mut self, flag: u16) -> bool {
        if self.rd8(flag) == 0 { return true; }
        self.cas_delay_esc(0x012C)
    }

    /// ROM `2673`: before a tape operation of kind `op` (1 read, 2 write), start the motor.
    /// If the operation differs from the last one, first prompt with message `msg`
    /// (`Press PLAY ...` / `Press REC and PLAY ...`).  Returns `false` on ESC.
    fn cas_motor_prompt(&mut self, msg: u8, op: u8) -> bool {
        let same = self.rd8(LAST_OP) == op;
        self.wr8(LAST_OP, 0);
        let go = if same { true } else { self.cas_prompt(msg) };
        if !go { return false; }
        self.wr8(LAST_OP, op);
        self.cas_start_motor().0
    }

    // ===============================================================================
    // Messages                                                       (2692 .. 27C4)
    // ===============================================================================

    /// ROM `2760`: show `msg`, wait for a key.  Returns `false` if the key was `1B`.
    /// Does nothing (returns `true`) while messages are disabled by `CAS NOISY`.
    fn cas_prompt(&mut self, msg: u8) -> bool {
        if self.rd8(NOISY) != 0 { return true; }
        self.cas_fresh_line();
        self.cas_print_message(msg);
        while self.km_read_char().is_some() {} // discard type-ahead
        self.txt_cur_on();
        let key = self.km_wait_key();
        self.txt_cur_off();
        if key == 0x1B { return false; }
        self.cas_newline();
        true
    }

    /// ROM `2692`: print the message, then the header's file name, block number and
    /// (while cataloguing) file type: `Loading FRED block 1`.  `header` is `B88C` for
    /// blocks read and `B84C` for blocks written.
    fn cas_file_line(&mut self, header: u16, msg: u8) {
        if self.rd8(NOISY) != 0 { return; }
        self.wr8(MSG_WRAPPED, 0);
        self.cas_column_one();
        self.cas_print_message(msg);
        let cataloguing = self.rd8(IN_STATE) == 5;
        if self.rd8(header) == 0 {
            self.cas_print_message(0x8E); // "Unnamed file"
        } else {
            let mut count = 16u8;
            if !cataloguing {
                // Only as many characters as the name has.
                count = (0..16).take_while(|&i| self.rd8(header + i) != 0).count() as u8;
            }
            self.cas_fit(count);
            for i in 0..count as u16 {
                let c = Self::cas_upper(self.rd8(header + i));
                self.txt_wr_char(if c == 0 { b' ' } else { c });
            }
            self.cas_space();
        }
        self.cas_print_message(0x8D); // "block"
        self.cas_fit(2);
        let block = self.rd8(header + H_BLOCK);
        self.cas_print_decimal(block);
        self.cas_space();
        if cataloguing {
            let file_type = self.rd8(header + H_TYPE);
            self.txt_output((file_type & 0x0F) + 0x24); // `$ % & ' ...`
            self.cas_space();
        } else if self.rd8(header + H_LAST) | self.rd8(MSG_WRAPPED) != 0 {
            self.cas_newline();
        }
    }

    /// ROM `270C`: a message on a line of its own.
    fn cas_message_line(&mut self, msg: u8) {
        self.cas_print_message(msg);
        self.cas_newline();
    }

    /// ROM `2713`: `Read error x` / `Write error x` followed by a newline.  `code` is the
    /// error number 1..: it is shown as a letter (`a`, `b` ...); `0xFF` = no letter.
    fn cas_error_line(&mut self, msg: u8, code: u8) {
        self.cas_fresh_line();
        self.cas_print_message(msg);
        let (c, overflow) = code.overflowing_add(0x60);
        if !overflow { self.txt_output(c); }
        self.cas_newline();
    }

    /// ROM `271F`: start a new line unless the cursor is already in column 1.
    fn cas_fresh_line(&mut self) {
        let (col, _, _) = self.txt_get_cursor();
        if col != 1 { self.cas_newline(); }
    }

    /// ROM `2727`: print message `n & 7F` (see [`MESSAGES`]).
    fn cas_print_message(&mut self, n: u8) {
        // Find the n-th NUL-terminated string.
        let mut p = 0usize;
        for _ in 0..(n & 0x7F) {
            while MESSAGES[p] != 0 { p += 1; }
            p += 1;
        }
        loop {
            let b = MESSAGES[p];
            if b == 0 { return; }
            if b & 0x80 != 0 {
                self.cas_print_message(b); // reference to another message
                p += 1;
                continue;
            }
            // A word: characters up to and including the one with bit 7 set.
            let mut len = 0usize;
            while MESSAGES[p + len] & 0x80 == 0 { len += 1; }
            len += 1;
            self.cas_fit(len as u8);
            for i in 0..len {
                self.txt_output(MESSAGES[p + i] & 0x7F);
            }
            p += len;
            self.cas_space();
        }
    }

    /// ROM `278D`: make sure `n` more characters fit on the line; otherwise start a new one
    /// (and remember that it wrapped, so the file line ends with a newline).
    fn cas_fit(&mut self, n: u8) {
        let ((left, _), (right, _), _) = self.txt_get_window();
        let (col, _, _) = self.txt_get_cursor();
        let last = col.wrapping_sub(1).wrapping_add(left).wrapping_add(n).wrapping_sub(1);
        if last >= right {
            self.wr8(MSG_WRAPPED, 0xFF);
            self.cas_newline();
        }
    }

    /// ROM `27A4`: print `v` in decimal (no leading zeros).
    fn cas_print_decimal(&mut self, v: u8) {
        if v >= 10 { self.cas_print_decimal(v / 10); }
        self.txt_output(b'0' + v % 10);
    }

    /// ROM `275C`: one space.
    fn cas_space(&mut self) { self.txt_output(b' '); }
    /// ROM `2783`: cursor to column 1.
    fn cas_column_one(&mut self) { self.txt_set_column(1); }
    /// ROM `277B`: carriage return, line feed.
    fn cas_newline(&mut self) {
        self.cas_column_one();
        self.txt_output(0x0A);
    }

    // ===============================================================================
    // Motor                                                          (2A4B .. 2A91)
    // ===============================================================================

    /// `CAS START MOTOR` (`BC6E`, ROM `2A4B`).
    pub fn cas_start_motor(&mut self) -> (bool, u8) { self.cas_restore_motor(0x10) }

    /// `CAS STOP MOTOR` (`BC71`, ROM `2A4F`).
    pub fn cas_stop_motor(&mut self) -> (bool, u8) { self.cas_restore_motor(0xEF) }

    /// ROM `260F`: stop the motor (the callers' status is unaffected).
    fn cas_stop_motor_keep(&mut self) { self.cas_stop_motor(); }

    /// `CAS RESTORE MOTOR` (`BC74`, ROM `2A51`): bit 4 of `state` selects motor on / off.
    ///
    /// Returns `(ok, previous port C value)`.  When the motor has to be started from rest
    /// the routine waits about two seconds for it to run up; `ok` is `false` if ESC was
    /// pressed during that wait.
    pub fn cas_restore_motor(&mut self, state: u8) -> (bool, u8) {
        let old = self.hw.inp(PPI_C);
        let on = state & 0x10 != 0;
        self.hw.out(PPI_CTL, if on { 9 } else { 8 }); // PPI bit set / reset: bit 4 = motor
        if !on { return (true, old); }
        let ok = if old & 0x10 == 0 { self.cas_delay_esc(0x00C8) } else { true };
        (ok, old)
    }

    /// ROM `2A72`: wait `n` × 10.8 ms, stopping early (returning `false`) if ESC is pressed.
    fn cas_delay_esc(&mut self, mut n: u16) -> bool {
        loop {
            self.cas_short_delay();
            self.interrupt(); // interrupts are enabled here, so the keyboard keeps scanning
            if self.km_test_key(0x42).0 { return false; }
            n = n.wrapping_sub(1);
            if n == 0 { return true; }
        }
    }

    /// ROM `2A89`: a 1666-pass busy loop (about 10.8 ms).
    fn cas_short_delay(&mut self) { self.hw.delay(1666 * 26); }

    // ===============================================================================
    // Signal level: CAS READ / WRITE / CHECK                         (2836 .. 2A41)
    // ===============================================================================

    /// `CAS READ` (`BCA1`, ROM `2836`): read one block of `len` bytes into memory at `dest`.
    /// `marker` selects header (`2C`) or data (`16`) blocks.
    ///
    /// Waits for a leader and sync byte (indefinitely: only ESC stops it), then reads the
    /// segments checking each CRC.  Errors: `0` ESC, `1` signal lost, `2` CRC error.
    pub fn cas_read(&mut self, dest: u16, len: u16, marker: u8) -> Result<(), u8> {
        self.cas_block(Transfer::Read, dest, len, marker)
    }

    /// `CAS CHECK` (`BCA7`, ROM `2851`): like [`Self::cas_read`] but compare the tape with
    /// memory instead of storing it.  Error `3` = mismatch.
    pub fn cas_check(&mut self, addr: u16, len: u16, marker: u8) -> Result<(), u8> {
        self.cas_block(Transfer::Check, addr, len, marker)
    }

    /// `CAS WRITE` (`BCA4`, ROM `283F`): write `len` bytes from `src` as one block.
    /// Errors: `0` ESC, `1` pulse timing failed.
    pub fn cas_write(&mut self, src: u16, len: u16, marker: u8) -> Result<(), u8> {
        self.cas_block(Transfer::Write, src, len, marker)
    }

    /// Common frame of the three calls (ROM `2873`, `2858`/`283F`, `2860`).
    fn cas_block(&mut self, mode: Transfer, addr: u16, len: u16, marker: u8) -> Result<(), u8> {
        // ROM 2873: set up.
        self.wr8(MARKER, marker);
        let l = len.wrapping_sub(1);
        let chunks = (l >> 8) as u8; // D: number of complete 256-byte segments before the last
        let last = (l as u8).wrapping_add(1); // E: bytes in the last segment (0 means 256)
        self.sound_reset(); // the PSG registers are about to be shared with the keyboard
        let (_, saved_motor) = self.cas_start_motor();
        // Interrupts off; PPI: port A input (PSG register 14 = keyboard), motor on, row 8.
        self.hw.out(PPI_A | 0x0E, 0x0E);
        self.hw.out(PPI_C | 0xD0, 0xD0);
        self.hw.out(PPI_C | 0x10, 0x10);
        self.hw.out(PPI_CTL | 0x92, 0x92);
        self.hw.out(PPI_C | 0x58, 0x58);

        let result = match mode {
            Transfer::Write => self.cas_write_leader().and_then(|()| {
                self.cas_segments(mode, addr, chunks, last)?;
                self.cas_write_trailer()
            }),
            _ => self.cas_find_block().and_then(|()| self.cas_segments(mode, addr, chunks, last)),
        };

        // ROM 2860: restore the PPI and the motor.
        self.hw.out(PPI_CTL | 0x82, 0x82);
        self.hw.out(PPI_C | 0x10, 0x10);
        self.cas_restore_motor(saved_motor);
        result
    }

    /// ROM `289D`: `chunks` full segments of 256 bytes, then one of `last` bytes.
    fn cas_segments(&mut self, mode: Transfer, addr: u16, chunks: u8, last: u8) -> Result<(), u8> {
        let mut ptr = addr;
        for _ in 0..chunks {
            self.cas_segment(mode, &mut ptr, 0)?;
        }
        self.cas_segment(mode, &mut ptr, last)
    }

    /// ROM `28AE`: one segment: `count` bytes (0 = 256), zero padding up to 256 bytes,
    /// then the inverted CRC.
    fn cas_segment(&mut self, mode: Transfer, ptr: &mut u16, count: u8) -> Result<(), u8> {
        self.wr16(CRC, 0xFFFF);
        let n: u16 = if count == 0 { 256 } else { count as u16 };
        for _ in 0..n {
            match mode {
                Transfer::Read => {
                    let b = self.cas_read_byte()?;
                    self.wr8(*ptr, b);
                }
                Transfer::Check => {
                    let b = self.cas_read_byte()?;
                    if self.rd8(*ptr) != b { return Err(3); }
                }
                Transfer::Write => {
                    let b = self.rd8(*ptr);
                    self.cas_write_byte(b)?;
                }
            }
            *ptr = ptr.wrapping_add(1);
        }
        // Padding: the tape always carries 256 bytes per segment.
        for _ in n..256 {
            match mode {
                Transfer::Write => self.cas_write_byte(0)?,
                _ => { self.cas_read_byte()?; }
            }
        }
        // CRC: the complement of the running CRC, high byte first.  The value is taken
        // *before* the CRC bytes themselves are processed.
        let crc = !self.rd16(CRC);
        let (hi, lo) = ((crc >> 8) as u8, crc as u8);
        match mode {
            Transfer::Write => {
                self.cas_write_byte(hi)?;
                self.cas_write_byte(lo)
            }
            _ => {
                if self.cas_read_byte()? != hi { return Err(2); }
                if self.cas_read_byte()? != lo { return Err(2); }
                Ok(())
            }
        }
    }

    // ---- reading ---------------------------------------------------------------------

    /// ROM `2919`: wait for a leader and the sync byte; retry on garbage; ESC → `Err(0)`.
    fn cas_find_block(&mut self) -> Result<(), u8> {
        loop {
            match self.cas_find_leader() {
                Ok(()) => return Ok(()),
                Err(0) => return Err(0),
                Err(_) => continue,
            }
        }
    }

    /// ROM `2923`: measure the leader, derive the 0/1 threshold from it, wait for the
    /// zero sync bit and check the marker byte.
    ///
    /// The leader is a train of long (one-bit) pulses.  256 half-pulses are summed to get
    /// their mean `D`.  A bit is then recognised as the sync when two successive
    /// half-pulses together are at most 1¼ × `D` (a zero bit is half as long as a one).
    /// The threshold between zero and one bits is 1½ × `D`.
    fn cas_find_leader(&mut self) -> Result<(), u8> {
        let mut l: u8 = 0x55;
        self.cas_pulse(&mut l)?;
        let mut sum: u16 = 0;
        let mut c: u8 = 0;
        for _ in 0..256 {
            c = self.cas_pulse(&mut l)?;
            sum = sum.wrapping_add(c as u16);
        }
        loop {
            // ROM 2939: `prev` is the half-pulse just measured.  Move the running sum
            // towards it by (prev - mean), measure the next half-pulse, and see whether
            // the two together are short enough to be the sync bit.
            let prev = c;
            let mean = (sum >> 8) as u8;
            sum = sum.wrapping_add((prev as i16 - mean as i16) as u16);
            c = self.cas_pulse(&mut l)?;
            let d = (sum >> 8) as u8;
            // 1.25 * D, computed in 8-bit arithmetic exactly as the Z80 does
            // (SRL A; SRL A; ADC A,D  -> D/4 + D + bit 1 of D).
            let limit = (d >> 2).wrapping_add(d).wrapping_add((d >> 1) & 1);
            if limit < prev { continue; }
            if limit - prev < c { continue; }
            // Threshold between a zero and a one bit: 1.5 * D  (RRA; ADC A,D).
            let threshold = (d >> 1).wrapping_add(d).wrapping_add(d & 1);
            self.wr8(PATTERN, l);
            self.wr8(THRESHOLD, threshold);
            break;
        }
        let b = self.cas_read_byte()?;
        if b != self.rd8(MARKER) { return Err(b ^ self.rd8(MARKER)); }
        Ok(())
    }

    /// ROM `29CD`: test ESC on the keyboard matrix, then measure the next half-pulse.
    fn cas_pulse(&mut self, l: &mut u8) -> Result<u8, u8> {
        if self.hw.inp(PPI_A) & 0x04 == 0 { return Err(0); } // ESC (row 8, bit 2) is down
        let mut c = 0u8;
        if self.cas_edge(l, &mut c) { Ok(c) } else { Err(1) }
    }

    /// ROM `29DD`: poll the cassette input until its level matches bit 7 of `l`, counting
    /// 2 per pass in `c`.  Returns `false` if `c` overflows (no signal).  On success `l`
    /// is rotated so the next call waits for the opposite level.
    fn cas_edge(&mut self, l: &mut u8, c: &mut u8) -> bool {
        loop {
            let (n, carry) = c.overflowing_add(2);
            *c = n;
            if carry { return false; }
            let level = self.hw.inp(PPI_B | n as u16);
            if (level ^ *l) & 0x80 == 0 {
                *l = l.rotate_right(1);
                return true;
            }
        }
    }

    /// ROM `29B0`: read eight bits (most significant first).  Each bit is the total length
    /// of one low phase plus one high phase compared with the threshold.  Updates the CRC.
    fn cas_read_byte(&mut self) -> Result<u8, u8> {
        let mut byte = 0u8;
        for _ in 0..8 {
            let mut l = self.rd8(PATTERN);
            let mut c = 0u8;
            if !self.cas_edge(&mut l, &mut c) || !self.cas_edge(&mut l, &mut c) {
                return Err(1);
            }
            let bit = self.rd8(THRESHOLD) < c;
            byte = (byte << 1) | bit as u8;
            self.cas_crc_update(bit);
        }
        Ok(byte)
    }

    /// ROM `2990`: shift one bit into the CRC (CRC-16/CCITT, polynomial `1021`).
    fn cas_crc_update(&mut self, bit: bool) {
        let mut crc = self.rd16(CRC);
        let flag: u8 = if bit { 0xFF } else { 0 };
        let feedback = (flag ^ (crc >> 8) as u8) & 0x80 != 0;
        if feedback { crc ^= 0x0810; }
        crc = (crc << 1) | feedback as u16;
        self.wr16(CRC, crc);
    }

    // ---- writing ---------------------------------------------------------------------

    /// ROM `2964`: silence, 2049 one-bits (leader), a zero bit (sync) and the marker byte.
    fn cas_write_leader(&mut self) -> Result<(), u8> {
        self.cas_short_delay();
        self.cas_write_ones(0x0801)?;
        if !self.cas_write_bit(false) { return Err(1); }
        let marker = self.rd8(MARKER);
        self.cas_write_byte(marker)
    }

    /// ROM `2979`: 33 one-bits after the data.
    fn cas_write_trailer(&mut self) -> Result<(), u8> {
        self.cas_write_ones(0x0021)
    }

    /// ROM `297C`: `n` one-bits.  Checks ESC before each; a failed bit is ignored.
    fn cas_write_ones(&mut self, n: u16) -> Result<(), u8> {
        for _ in 0..n {
            if self.hw.inp(PPI_A) & 0x04 == 0 { return Err(0); }
            self.cas_write_bit(true);
        }
        Ok(())
    }

    /// ROM `29F8`: write a byte, most significant bit first.
    fn cas_write_byte(&mut self, byte: u8) -> Result<(), u8> {
        for i in (0..8).rev() {
            if !self.cas_write_bit(byte >> i & 1 != 0) { return Err(1); }
        }
        Ok(())
    }

    /// ROM `2A08`: write one bit as a low phase followed by a high phase.  Returns `false`
    /// if the bit could not be timed (the ROM's "ran late" case).
    ///
    /// The delay is held in `B8D0`.  A one-bit shortens it by the pre-compensation
    /// (`B8D1`) and loads `B8D0` with `2 × half + precomp` for the following bit; a
    /// zero-bit loads it with the half-period (`B8D2`).
    fn cas_write_bit(&mut self, bit: bool) -> bool {
        let mut delay = self.rd8(WRITE_DELAY);
        let precomp = self.rd8(SPEED_PRECOMP);
        let mut next = self.rd8(SPEED_HALF);
        if bit {
            next = next.wrapping_mul(2).wrapping_add(precomp);
            delay = delay.wrapping_sub(precomp);
        }
        self.wr8(WRITE_DELAY, next);
        // First phase: wait, then drive the output low (PPI bit 5 reset).
        let waited = self.cas_edge_out(delay, 0x0A);
        if !waited {
            // Already late for the first edge: the ROM gives up on the second.
            return false;
        }
        self.cas_crc_update(bit);
        // Second phase: wait again, then drive it high (PPI bit 5 set).
        self.cas_edge_out(delay, 0x0B)
    }

    /// ROM `2A37`: wait `delay` loop passes (16 T-states each) and then program the PPI
    /// with the bit set / reset code `bsr`.  Returns `true` if it had to wait.
    fn cas_edge_out(&mut self, delay: u8, bsr: u8) -> bool {
        self.hw.delay(delay as u32 * 16);
        self.hw.out(PPI_CTL, bsr);
        delay != 0
    }
}
