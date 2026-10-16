//! BASIC's variables in RAM: the workspace `AC00-B0FF` reserved for the foreground ROM.
//!
//! All names are ours; the ROM only has addresses.  Multi-byte values are little-endian.
//!
//! ## Memory layout of a running BASIC
//!
//! ```text
//!   0040 ..        (PROGRAM_BASE)   1 spare byte, then the program lines
//!   PROGRAM_END                     end of the program = start of simple variables
//!   VARS_START                        end of simple variables = start of arrays  (often called "array base")
//!   ARRAYS_END                      end of arrays; free memory follows
//!   ...            free space (grows from both ends)
//!   STRINGS_BOTTOM ..STRINGS_TOP    string space, growing down from the top
//!   HIMEM          (+1 = start of user memory reserved by MEMORY / SYMBOL AFTER / buffers)
//!   AC00-B0FF      this workspace
//! ```
//!
//! A tokenised program line is `length(2) line-number(2) tokens... 00`; the program ends
//! with a zero length.  [`PROGRAM_BASE`] holds the address of the byte *before* the first
//! line (that byte is always zero, so the text before the first line looks like the end of
//! a line).

// ===================================================================================
// RAM hooks, streams                                                (AC00 .. AC36)
// ===================================================================================

/// Set to 0 at cold start and never changed by the ROM itself.  The tokeniser drops
/// spaces (and LIST prints them) only while it is 0 - a flag for extensions.
pub const EXT_FLAG: u16 = 0xAC00;
/// Nine three-byte patch points (`AC01, AC04 ... AC19`), initialised to `RET`.  The ROM
/// calls them at interesting moments so an extension (a disc ROM) can intercept:
///
/// | hook   | called from                                   |
/// |--------|-----------------------------------------------|
/// | `AC01` | the `Ready` loop, before each prompt (C064)   |
/// | `AC04` | the error handler, first thing (CA94)         |
/// | `AC07` | statement token out of range (DDC3)           |
/// | `AC0A` | function token out of range (D0A9)            |
/// | `AC0D` | operand token not recognised (D078)           |
/// | `AC10` | tokeniser, per item (DEE1)                    |
/// | `AC13` | de-tokeniser, per item (E196)                 |
/// | `AC16` | keyword search while tokenising (DF51)        |
/// | `AC19` | keyword not found when listing (E30B)         |
pub const HOOKS: u16 = 0xAC01;
pub const HOOK_READY: u16 = 0xAC01;
pub const HOOK_ERROR: u16 = 0xAC04;
pub const HOOK_BAD_STATEMENT: u16 = 0xAC07;
pub const HOOK_BAD_FUNCTION: u16 = 0xAC0A;
pub const HOOK_BAD_OPERAND: u16 = 0xAC0D;
pub const HOOK_TOKENISE: u16 = 0xAC10;
pub const HOOK_DETOKENISE: u16 = 0xAC13;
pub const HOOK_KEYWORD: u16 = 0xAC16;
pub const HOOK_NO_KEYWORD: u16 = 0xAC19;

/// `AUTO` active (`FF`) or not (`00`).
pub const AUTO_ACTIVE: u16 = 0xAC1C;
/// Next line number `AUTO` offers (word).
pub const AUTO_LINE: u16 = 0xAC1D;
/// `AUTO` increment (word).
pub const AUTO_STEP: u16 = 0xAC1F;
/// Current output stream: 0-7 screen windows, 8 printer, 9 cassette file.
pub const OUT_STREAM: u16 = 0xAC21;
/// Current input stream: 0-7 keyboard, 9 cassette file.
pub const IN_STREAM: u16 = 0xAC22;
/// Printer column (for `POS(#8)`), 0-based.
pub const PRINTER_COLUMN: u16 = 0xAC23;
/// `WIDTH` (printer line width), `FF` = no automatic new line.
pub const PRINTER_WIDTH: u16 = 0xAC24;
/// Column of the cassette output file (`POS(#9)`), 1-based.
pub const FILE_COLUMN: u16 = 0xAC25;
/// 0 while `FOR` runs the first loop test (the step is not added), `FF` for `NEXT`.
pub const FOR_STEP_FLAG: u16 = 0xAC26;
/// 5-byte scratch value used while setting up `FOR` (the start value).
pub const FOR_SCRATCH: u16 = 0xAC27;
/// Text position of the `NEXT` that closes the `FOR` being set up (it identifies the
/// loop's frame).
pub const FOR_VARIABLE: u16 = 0xAC2C;
/// The line of the `WEND` while a `WHILE` condition is tested.
pub const WHILE_SCRATCH: u16 = 0xAC2E;

// ===================================================================================
// Events                                                             (AC30 .. ACA3)
// ===================================================================================

/// Flags gathered while processing synchronous events (see [`crate::events`]):
/// bit 0 break (no `ON BREAK GOSUB`), bit 1 an event `GOSUB` was set up, bit 2 re-arm
/// the break key, bit 6 stop processing events, bit 7 do not call `KL DONE SYNC` (the
/// event's `RETURN` will).
pub const EVENT_FLAGS: u16 = 0xAC30;
/// The `ON BREAK GOSUB` pseudo event's extra bytes, laid out like those of the other
/// events: the saved priority (`AC31`), the interrupted statement (`AC32`), the line
/// (`AC34`).
pub const BREAK_GOSUB: u16 = 0xAC31;
/// The statement interrupted by `ON BREAK GOSUB`.
pub const EVENT_SAVED_PTR: u16 = 0xAC32;
/// `ON BREAK GOSUB` line (address of the byte before the line; 0 = none).
pub const ON_BREAK_LINE: u16 = 0xAC34;
/// The break event's routine pointer (event block + 4), saved by the break routine.
pub const BREAK_SAVED_PTR: u16 = 0xAC36;
/// Three `ON SQ` event blocks of 12 bytes (channels A, B, C): the firmware event block
/// (7), the saved priority (1), the interrupted statement (2), the `GOSUB` line (2).
pub const SQ_EVENTS: u16 = 0xAC38;
/// Four `AFTER`/`EVERY` ticker blocks of 18 bytes (timers 0-3): ticker chain (2),
/// count (2), recharge (2), then an event block laid out like the `ON SQ` ones (12).
pub const TICKERS: u16 = 0xAC5C;

/// The line input buffer (256 bytes): typed lines, `INPUT`, file input.
pub const INPUT_BUFFER: u16 = 0xACA4;

// ===================================================================================
// Errors, CONT, ON ERROR                                             (ADA6 .. ADB1)
// ===================================================================================

/// Line (address of the line's number word) where the last error happened (`ERL`).
pub const ERROR_LINE: u16 = 0xADA6;
/// Statement pointer where the last error happened (for `RESUME`).
pub const ERROR_STATEMENT: u16 = 0xADA8;
/// Number of the last error (`ERR`).
pub const ERROR_NUMBER: u16 = 0xADAA;
/// Where `CONT` continues (statement pointer; 0 = cannot continue).
pub const CONT_STATEMENT: u16 = 0xADAB;
/// Line `CONT` continues in.
pub const CONT_LINE: u16 = 0xADAD;
/// `ON ERROR GOTO` line address (0 = none).
pub const ON_ERROR_LINE: u16 = 0xADAF;
/// Non-zero while an `ON ERROR` handler runs (cleared by `RESUME`).
pub const IN_ERROR_HANDLER: u16 = 0xADB1;

// ===================================================================================
// Sound parameter blocks                                             (ADB2 .. ADCA)
// ===================================================================================

/// The 9-byte `SOUND QUEUE` parameter block built by `SOUND`.
pub const SOUND_PARAMS: u16 = 0xADB2;
/// The envelope block built by `ENV` / `ENT` (count + 5 sections of 3 bytes).
pub const ENVELOPE_PARAMS: u16 = 0xADBB;

// ===================================================================================
// Variables                                                          (ADCB .. AE25)
// ===================================================================================

/// Five-byte scratch real (the `^` operator, `DEFINT` table base pointer arithmetic).
pub const SCRATCH_REAL: u16 = 0xADCB;
/// 26 list heads (word, offset from `VARS_START - 1`, see [`crate::variables`]) of the
/// simple variables, one per initial letter `A..Z`.
pub const VAR_HEADS: u16 = 0xADD0;
/// List head of the `DEF FN` functions.
pub const FN_HEAD: u16 = 0xAE04;
/// List heads of the arrays: real (`AE06`), integer (`AE08`), string (`AE0A`).
pub const ARRAY_HEADS: u16 = 0xAE06;
/// `DEFINT`/`DEFSTR`/`DEFREAL` table: the default type (2, 3 or 5) for each letter.
pub const DEFAULT_TYPES: u16 = 0xAE0C;

/// Scratch byte of the variable code ("create if missing" flag of the array code ...).
pub const VAR_SCRATCH: u16 = 0xAE26;
/// Address of the name of the variable being looked up (it is copied onto the BASIC stack).
pub const VAR_NAME: u16 = 0xAE27;
/// Head of the chain of active `DEF FN` parameter frames.
pub const FN_FRAMES: u16 = 0xAE29;
/// Frame whose parameters are currently visible.
pub const FN_ACTIVE_FRAME: u16 = 0xAE2B;

// ===================================================================================
// Interpreter                                                        (AE2D .. AE45)
// ===================================================================================

/// The separator after the `INPUT` prompt (`;` suppresses the `?`).
pub const INPUT_SEPARATOR: u16 = 0xAE2D;
/// Line of the `DATA` statement being read (for the error line).
pub const DATA_LINE: u16 = 0xAE2E;
/// `READ` pointer into the `DATA` statements.
pub const DATA_PTR: u16 = 0xAE30;
/// BASIC stack pointer at the start of the current statement.
pub const STATEMENT_STACK: u16 = 0xAE32;
/// Address of the current statement.
pub const STATEMENT_PTR: u16 = 0xAE34;
/// Address of the current line's number (0 = direct mode).
pub const CURRENT_LINE: u16 = 0xAE36;
/// `TRON` active.
pub const TRACE: u16 = 0xAE38;
/// Tokeniser state: what the previous item allows next (see [`crate::tokens`]).
pub const TOKENISER_STATE: u16 = 0xAE39;
/// Non-zero when line-number references in the program have been converted to line
/// addresses (token `1D`) and must be converted back before the program is edited.
pub const LINE_REFS_ARE_ADDRESSES: u16 = 0xAE3A;
/// `DELETE`/`LIST` range: address of the first line and length.
pub const RANGE_START: u16 = 0xAE3B;
pub const RANGE_LENGTH: u16 = 0xAE3D;
/// `LOAD` address given after the file name.
pub const LOAD_ADDRESS: u16 = 0xAE3F;
/// Non-zero for `MERGE` (vs `CHAIN MERGE` handling).
pub const MERGE_FLAG: u16 = 0xAE41;
/// File type byte of the file just opened.
pub const FILE_TYPE: u16 = 0xAE42;
/// File length of the file just opened.
pub const FILE_LENGTH: u16 = 0xAE43;
/// Program is protected (`SAVE ,P`): any return to `Ready` erases it.
pub const PROTECTED: u16 = 0xAE45;

// ===================================================================================
// Number conversion                                                  (AE46 .. AE7A)
// ===================================================================================

/// Digit buffer used by number parsing and formatting.
pub const DIGITS: u16 = 0xAE46;
/// Formatted number output buffer (ends at `AE68`).
pub const NUMBER_TEXT: u16 = 0xAE68;
/// `PRINT USING` field: digits after the point / flags.
pub const FORMAT_A: u16 = 0xAE6E;
pub const FORMAT_B: u16 = 0xAE6F;
/// End pointer of the formatted number.
pub const NUMBER_END: u16 = 0xAE70;
/// `CALL`: address, ROM select, return text pointer, saved `SP`.
pub const CALL_ADDRESS: u16 = 0xAE72;
pub const CALL_ROM: u16 = 0xAE74;
pub const CALL_SAVED_PTR: u16 = 0xAE75;
pub const CALL_SAVED_SP: u16 = 0xAE77;
/// `ZONE` width.
pub const ZONE: u16 = 0xAE79;
/// `PRINT USING`: non-zero once the format string has been used up once.
pub const USING_DONE: u16 = 0xAE7A;

// ===================================================================================
// Memory layout                                                      (AE7B .. B09A)
// ===================================================================================

/// `HIMEM`: the highest byte BASIC may use.
pub const HIMEM: u16 = 0xAE7B;
/// Top of memory as left by the firmware / the last `MEMORY` (cassette buffers and the
/// user-defined character matrices are allocated downwards from here).
pub const MEMORY_TOP: u16 = 0xAE7D;
/// Lowest byte of memory given to BASIC (`DE` at cold start).
pub const MEMORY_BOTTOM: u16 = 0xAE7F;
/// Address of the zero byte before the first program line.
pub const PROGRAM_BASE: u16 = 0xAE81;
/// End of the program (after its terminating zero length).
pub const PROGRAM_END: u16 = 0xAE83;
/// Start of the simple variables and `DEF FN`s (normally = [`PROGRAM_END`]).  The
/// links of their lists are offsets from `VARS_START - 1`.
pub const VARS_START: u16 = 0xAE85;
/// End of the simple variables (= start of the arrays; array links are offsets from
/// `ARRAYS_START - 1`).
pub const ARRAYS_START: u16 = 0xAE87;
/// End of the arrays (= start of free memory).
pub const ARRAYS_END: u16 = 0xAE89;
/// Base of BASIC's control stack (`FOR`, `GOSUB`, `WHILE` frames, temporaries).
pub const STACK_BASE: u16 = 0xAE8B;
/// BASIC's stack pointer (grows upwards from [`STACK_BASE`]; the stack is full when it
/// would pass `B088`).
pub const STACK_PTR: u16 = 0xB08B;
/// Lowest byte in use by strings (the string space grows downwards).
pub const STRINGS_BOTTOM: u16 = 0xB08D;
/// Top of the string space.
pub const STRINGS_TOP: u16 = 0xB08F;
/// Cassette buffer state: bit 0 input buffer allocated, bit 1 output..., bit 2 the
/// buffers are in use / were allocated by BASIC.
pub const CASSETTE_BUFFERS: u16 = 0xB091;
/// Address of the 4 KiB cassette buffer area.
pub const CASSETTE_BUFFER: u16 = 0xB092;
/// [`MEMORY_TOP`] before the cassette buffers were allocated.
pub const CASSETTE_SAVED_TOP: u16 = 0xB094;
/// [`MEMORY_TOP`] before the user character matrices were allocated.
pub const SYMBOL_SAVED_TOP: u16 = 0xB096;
/// Distance a memory block is being moved by (`MEMORY`, garbage collection).
pub const MOVE_DELTA: u16 = 0xB098;

// ===================================================================================
// Strings and the accumulator                                        (B09A .. B0C6)
// ===================================================================================

/// String descriptor stack pointer (temporaries: next free entry).
pub const DESCRIPTOR_SP: u16 = 0xB09A;
/// The descriptor stack: ten 3-byte entries.
pub const DESCRIPTOR_STACK: u16 = 0xB09C;
/// One past the descriptor stack.
pub const DESCRIPTOR_STACK_END: u16 = 0xB0BA;
/// The result descriptor (length, address) of the last string operation.
pub const TEMP_DESCRIPTOR: u16 = 0xB0BA;
/// Garbage collector: descriptor of the highest string not yet moved, and its address.
pub const GC_DESCRIPTOR: u16 = 0xB0BD;
pub const GC_LIMIT: u16 = 0xB0BF;
/// Type of the value in the accumulator: 2 integer, 3 string, 5 real.
pub const ACC_TYPE: u16 = 0xB0C1;
/// The accumulator: an integer (2 bytes), a string descriptor address (2 bytes) or a
/// real (5 bytes).
pub const ACC: u16 = 0xB0C2;

/// Value types as stored in [`ACC_TYPE`] and in variable records.
pub const T_INT: u8 = 2;
pub const T_STR: u8 = 3;
pub const T_REAL: u8 = 5;
