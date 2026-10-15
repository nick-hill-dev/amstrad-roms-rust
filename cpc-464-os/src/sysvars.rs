//! System variables: the firmware's private RAM at `0xB100..0xB8FF` (cleared at boot)
//! plus the RAM-resident jumpblock / indirection area `0xBB00..0xBDFF`.
//!
//! All names are ours; the ROM itself only has addresses.  Multi-byte values are
//! little-endian, exactly as stored by the Z80.

// ===================================================================================
// Kernel – interrupt, event queues, ROM management            (0xB100 .. 0xB1C7)
// ===================================================================================

/// Head of the queue of pending *asynchronous* events (word; high byte 0 = empty).
pub const KL_ASYNC_HEAD: u16 = 0xB100;
/// Tail of that queue (word).
pub const KL_ASYNC_TAIL: u16 = 0xB102;
/// Interrupt-exit work flags: bit0 = tickers due, bit6 = async events queued,
/// bit7 = event processing already running.
pub const KL_FLAGS: u16 = 0xB104;
/// Saved SP while the interrupt stack (below `KL_TIME`) is in use (word).
pub const KL_SAVED_SP: u16 = 0xB105;
/// 32-bit interrupt (1/300 s) counter, least significant byte first.
pub const KL_TIME: u16 = 0xB187;
/// Overflow byte above the 32-bit counter; cleared by `KL TIME SET`.
pub const KL_TIME_OVERFLOW: u16 = 0xB18B;
/// Head of the frame-flyback block list (word).
pub const KL_FRAME_FLY_LIST: u16 = 0xB18C;
/// Head of the fast-ticker block list (word).
pub const KL_FAST_TICKER_LIST: u16 = 0xB18E;
/// Head of the ticker block list (word).
pub const KL_TICKER_LIST: u16 = 0xB190;
/// Counts interrupts 6..1; the keyboard is scanned and tickers run when it hits 0.
pub const KL_TICK_DIVIDER: u16 = 0xB192;
/// Head of the synchronous event queue (word, high byte 0 = empty).
pub const KL_SYNC_HEAD: u16 = 0xB193;
/// Priority (event class) of the synchronous event currently running; bit5 = disabled.
pub const KL_SYNC_PRIORITY: u16 = 0xB195;
/// 16-byte scratch buffer used by `KL FIND COMMAND` for the command name.
pub const KL_COMMAND_NAME: u16 = 0xB196;
/// Head of the chain of logged ROM-extension RAM blocks (word).
pub const KL_ROM_CHAIN: u16 = 0xB1A6;
/// Currently selected ROM (`KL CURR SELECTION`).
pub const KL_CURR_ROM: u16 = 0xB1A8;
/// Foreground-program entry block: address (word) + ROM select byte.
pub const KL_ENTRY_ADDR: u16 = 0xB1A9;
pub const KL_ENTRY_ROM: u16 = 0xB1AB;
/// Table of per-ROM RAM-limit words filled in by `KL INIT BACK` (8 entries).
pub const KL_ROM_TABLE: u16 = 0xB1AA;

// ===================================================================================
// Screen pack                                                (0xB1C8 .. 0xB20B)
// ===================================================================================

/// Current screen mode 0-2.
pub const SCR_MODE: u16 = 0xB1C8;
/// Hardware scroll offset (word, 0..0x7FF, bit 0 normally clear).
pub const SCR_OFFSET: u16 = 0xB1C9;
/// High byte of the screen base address (0x00, 0x40, 0x80 or 0xC0).
pub const SCR_BASE: u16 = 0xB1CB;
/// `JP xxxx` opcode + address of the currently selected pixel-write routine
/// (see `SCR ACCESS`); `BDE8` jumps here.
pub const SCR_WRITE_JP: u16 = 0xB1CC;
/// 8-byte table of pixel masks for the current mode (leftmost pixel first).
pub const SCR_PIXEL_MASKS: u16 = 0xB1CF;
/// Flash periods: low byte = period of the *second* ink set, high byte = *first*.
pub const SCR_FLASH_PERIODS: u16 = 0xB1D7;
/// 17-byte hardware-colour table of the *second* flash colour (border + 16 pens).
pub const SCR_INKS_SECOND: u16 = 0xB1D9;
/// 17-byte hardware-colour table of the *first* flash colour (border + 16 pens).
pub const SCR_INKS_FIRST: u16 = 0xB1EA;
/// Which ink set is on screen next (0 = first, 0xFF = second).
pub const SCR_FLASH_PHASE: u16 = 0xB1FB;
/// Non-zero when `SCR SET INK/BORDER` changed a table and the gate array needs a refresh.
pub const SCR_INKS_DIRTY: u16 = 0xB1FC;
/// Frames left until the flash phase changes.
pub const SCR_FLASH_COUNTDOWN: u16 = 0xB1FD;
/// Frame-flyback event block (chain+event, 9 bytes) driving ink flashing.
pub const SCR_FLASH_BLOCK: u16 = 0xB1FE;
/// Scratch word used by `SCR HORIZONTAL` (right-edge mask table pointer).
pub const SCR_HORIZ_SCRATCH: u16 = 0xB207;

// ===================================================================================
// Text VDU                                                   (0xB20C .. 0xB327)
// ===================================================================================

/// Number of the selected text stream (0..7).
pub const TXT_CURRENT_STREAM: u16 = 0xB20C;
/// Saved state of stream 0; stream *n* lives at `+ n * TXT_STREAM_SIZE`.
pub const TXT_STREAM_SAVE: u16 = 0xB20D;
/// Size of one stream block.
pub const TXT_STREAM_SIZE: u16 = 15;
/// Working copy of the selected stream (15 bytes, see [`StreamOffsets`]).
pub const TXT_STREAM: u16 = 0xB285;

/// Byte offsets inside a stream block (`TXT_STREAM + n`).
pub struct StreamOffsets;
impl StreamOffsets {
    pub const ROW: u16 = 0;          // B285 cursor row (window-relative, absolute here)
    pub const COLUMN: u16 = 1;       // B286 cursor column
    pub const WINDOW_FLAG: u16 = 2;  // B287 non-zero if window is smaller than the screen
    pub const TOP: u16 = 3;          // B288 window top row
    pub const LEFT: u16 = 4;         // B289 window left column
    pub const BOTTOM: u16 = 5;       // B28A window bottom row
    pub const RIGHT: u16 = 6;        // B28B window right column
    pub const SCROLL_COUNT: u16 = 7; // B28C net scroll count
    pub const CURSOR_FLAGS: u16 = 8; // B28D bit0 = disabled, bit1 = off; cursor shown iff 0
    pub const VDU_ENABLED: u16 = 9;  // B28E 0 = VDU disabled, 0xFF = enabled
    pub const PEN: u16 = 10;         // B28F encoded pen
    pub const PAPER: u16 = 11;       // B290 encoded paper
    pub const BACK_VECTOR: u16 = 12; // B291 address of the opaque/transparent writer (word)
    pub const GRAPHICS_MODE: u16 = 14; // B293 non-zero = "write characters at graphics cursor"
}

pub const TXT_CURSOR_ROW: u16 = 0xB285;
pub const TXT_CURSOR_COL: u16 = 0xB286;
pub const TXT_WINDOW_FLAG: u16 = 0xB287;
pub const TXT_WIN_TOP: u16 = 0xB288;
pub const TXT_WIN_LEFT: u16 = 0xB289;
pub const TXT_WIN_BOTTOM: u16 = 0xB28A;
pub const TXT_WIN_RIGHT: u16 = 0xB28B;
pub const TXT_SCROLL_COUNT: u16 = 0xB28C;
pub const TXT_CURSOR_FLAGS: u16 = 0xB28D;
pub const TXT_VDU_ENABLED: u16 = 0xB28E;
pub const TXT_PEN: u16 = 0xB28F;
pub const TXT_PAPER: u16 = 0xB290;
pub const TXT_BACK_VECTOR: u16 = 0xB291;
pub const TXT_GRAPHICS_MODE: u16 = 0xB293;
/// User-defined matrix table: first character (byte) and "present" flag (byte).
pub const TXT_MATRIX_FIRST: u16 = 0xB294;
pub const TXT_MATRIX_FLAG: u16 = 0xB295;
/// Address of the user-defined matrix table (word).
pub const TXT_MATRIX_TABLE: u16 = 0xB296;
/// 32-byte scratch area: one character cell unpacked to screen format.
pub const TXT_CELL_BUFFER: u16 = 0xB298;
/// Number of bytes collected so far for the current control sequence.
pub const TXT_CTRL_COUNT: u16 = 0xB2B8;
/// Control sequence buffer: control code followed by up to 9 parameters.
pub const TXT_CTRL_BUFFER: u16 = 0xB2B9;
/// 32 × 3 byte control-code table: (parameter count, handler address).
pub const TXT_CTRL_TABLE: u16 = 0xB2C3;

// ===================================================================================
// Graphics VDU                                               (0xB328 .. 0xB347)
// ===================================================================================

pub const GRA_ORIGIN_X: u16 = 0xB328;
pub const GRA_ORIGIN_Y: u16 = 0xB32A;
pub const GRA_CURSOR_X: u16 = 0xB32C;
pub const GRA_CURSOR_Y: u16 = 0xB32E;
pub const GRA_WIN_LEFT: u16 = 0xB330;
pub const GRA_WIN_RIGHT: u16 = 0xB332;
/// Window edge nearer the top of the screen (larger y, line units).
pub const GRA_WIN_TOP: u16 = 0xB334;
/// Window edge nearer the bottom of the screen (smaller y).
pub const GRA_WIN_BOTTOM: u16 = 0xB336;
pub const GRA_PEN: u16 = 0xB338;
pub const GRA_PAPER: u16 = 0xB339;
/// Scratch: 8-byte character matrix during `GRA WR CHAR`; line-drawing step (word).
pub const GRA_SCRATCH: u16 = 0xB33A;
pub const GRA_LINE_MAJOR_STEP: u16 = 0xB33A;
pub const GRA_LINE_QUOT: u16 = 0xB33C;
pub const GRA_LINE_REM: u16 = 0xB33E;
pub const GRA_LINE_LEN: u16 = 0xB340;
pub const GRA_LINE_X: u16 = 0xB342;
pub const GRA_LINE_Y: u16 = 0xB344;
pub const GRA_LINE_STEEP: u16 = 0xB346;

// ===================================================================================
// RAM jumpblock & indirections                               (0xBB00 .. 0xBDFF)
// ===================================================================================

pub const JUMPBLOCK_BASE: u16 = 0xBB00;
/// User-patchable indirection vectors (each three bytes: `JP xxxx`).
pub struct Ind;
impl Ind {
    pub const TXT_DRAW_CURSOR: u16 = 0xBDCD;   // default JP 1263
    pub const TXT_UNDRAW_CURSOR: u16 = 0xBDD0; // default JP 1263
    pub const TXT_WRITE_CHAR: u16 = 0xBDD3;    // default JP 134A
    pub const TXT_UNWRITE: u16 = 0xBDD6;       // default JP 13C0
    pub const TXT_OUT_ACTION: u16 = 0xBDD9;    // default JP 140C
    pub const GRA_PLOT: u16 = 0xBDDC;          // default JP 1816
    pub const GRA_TEST: u16 = 0xBDDF;          // default JP 182A
    pub const GRA_LINE: u16 = 0xBDE2;          // default JP 183C
    pub const SCR_READ: u16 = 0xBDE5;          // default JP 0C82
    pub const SCR_WRITE: u16 = 0xBDE8;         // default JP 0C68
    pub const SCR_MODE_CLEAR: u16 = 0xBDEB;    // default JP 0AF7
    pub const KM_TEST_BREAK: u16 = 0xBDEE;     // default JP 1C2F
    pub const MC_WAIT_PRINTER: u16 = 0xBDF1;   // default JP 07F8
}

// ===================================================================================
// Maths scratch area                                         (0xB8E4 .. 0xB8F7)
// ===================================================================================

/// The 32-bit random number seed (`B8E4..B8E7`, little endian; reset value `0x89656C07`).
pub const MATHS_SEED: u16 = 0xB8E4;
/// Scratch real "T1" (5 bytes).
pub const MATHS_T1: u16 = 0xB8E8;
/// Scratch real "T2" (5 bytes): usually the unmodified argument of a series evaluation.
pub const MATHS_T2: u16 = 0xB8ED;
/// Scratch real "T3" (5 bytes): the variable the series polynomial is evaluated in.
pub const MATHS_T3: u16 = 0xB8F2;
/// Angle mode: 0 = radians, non-zero = degrees (`BD73`).
pub const MATHS_ANGLE_MODE: u16 = 0xB8F7;

// ---- Line editor (BD3A) ---------------------------------------------------------
/// Non-zero if printing text moved the "copy cursor" (it must then be shifted too).
pub const ED_COPY_MOVED: u16 = 0xB8DC;
/// Non-zero while the editor is in overwrite mode (cleared on entry).
pub const ED_OVERWRITE: u16 = 0xB8DD;
/// Position of the COPY cursor, `H = column`, `L = row` (window relative, 1-based),
/// or zero if there is no copy cursor.
pub const ED_COPY_CURSOR: u16 = 0xB8DE;

/// Scratch word used by `SCR HORIZONTAL`: `FF00 | !(pixels_per_byte - 1)`.
pub const SCR_LINE_SCRATCH: u16 = 0xB207;
