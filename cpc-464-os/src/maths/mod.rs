//! The arithmetic package: 16-bit integer routines and the 5-byte floating-point package
//! (ROM `2E18`-`37FF`).
//!
//! ## Jumpblock entries
//!
//! The package is reached through 49 jumpblock entries (`BD3D`-`BDCA`; the entry `BD3A`
//! is something else entirely, the line editor, see [`crate::editor`]).  Every routine
//! keeps the Z80 register calling convention in its documentation (`Entry:` / `Exit:`);
//! the Rust signatures return the register values and flags that are meaningful.
//!
//! * [`int`]   – signed/unsigned 16-bit integer arithmetic (`BDA3`-`BDCA`)
//! * [`real`]  – the floating-point core: copy, conversions, add, multiply, divide,
//!               compare (`BD3D`-`BD70`)
//! * [`trans`] – powers of ten, random numbers, log, exp, trigonometry (`BD55`, `BD73`-`BDA0`)
//!
//! ## Number format
//!
//! A *real* is five bytes, least significant first:
//!
//! ```text
//!   byte 0..2   mantissa bits 0..23
//!   byte 3      bit 7 = sign, bits 6..0 = mantissa bits 24..30 (the leading 1 is implicit)
//!   byte 4      exponent, biased by 0x80; 0 means the number is zero
//! ```
//!
//! so `value = (-1)^sign * 1.mantissa * 2^(exponent - 0x81)`.  Only the exponent byte is
//! looked at to detect zero; the other four bytes of a zero are arbitrary.

pub mod int;
pub mod real;
pub mod trans;

pub use int::*;

/// The Z80 flags a routine leaves behind, for the routines whose callers test them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Flags {
    /// Carry flag (almost always "success" or "no overflow").
    pub carry: bool,
    /// Zero flag.
    pub zero: bool,
    /// Sign flag (bit 7 of the last result).
    pub sign: bool,
}

impl Flags {
    /// Flags after a result `v` that sets Z and S from `v` and leaves carry as given.
    pub fn of(v: u8, carry: bool) -> Flags { Flags { carry, zero: v == 0, sign: v & 0x80 != 0 } }
}
