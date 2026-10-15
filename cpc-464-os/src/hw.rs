//! Hardware abstraction.
//!
//! The original firmware talks to the machine through Z80 `IN`/`OUT` instructions.
//! The Rust port keeps that structure: every port access goes through the
//! [`Hardware`] trait so the code can run against a real emulator, a test stub, or
//! nothing at all.
//!
//! ## CPC I/O map (the bits the lower ROM uses)
//!
//! | Port (high byte)  | Device                                                     |
//! |-------------------|------------------------------------------------------------|
//! | `0x7Fxx`          | Gate Array: pen select / colour / screen mode + ROM / RAM  |
//! | `0xBCxx`/`0xBDxx` | CRTC 6845 register select / data                           |
//! | `0xDFxx`          | Upper-ROM bank select                                      |
//! | `0xEFxx`          | Centronics printer data + strobe (bit 7)                   |
//! | `0xF4xx`          | PPI 8255 port A (PSG data bus / keyboard read)             |
//! | `0xF5xx`          | PPI port B (vsync, printer busy, cassette in, 50Hz jumper) |
//! | `0xF6xx`          | PPI port C (PSG control, keyboard row, cassette motor/out) |
//! | `0xF7xx`          | PPI control register                                       |

/// Register file handed to machine-code routines that the firmware has to *call*
/// (event handlers, user-patched indirections, ROM entry points ...).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Regs {
    pub af: u16,
    pub bc: u16,
    pub de: u16,
    pub hl: u16,
    pub ix: u16,
    pub iy: u16,
}

impl Regs {
    pub fn a(&self) -> u8 { (self.af >> 8) as u8 }
    pub fn set_a(&mut self, v: u8) { self.af = (self.af & 0x00FF) | ((v as u16) << 8); }
    pub fn carry(&self) -> bool { self.af & 1 != 0 }
    pub fn set_carry(&mut self, c: bool) { self.af = (self.af & !1) | c as u16; }
}

/// What the firmware needs from the outside world.
pub trait Hardware {
    /// Z80 `OUT (C),r` – `port` is the full 16-bit BC value.
    fn out(&mut self, port: u16, value: u8);
    /// Z80 `IN r,(C)`.
    fn inp(&mut self, port: u16) -> u8;
    /// Call a machine-code routine located at `addr` in the CPU's current memory
    /// configuration.  Used for event routines, user-patched indirection vectors and
    /// anything else the ROM reaches through a RAM pointer.  A hardware stub can
    /// simply ignore it.
    fn call(&mut self, _addr: u16, _regs: &mut Regs) {}
    /// Variant of [`Hardware::call`] for "far" addresses: `rom` is the ROM select
    /// byte (0-251 = upper ROM number, 252-255 = RAM/lower-ROM combinations).
    fn far_call(&mut self, addr: u16, _rom: u8, regs: &mut Regs) { self.call(addr, regs) }
    /// Content of byte `offset` (0-0x3FFF) of upper ROM number `rom` (as seen at
    /// `C000`), or `0xFF` if no such ROM is fitted.  Needed by the ROM-scanning
    /// routines (`KL ROM WALK`, `KL FIND COMMAND` ...).
    fn upper_rom_byte(&self, _rom: u8, _offset: u16) -> u8 { 0xFF }
    /// Let `t_states` Z80 clock cycles (4 MHz) elapse.  Only the cassette routines need
    /// this: they generate pulses by busy-waiting, and a host that wants to record or
    /// play back a tape has to know how long each wait lasts.  Default: ignore.
    fn delay(&mut self, _t_states: u32) {}
}

/// A `Hardware` implementation that does nothing and reads back `0xFF`; handy for
/// running the pure parts of the firmware in unit tests.
#[derive(Default, Debug)]
pub struct NullHardware {
    /// Every `(port, value)` written, in order.
    pub log: Vec<(u16, u8)>,
}

impl Hardware for NullHardware {
    fn out(&mut self, port: u16, value: u8) { self.log.push((port, value)); }
    fn inp(&mut self, _port: u16) -> u8 { 0xFF }
}

// ---- port numbers (high byte only is decoded by the real hardware) --------------
pub const GATE_ARRAY: u16 = 0x7F00;
pub const CRTC_SELECT: u16 = 0xBC00;
pub const CRTC_DATA: u16 = 0xBD00;
pub const ROM_SELECT: u16 = 0xDF00;
pub const PRINTER: u16 = 0xEF00;
pub const PPI_A: u16 = 0xF400;
pub const PPI_B: u16 = 0xF500;
pub const PPI_C: u16 = 0xF600;
pub const PPI_CTL: u16 = 0xF700;

/// Gate-array function codes (top two bits of the byte written to `0x7Fxx`).
pub const GA_PEN_SELECT: u8 = 0x00;
pub const GA_COLOUR: u8 = 0x40;
pub const GA_MODE_ROM: u8 = 0x80;

/// Bits in the gate-array "mode / ROM" byte (`0x80 | ...`).
pub const GA_MODE_MASK: u8 = 0x03;
pub const GA_LOWER_ROM_OFF: u8 = 0x04;
pub const GA_UPPER_ROM_OFF: u8 = 0x08;
pub const GA_INT_RESET: u8 = 0x10;
