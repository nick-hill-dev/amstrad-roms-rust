//! # cpc464-os
//!
//! A readable Rust reconstruction of the **Amstrad CPC464 lower ROM** (firmware v1.0,
//! the 16 KiB image normally called `OS464.ROM`).
//!
//! The ROM was disassembled (Z80) and every routine was then re-expressed as a Rust
//! function working on an emulated 64 KiB memory map, using the same data layout the
//! firmware itself uses.  Each function documents
//!
//! * the ROM address it was recovered from,
//! * its original register calling convention (`Entry:` / `Exit:`), and
//! * any behaviour that is surprising or only makes sense in terms of the Z80 code.
//!
//! ## Layout of the ROM
//!
//! | ROM range     | Contents                                                          | Rust module        |
//! |---------------|-------------------------------------------------------------------|--------------------|
//! | `0000-00FF`   | RST vectors, boot copy, time, interrupt tick                      | [`kernel`]         |
//! | `0100-038F`   | frame-fly / ticker / event queues, ROM extension logging          | [`kernel`]         |
//! | `0391-057F`   | RAM-resident kernel (copied to `B900`): ROM banking, far calls    | [`kernel`] (`banking`) |
//! | `0580-0888`   | cold start, machine pack (gate array, CRTC, printer, PSG)         | [`machine`]        |
//! | `0888-0A89`   | jumpblock generator + table of 239 entry points                   | [`jumpblock`]      |
//! | `0AA0-1077`   | screen pack                                                       | [`screen`]         |
//! | `1078-15AF`   | text VDU                                                          | [`text`]           |
//! | `15B0-19DF`   | graphics VDU                                                      | [`graphics`]       |
//! | `19E0-1E67`   | keyboard manager                                                  | [`keyboard`]       |
//! | `1E68-236F`   | sound manager                                                     | [`sound`]          |
//! | `2370-2A8F`   | cassette manager                                                  | [`cassette`]       |
//! | `2A98-2E0C`   | BASIC line editor (`BD3A`)                                        | [`editor`]         |
//! | `2E18-37FF`   | floating-point / integer arithmetic package                       | [`maths`]          |
//! | `3800-3FFF`   | 256-character 8×8 font                                            | [`font`]           |
//!
//! ## Modelling decisions
//!
//! * Memory is a flat `[u8; 65536]` ([`Cpc::ram`]) plus the 16 KiB lower ROM image
//!   ([`Cpc::rom`]).  Firmware variables are addressed with the original addresses
//!   (see [`sysvars`]) so machine-code programs sharing the structures keep working.
//! * Registers are replaced by ordinary parameters and return values; the Z80 carry /
//!   zero flags that routines use to report success become `bool` / `Option`.
//! * Hardware access goes through the [`Hardware`] trait.
//! * ROM banking (`KL U ROM ENABLE` ...) is tracked in [`Cpc::banking`]; the CPU view of
//!   memory is available through [`Cpc::read`].
//! * User-patchable indirections (`BDCD..BDF1`) live in RAM as `JP nnnn`, like on the real
//!   machine.  A routine that goes through an indirection checks whether the jump still
//!   points at the ROM default and, if a program has patched it, calls the patched code
//!   through [`Hardware::call`].

pub mod cassette;
pub mod editor;
pub mod font;
pub mod graphics;
pub mod hw;
pub mod jumpblock;
pub mod kernel;
pub mod keyboard;
pub mod machine;
pub mod maths;
pub mod screen;
pub mod sound;
pub mod sysvars;
pub mod text;

pub use hw::{Hardware, NullHardware, Regs};

/// ROM/RAM banking state – mirrors the bits of the gate-array "mode/ROM" register
/// that the kernel manipulates, plus the selected upper ROM.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Banking {
    /// Lower ROM (0000-3FFF) visible instead of RAM.
    pub lower_rom: bool,
    /// Upper ROM (C000-FFFF) visible instead of RAM.
    pub upper_rom: bool,
    /// Currently selected upper ROM number (port `DFxx`).
    pub upper_rom_number: u8,
    /// The last value written to the gate array mode/ROM register (`0x80 | ...`).
    pub mode_rom_register: u8,
}

impl Default for Banking {
    fn default() -> Self {
        // Reset state: `OUT (C),C` with C=0x89 -> mode 1, lower ROM on, upper ROM off.
        Banking { lower_rom: true, upper_rom: false, upper_rom_number: 0, mode_rom_register: 0x89 }
    }
}

/// The whole firmware state: memory + hardware.
pub struct Cpc<H: Hardware> {
    /// 64 KiB of RAM (the screen lives in here at `C000` after reset).
    pub ram: Box<[u8; 0x10000]>,
    /// The 16 KiB lower ROM image (`OS464.ROM`).  Only the font and a few tables are
    /// ever read from it by the Rust port.
    pub rom: Box<[u8; 0x4000]>,
    /// Port-level access to the machine.
    pub hw: H,
    /// ROM banking state.
    pub banking: Banking,
    /// `true` while the interrupt service code runs.  On the Z80 this is the carry flag
    /// of the *alternate* AF register pair (set by the interrupt entry, cleared on
    /// exit); `KL EVENT` consults it to decide whether to run an event now or queue it.
    pub in_interrupt: bool,
    /// Set when the keyboard scan sees CONTROL+SHIFT+ESC: the firmware executes `RST 0`.
    /// The host should then call [`Cpc::power_on`] (the reset is not performed from inside
    /// the interrupt because it would have to start a new program).
    pub reset_requested: bool,
}

impl<H: Hardware> Cpc<H> {
    /// Create a machine around a lower-ROM image.  Panics if `rom` is not 16 KiB.
    pub fn new(rom: &[u8], hw: H) -> Self {
        assert_eq!(rom.len(), 0x4000, "lower ROM image must be exactly 16384 bytes");
        let mut r = Box::new([0u8; 0x4000]);
        r.copy_from_slice(rom);
        Cpc {
            ram: Box::new([0u8; 0x10000]),
            rom: r,
            hw,
            banking: Banking::default(),
            in_interrupt: false,
            reset_requested: false,
        }
    }

    // ---- raw RAM access (what the firmware does to its own variables) -------------
    #[inline] pub fn rd8(&self, addr: u16) -> u8 { self.ram[addr as usize] }
    #[inline] pub fn wr8(&mut self, addr: u16, v: u8) { self.ram[addr as usize] = v; }
    #[inline] pub fn rd16(&self, addr: u16) -> u16 {
        self.ram[addr as usize] as u16 | (self.ram[addr.wrapping_add(1) as usize] as u16) << 8
    }
    #[inline] pub fn wr16(&mut self, addr: u16, v: u16) {
        self.ram[addr as usize] = v as u8;
        self.ram[addr.wrapping_add(1) as usize] = (v >> 8) as u8;
    }

    /// Is the indirection at `ind` (a `JP nnnn` in RAM) still the ROM default?  Returns the
    /// address to call through [`Hardware::call`] if a program has changed it.
    pub(crate) fn patched(&self, ind: u16, default: u16) -> Option<u16> {
        if self.rd8(ind) == 0xC3 && self.rd16(ind + 1) == default { None } else { Some(ind) }
    }

    /// CPU view of memory: the lower ROM shadows `0000-3FFF` and the selected upper ROM
    /// shadows `C000-FFFF` while they are enabled.
    pub fn read(&self, addr: u16) -> u8 {
        if addr < 0x4000 && self.banking.lower_rom {
            self.rom[addr as usize]
        } else if addr >= 0xC000 && self.banking.upper_rom {
            self.hw.upper_rom_byte(self.banking.upper_rom_number, addr - 0xC000)
        } else {
            self.ram[addr as usize]
        }
    }
    pub fn read16(&self, addr: u16) -> u16 {
        self.read(addr) as u16 | (self.read(addr.wrapping_add(1)) as u16) << 8
    }
}
