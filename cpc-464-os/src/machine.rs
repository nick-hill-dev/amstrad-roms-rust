//! Machine pack (`MC`) and the cold-start sequence.
//!
//! ROM `0000-0005` (reset vector), `0044-005B` (vector/RAM-code copy),
//! `0580-0888` (hardware reset, boot/start program, gate array, CRTC, printer, PSG).

use crate::hw::*;
use crate::sysvars::*;
use crate::{Cpc, Regs};

/// CRTC register values for the 50 Hz display (`R0..R15`), ROM `05B4`.
pub const CRTC_50HZ: [u8; 16] = [
    0x3F, 0x28, 0x2E, 0x8E, 0x26, 0x00, 0x19, 0x1E, 0x00, 0x07, 0x00, 0x00, 0x30, 0x00, 0xC0, 0x00,
];
/// CRTC register values for the 60 Hz display, ROM `05C4` (only R4,R5,R7 differ).
pub const CRTC_60HZ: [u8; 16] = [
    0x3F, 0x28, 0x2E, 0x8E, 0x1F, 0x06, 0x19, 0x1B, 0x00, 0x07, 0x00, 0x00, 0x30, 0x00, 0xC0, 0x00,
];

/// Machine-maker names selected by the three links on PPI port B (ROM `0727`).
/// The string table in the ROM starts with "Arnold" (never reached with the
/// `index + 1` skip the code uses) – kept here for completeness.
pub const MAKER_NAMES: [&str; 9] =
    ["Arnold", "\n Amstrad", "\n Orion", "\n Schneider", "\n Awa", "\n Solavox", "\n Saisho", "\n Triumph", "\n Isp"];

/// Banner printed after reset (ROM `066D` / `0693`). `\xA4` is the CPC copyright glyph.
///
/// The ROM stores the word `Copyright` in front of the second string but the code
/// points at the space after it (`0693`), so the word is never shown.
pub const BANNER_PRODUCT: &str = " 64K Microcomputer  (v1)\r\n\r\n";
pub const BANNER_COPYRIGHT: &str =
    " \u{a4}1984 Amstrad Consumer Electronics plc\r\n           and Locomotive Software Ltd.\r\n\r\n";
/// Printed by `MC BOOT PROGRAM` when the loader reports failure (ROM `06F4`).
pub const LOAD_FAILED: &str = "*** PROGRAM LOAD FAILED ***\r\n";

/// What to print after the hardware and OS have been reset (the `DE` register of the
/// restart routine at `060E`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Announce {
    /// Power-on banner (ROM `065C`).
    ColdStartBanner,
    /// "*** PROGRAM LOAD FAILED ***" (ROM `06E8`).
    LoadFailed,
    /// Nothing (ROM `0726` is just a `RET`).
    Nothing,
}

/// Where control goes after a restart: `(entry address, ROM select)`.  Handed back to the
/// caller because jumping into the foreground program is outside this crate's remit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StartTarget {
    pub entry: u16,
    pub rom: u8,
    /// The `HL, DE, BC` values the ROM passes to the program: free-memory limits.
    pub hl: u16,
    pub de: u16,
    pub bc: u16,
}

impl<H: Hardware> Cpc<H> {
    // ===============================================================================
    // Reset / start-up                                              (0000, 0580 ..)
    // ===============================================================================

    /// Power-on reset (`RST 0` -> `0000` -> `0580`).
    ///
    /// 1. Hardware set-up: PPI mode, PSG quiet, printer strobe high, CRTC registers
    ///    (50 or 60 Hz according to the link on PPI port B bit 4).
    /// 2. [`Self::restart`] with the cold-start banner and BASIC (`HL = 0`) as the
    ///    foreground program.
    pub fn power_on(&mut self) -> StartTarget {
        self.hw.out(PPI_CTL, 0x82);   // port A out, B in, C out
        self.hw.out(PPI_A, 0x00);
        self.hw.out(PPI_C, 0x00);
        self.hw.out(PRINTER, 0x7F);   // printer strobe high (inactive)
        let fifty = self.hw.inp(PPI_B) & 0x10 != 0;
        let table = if fifty { &CRTC_50HZ } else { &CRTC_60HZ };
        for reg in (0..16u8).rev() {
            self.hw.out(CRTC_SELECT, reg);
            self.hw.out(CRTC_DATA, table[reg as usize]);
        }
        self.restart(Announce::ColdStartBanner, 0, 0xFF)
    }

    /// The common restart routine (ROM `060E`), used by cold start, [`Self::mc_start_program`]
    /// and the failed-load path of [`Self::mc_boot_program`].
    ///
    /// * clears the system RAM `B100-B8FF`, selects mode 1 / lower ROM on / upper ROM off,
    /// * re-creates the page-zero vectors, the RAM-resident kernel and the jumpblock,
    /// * initialises every manager, enables interrupts,
    /// * runs the `announce` routine,
    /// * and finally returns where the foreground program must be entered (ROM `0077`).
    ///
    /// `hl`/`c` select the program: `hl == 0` means "BASIC in ROM 0 at `C006`"; otherwise
    /// entry address `hl` in ROM `c`.
    pub fn restart(&mut self, announce: Announce, hl: u16, c: u8) -> StartTarget {
        self.hw.out(ROM_SELECT, 0);                // upper ROM 0
        self.hw.out(0xF8FF, 0xFF);          // reset peripherals
        for a in 0xB100u16..0xB900 { self.ram[a as usize] = 0; }
        self.banking = crate::Banking::default();
        self.hw.out(GATE_ARRAY, 0x89);
        self.in_interrupt = false;

        self.install_vectors();                    // 0044
        self.jump_restore();                       // 0888: BB00.. jumpblock
        self.km_initialise();                      // 19E0
        self.sound_reset();                        // 1E68
        self.scr_initialise();                     // 0AA0
        self.txt_initialise();                     // 1078
        self.gra_initialise();                     // 15B0
        self.cas_initialise();                     // 2370
        self.mc_reset_printer();                   // 07E6
        // EI – from here the interrupt service may run

        match announce {
            Announce::ColdStartBanner => {
                let who = self.maker_name_index();
                self.txt_print(MAKER_NAMES[who]);
                self.txt_print(BANNER_PRODUCT);
                self.txt_print(BANNER_COPYRIGHT);
            }
            Announce::LoadFailed => self.txt_print(LOAD_FAILED),
            Announce::Nothing => {}
        }
        self.start_target(hl, c)
    }

    /// ROM `0712`: which entry of [`MAKER_NAMES`] the links on PPI port B select.
    /// (The ROM skips `index + 1` strings, so entry 0 – "Arnold" – is never printed.)
    fn maker_name_index(&mut self) -> usize {
        let b = self.hw.inp(PPI_B);
        ((!b & 0x0E) >> 1) as usize + 1
    }

    /// ROM `06EB`: print a string through `TXT OUTPUT`.
    pub(crate) fn txt_print(&mut self, s: &str) {
        for ch in s.chars() {
            let b = match ch { '\u{a4}' => 0xA4, c => c as u32 as u8 };
            self.txt_output(b);
        }
    }

    /// ROM `0044`-`005B`: make RAM `0000-003F` a copy of the ROM's restart table, put
    /// `RST 0` at `0030` (default `RST 6` user restart) and copy the 0x1E9-byte kernel to
    /// `B900`.  (The ROM then falls into `KL CHOKE OFF`, which wipes `B100-B1BF` again.)
    fn install_vectors(&mut self) {
        for i in 0..0x40usize { self.ram[i] = self.rom[i]; }
        self.ram[0x30] = 0xC7;
        for i in 0..0x1E9usize { self.ram[0xB900 + i] = self.rom[0x391 + i]; }
        let _ = self.kl_choke_off();
    }

    /// ROM `0077`: pick the foreground program and record it.  Returns the values the ROM
    /// loads before its `RST 3` (far call): `HL = ABFF` (highest free byte), `DE = 0040`
    /// (lowest), `BC = B0FF` (highest usable by BASIC).
    fn start_target(&mut self, hl: u16, c: u8) -> StartTarget {
        let (entry, rom) = if hl == 0 { (0xC006, 0) } else { (hl, c) };
        self.wr8(KL_CURR_ROM, rom);
        self.wr8(KL_ENTRY_ROM, rom);
        self.wr16(KL_ENTRY_ADDR, entry);
        StartTarget { entry, rom, hl: 0xABFF, de: 0x0040, bc: 0xB0FF }
    }

    /// `MC BOOT PROGRAM` (`BD13`, ROM `05DC`).
    ///
    /// Entry: `loader` – address of a routine that loads a program; it returns carry set
    /// (and the entry address in `HL`) on success.
    ///
    /// Resets the sound/keyboard/text/screen managers, runs the loader, and on success
    /// restarts into the new program; on failure restarts into the *previous* foreground
    /// program after printing "*** PROGRAM LOAD FAILED ***".
    pub fn mc_boot_program(&mut self, loader: u16) -> StartTarget {
        self.sound_reset();
        self.hw.out(0xF8FF, 0xFF);
        let (old_entry, old_rom, c) = self.kl_choke_off();
        self.km_reset();
        self.txt_reset();
        self.scr_reset();
        self.kl_u_rom_enable();
        let mut regs = Regs::default();
        self.hw.call(loader, &mut regs);
        let _ = c; // POP BC restores C = 0xFF (or 0)
        if regs.carry() {
            self.mc_start_program(regs.hl, c)
        } else {
            self.restart(Announce::LoadFailed, old_entry, old_rom)
        }
    }

    /// `MC START PROGRAM` (`BD16`, ROM `060B`): restart the machine and enter the
    /// program at `entry` in ROM `rom`. Nothing is printed.
    pub fn mc_start_program(&mut self, entry: u16, rom: u8) -> StartTarget {
        self.restart(Announce::Nothing, entry, rom)
    }

    // ===============================================================================
    // Screen hardware                                            (0776 .. 07E5)
    // ===============================================================================

    /// `MC SET MODE` (`BD1C`, ROM `0776`): put `mode` (0-2) in the gate array.
    pub fn mc_set_mode(&mut self, mode: u8) {
        if mode >= 3 { return; }
        let r = (self.banking.mode_rom_register & !GA_MODE_MASK) | mode;
        self.banking.mode_rom_register = r;
        self.hw.out(GATE_ARRAY, r);
    }

    /// `MC WAIT FLYBACK` (`BD19`, ROM `07BA`): busy-wait for the start of vertical sync.
    pub fn mc_wait_flyback(&mut self) {
        while self.hw.inp(PPI_B) & 1 == 0 {}
    }

    /// `MC SCREEN OFFSET` (`BD1F`, ROM `07C6`): program CRTC R12/R13 with the 16 K screen
    /// `base` (top two bits of the address, `0x00/0x40/0x80/0xC0`) and 11-bit `offset`.
    pub fn mc_screen_offset(&mut self, base: u8, offset: u16) {
        let h = (offset >> 8) as u8;
        let l = offset as u8;
        let r12 = ((base >> 2) & 0x30) | ((h >> 1) & 0x03);
        let r13 = ((h & 1) << 7) | (l >> 1);
        self.hw.out(CRTC_SELECT, 12);
        self.hw.out(CRTC_DATA, r12);
        self.hw.out(CRTC_SELECT, 13);
        self.hw.out(CRTC_DATA, r13);
    }

    /// ROM `07AB`: select gate-array pen `c` and give it hardware colour `colour`
    /// (only the low five bits are used).
    fn ga_set_colour(&mut self, pen: u8, colour: u8) {
        self.hw.out(GATE_ARRAY, pen);
        self.hw.out(GATE_ARRAY, (colour & 0x1F) | GA_COLOUR);
    }

    /// `MC CLEAR INKS` (`BD22`, ROM `0786`): `table[0]` is the border colour; **every** pen
    /// is set to `table[1]` (this is how all inks are blanked).
    pub fn mc_clear_inks(&mut self, table: &[u8]) {
        self.ga_set_colour(0x10, table[0]);
        for pen in 0..16 { self.ga_set_colour(pen, table[1]); }
    }

    /// `MC SET INKS` (`BD25`, ROM `0799`): `table[0]` = border, `table[1..=16]` = pens 0-15.
    pub fn mc_set_inks(&mut self, table: &[u8]) {
        self.ga_set_colour(0x10, table[0]);
        for pen in 0..16 { self.ga_set_colour(pen, table[1 + pen as usize]); }
    }

    // ===============================================================================
    // Printer                                                    (07E6 .. 0825)
    // ===============================================================================

    /// `MC RESET PRINTER` (`BD28`, ROM `07E6`): restore the `MC WAIT PRINTER` indirection.
    pub fn mc_reset_printer(&mut self) {
        self.wr8(Ind::MC_WAIT_PRINTER, 0xC3);
        self.wr16(Ind::MC_WAIT_PRINTER + 1, 0x07F8);
    }

    /// `MC PRINT CHAR` (`BD2B`, ROM `07F2`): send `ch` through the `MC WAIT PRINTER`
    /// vector. Exit: `true` if the character was accepted before the time-out.
    pub fn mc_print_char(&mut self, ch: u8) -> bool {
        match self.patched(Ind::MC_WAIT_PRINTER, 0x07F8) {
            None => self.mc_wait_printer_default(ch),
            Some(addr) => {
                let mut r = Regs::default();
                r.set_a(ch);
                self.hw.call(addr, &mut r);
                r.carry()
            }
        }
    }

    /// Default `MC WAIT PRINTER` (ROM `07F8`): poll the busy line up to 50 × 256 times.
    pub fn mc_wait_printer_default(&mut self, ch: u8) -> bool {
        for _ in 0..(0x32u32 * 256) {
            if !self.mc_busy_printer() {
                return self.mc_send_printer(ch);
            }
        }
        false
    }

    /// `MC BUSY PRINTER` (`BD2E`, ROM `081B`): PPI port B bit 6 – `true` = busy.
    pub fn mc_busy_printer(&mut self) -> bool { self.hw.inp(PPI_B) & 0x40 != 0 }

    /// `MC SEND PRINTER` (`BD31`, ROM `0807`): put the low 7 bits on the data lines and
    /// pulse the strobe (bit 7 of the port).  Always succeeds.
    pub fn mc_send_printer(&mut self, ch: u8) -> bool {
        let d = ch & 0x7F;
        self.hw.out(PRINTER, d);          // data, strobe low
        self.hw.out(PRINTER, d | 0x80);   // strobe high
        self.hw.out(PRINTER, d);          // strobe low again
        true
    }

    // ===============================================================================
    // Sound chip and keyboard matrix                             (0826 .. 087F)
    // ===============================================================================

    /// `MC SOUND REGISTER` (`BD34`, ROM `0826`): write `value` to AY-3-8912 register `reg`
    /// through the PPI (port A = data bus, port C bits 7/6 = BDIR/BC1).
    pub fn mc_sound_register(&mut self, reg: u8, value: u8) {
        self.hw.out(PPI_A, reg);                     // register number on the data bus
        let c = self.hw.inp(PPI_C);
        self.hw.out(PPI_C, c | 0xC0);                // BDIR=1, BC1=1: latch address
        let idle = (c | 0xC0) & 0x3F;
        self.hw.out(PPI_C, idle);                    // inactive
        self.hw.out(PPI_A, value);                   // value on the data bus
        self.hw.out(PPI_C, idle | 0x80);             // BDIR=1, BC1=0: write
        self.hw.out(PPI_C, idle);                    // inactive
    }

    /// ROM `0846`: read the 10 rows of the keyboard matrix.
    ///
    /// For each row the raw port-A reading is stored in `raw[row]`; `held[row]` receives
    /// the *complement* of `old & new`, i.e. a 1 for every key that is down now **or** was
    /// down on the previous scan (cheap de-bouncing; keys are active-low in `raw`).
    pub fn scan_keyboard_matrix(&mut self, raw: &mut [u8; 10], held: &mut [u8; 10]) {
        // The matrix is wired to PSG register 14 (I/O port A): select it first.
        self.hw.out(PPI_A, 0x0E);
        let c = self.hw.inp(PPI_C) & 0x30;           // keep cassette motor / write bits
        self.hw.out(PPI_C, c | 0xC0);
        self.hw.out(PPI_C, c);
        self.hw.out(PPI_CTL, 0x92);                  // port A becomes an input
        for row in 0..10u8 {
            self.hw.out(PPI_C, c | 0x40 | row);      // BC1=1 (read PSG), row select
            let now = self.hw.inp(PPI_A);
            let old = raw[row as usize];
            raw[row as usize] = now;
            held[row as usize] = !(now & old);
        }
        self.hw.out(PPI_CTL, 0x82);                  // port A back to output
        self.hw.out(PPI_C, c);
    }
}
