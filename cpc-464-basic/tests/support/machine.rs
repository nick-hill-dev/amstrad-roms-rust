//! A CPC-like machine around the [`Z80`](super::z80::Z80): 64 KiB RAM, the lower ROM, and
//! just enough gate array / PPI / PSG to run the firmware.
#![allow(dead_code)]
use super::z80::{Bus, Z80};
use cpc464_os::Hardware;

/// Everything behind the I/O ports.  Shared between the emulated ROM run and the Rust
/// port's [`TestHw`], so both see identical hardware behaviour.
#[derive(Clone)]
pub struct Peripherals {
    /// Every `OUT` as `(full 16-bit port, value)`.
    pub log: Vec<(u16, u8)>,
    /// Lower/upper ROM enables as set by the gate array (`true` = ROM visible).
    pub lower_rom: bool,
    pub upper_rom: bool,
    pub upper_rom_number: u8,
    pub mode: u8,
    /// PPI port B input (bit 4 = 50 Hz, bits 1-3 = manufacturer links, bit 7 = tape in).
    pub ppi_b: u8,
    pub ppi_a_out: u8,
    pub ppi_a_in: u8,
    pub ppi_c: u8,
    pub ppi_ctl: u8,
    pub psg: [u8; 16],
    pub psg_addr: u8,
    /// Keyboard matrix, 10 rows, `1` bit = key up (active low like the real thing).
    pub keys: [u8; 10],
    /// Cassette: level of the output bit over time `(tstate, level)`.
    pub tape_out: Vec<(u64, bool)>,
    /// Cassette input as a function of time.
    pub tape_in: Vec<(u64, bool)>,
    /// Motor on?
    pub motor: bool,
    /// Force the CRTC VSYNC input (PPI B bit 0); `None` = derived from the clock.
    pub vsync_override: Option<bool>,
}

impl Default for Peripherals {
    fn default() -> Self {
        Peripherals {
            log: vec![], lower_rom: true, upper_rom: false, upper_rom_number: 0, mode: 1,
            ppi_b: 0x5E, ppi_a_out: 0, ppi_a_in: 0xFF, ppi_c: 0, ppi_ctl: 0x82,
            psg: [0; 16], psg_addr: 0, keys: [0xFF; 10], tape_out: vec![], tape_in: vec![], motor: false, vsync_override: None,
        }
    }
}

impl Peripherals {
    pub fn out(&mut self, port: u16, v: u8, tstates: u64) {
        self.log.push((port, v));
        if port & 0x8000 == 0 && port & 0x4000 != 0 {
            // Gate array
            if v >> 6 == 2 {
                self.mode = v & 3;
                self.lower_rom = v & 4 == 0;
                self.upper_rom = v & 8 == 0;
            }
        }
        if port & 0x2000 == 0 { self.upper_rom_number = v; }
        if port & 0x0800 == 0 {
            match (port >> 8) & 3 {
                0 => self.ppi_a_out = v,
                1 => {}
                2 => { self.set_port_c(v, tstates); }
                _ => {
                    if v & 0x80 != 0 {
                        self.ppi_ctl = v;
                        self.ppi_a_out = 0;
                        self.set_port_c(0, tstates);
                    } else {
                        let bit = (v >> 1) & 7;
                        let mut c = self.ppi_c;
                        if v & 1 != 0 { c |= 1 << bit } else { c &= !(1 << bit) }
                        self.set_port_c(c, tstates);
                    }
                }
            }
        }
    }

    fn set_port_c(&mut self, c: u8, tstates: u64) {
        let old = self.ppi_c;
        self.ppi_c = c;
        if (old ^ c) & 0x20 != 0 { self.tape_out.push((tstates, c & 0x20 != 0)); }
        self.motor = c & 0x10 != 0;
        if (old ^ c) & 0xC0 != 0 || c & 0xC0 != 0 {
            match c >> 6 {
                3 => self.psg_addr = self.ppi_a_out & 0x0F,
                2 => { let a = self.psg_addr as usize; self.psg[a] = self.ppi_a_out; }
                1 => {
                    let a = self.psg_addr as usize;
                    self.ppi_a_in = if a == 14 {
                        let row = (c & 0x0F) as usize;
                        if row < 10 { self.keys[row] } else { 0xFF }
                    } else { self.psg[a] };
                }
                _ => {}
            }
        }
    }

    pub fn inp(&mut self, port: u16, tstates: u64) -> u8 {
        if port & 0x0800 == 0 {
            match (port >> 8) & 3 {
                0 => if self.ppi_ctl & 0x10 != 0 { self.ppi_a_in } else { self.ppi_a_out },
                1 => {
                    let tape = self.tape_level(tstates);
                    // Bit 0 is the CRTC's VSYNC (about 50 Hz at 4 MHz); bit 7 the tape input.
                    let vsync = self.vsync_override.unwrap_or(tstates % 80_000 < 8_000);
                    (self.ppi_b & 0x7E) | vsync as u8 | if tape { 0x80 } else { 0 }
                }
                2 => self.ppi_c,
                _ => 0xFF,
            }
        } else {
            0xFF
        }
    }

    /// Tape input level at `t`: the last entry of `tape_in` not after `t` (initially low).
    pub fn tape_level(&self, t: u64) -> bool {
        match self.tape_in.binary_search_by(|e| e.0.cmp(&t)) {
            Ok(i) => self.tape_in[i].1,
            Err(0) => false,
            Err(i) => self.tape_in[i - 1].1,
        }
    }
}

/// The emulated machine used to run the original ROMs: the lower ROM, and BASIC as
/// every upper ROM number.
pub struct RomMachine {
    pub ram: Vec<u8>,
    pub rom: Vec<u8>,
    pub basic: Vec<u8>,
    pub io: Peripherals,
}

impl RomMachine {
    pub fn new(rom: &[u8], basic: &[u8]) -> Self {
        RomMachine { ram: vec![0; 0x10000], rom: rom.to_vec(), basic: basic.to_vec(), io: Peripherals::default() }
    }
}

impl Bus for RomMachine {
    fn rd(&mut self, a: u16) -> u8 {
        if a < 0x4000 && self.io.lower_rom { self.rom[a as usize] }
        else if a >= 0xC000 && self.io.upper_rom { self.basic[(a - 0xC000) as usize] }
        else { self.ram[a as usize] }
    }
    fn wr(&mut self, a: u16, v: u8) { self.ram[a as usize] = v; }
    fn input(&mut self, port: u16, t: u64) -> u8 { self.io.inp(port, t) }
    fn output(&mut self, port: u16, v: u8, t: u64) { self.io.out(port, v, t) }
}

/// [`Hardware`] for the Rust port with exactly the same peripherals.
#[derive(Clone)]
pub struct TestHw {
    pub io: Peripherals,
    /// Clock for the cassette timing (advanced by `delay` and by every `inp`).
    pub clock: u64,
    /// The BASIC ROM, seen as every upper ROM.
    pub basic: Vec<u8>,
    /// Every `Hardware::call` / `far_call` made (address, ROM select).
    pub calls: Vec<(u16, u8)>,
    /// A VSYNC level forced by the test (shared with the line-input hook).
    pub vsync: std::rc::Rc<std::cell::Cell<Option<bool>>>,
    /// A recorded tape to start playing (relative to the clock at the next input).
    pub tape_load: std::rc::Rc<std::cell::RefCell<Option<Vec<(u64, bool)>>>>,
}

impl TestHw {
    pub fn new(basic: &[u8]) -> Self {
        TestHw { io: Peripherals::default(), clock: 0, basic: basic.to_vec(), calls: vec![], vsync: Default::default(), tape_load: Default::default() }
    }
}

impl Hardware for TestHw {
    fn out(&mut self, port: u16, value: u8) { let c = self.clock; self.io.out(port, value, c); }
    fn inp(&mut self, port: u16) -> u8 {
        if let Some(t) = self.tape_load.borrow_mut().take() { self.io.tape_in = play(&t, self.clock, 50_000); }
        let c = self.clock;
        if let Some(v) = self.vsync.get() { self.io.vsync_override = Some(v); }
        let v = self.io.inp(port, c);
        if self.vsync.get().is_some() { self.io.vsync_override = None; }
        // One pass of the cassette polling loop is about 57 T-states.
        if port & 0x0800 == 0 && (port >> 8) & 3 == 1 { self.clock += 57; }
        v
    }
    fn delay(&mut self, t: u32) { self.clock += t as u64; }
    fn upper_rom_byte(&self, _rom: u8, offset: u16) -> u8 { self.basic[offset as usize & 0x3FFF] }
    fn call(&mut self, addr: u16, _regs: &mut cpc464_os::Regs) { self.calls.push((addr, 0)); }
    fn far_call(&mut self, addr: u16, rom: u8, _regs: &mut cpc464_os::Regs) { self.calls.push((addr, rom)); }
}

/// Run `cpu` until `pc == stop` (checked before each instruction) or `limit` instructions.
pub fn run_until(cpu: &mut Z80, m: &mut RomMachine, stop: u16, limit: u64) -> Result<(), String> {
    for _ in 0..limit {
        if cpu.pc == stop { return Ok(()); }
        cpu.step(m);
    }
    Err(format!("did not reach {stop:04X}; pc={:04X}", cpu.pc))
}

/// Render the top `rows` character rows of a mode-1 screen at `C000` as ASCII art
/// (`#` = pen 1..3, `.` = pen 0), 80 pixels × 8 lines per row, only the first 40 columns.
pub fn dump_mode1(ram: &[u8], rows: usize, cols: usize) -> String {
    let mut out = String::new();
    for row in 0..rows {
        for line in 0..8 {
            let base = 0xC000 + (line * 0x800) + row * 80;
            for col in 0..cols {
                let b = ram[base + col * 2..base + col * 2 + 2].to_vec();
                for byte in b {
                    for px in 0..4 {
                        // mode 1: bit (7-px) = pen bit 0, bit (3-px) = pen bit 1
                        let p = ((byte >> (7 - px)) & 1) | (((byte >> (3 - px)) & 1) << 1);
                        out.push(if p != 0 { '#' } else { '.' });
                    }
                }
            }
            out.push('\n');
        }
        out.push('\n');
    }
    out
}

/// A recorded edge list, played so that its first edge comes `after` T-states after `now`.
pub fn play(edges: &[(u64, bool)], now: u64, after: u64) -> Vec<(u64, bool)> {
    let t0 = edges.first().map(|x| x.0).unwrap_or(0);
    edges.iter().map(|&(t, l)| (now + after + (t - t0), l)).collect()
}
