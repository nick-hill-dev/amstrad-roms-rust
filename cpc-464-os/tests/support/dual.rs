//! Run the original ROM (in the emulator) and the Rust port side by side from the same
//! post-boot state, and compare them.
#![allow(dead_code)]
use super::machine::*;
use super::z80::Z80;
use cpc464_os::{Cpc, Regs};

/// Address the emulated routine "returns" to (a `RST 0` in the ROM that is never executed).
pub const RETURN_TO: u16 = 0x0098;

pub struct Dual {
    pub cpu: Z80,
    pub m: RomMachine,
    pub cpc: Cpc<TestHw>,
    /// How many hardware writes of each side were already compared.
    log_mark: (usize, usize),
}

pub fn rom_path() -> Option<Vec<u8>> {
    let path = std::env::var("OS464_ROM").ok()?;
    std::fs::read(path).ok()
}

impl Dual {
    /// Boot both sides.
    pub fn boot(rom: &[u8]) -> Dual {
        let mut m = RomMachine::new(rom);
        let mut cpu = Z80::new();
        run_until(&mut cpu, &mut m, 0x0095, 50_000_000).expect("ROM boot");
        let mut cpc = Cpc::new(rom, TestHw::new());
        cpc.power_on();
        let marks = (m.io.log.len(), cpc.hw.io.log.len());
        let mut d = Dual { cpu, m, cpc, log_mark: marks };
        d.assert_same("after boot");
        d
    }

    /// Write a byte on both sides.
    pub fn poke(&mut self, a: u16, v: u8) {
        self.m.ram[a as usize] = v;
        self.cpc.wr8(a, v);
    }
    pub fn poke16(&mut self, a: u16, v: u16) { self.poke(a, v as u8); self.poke(a.wrapping_add(1), (v >> 8) as u8); }
    pub fn poke_bytes(&mut self, a: u16, bytes: &[u8]) { for (i, b) in bytes.iter().enumerate() { self.poke(a.wrapping_add(i as u16), *b); } }
    pub fn peek(&self, a: u16) -> u8 { self.m.ram[a as usize] }

    /// Call the ROM routine at `addr` with the given registers; returns the registers on
    /// return.  The 16-bit `ix`/`iy` are also set.
    pub fn rom(&mut self, addr: u16, regs: Regs) -> Regs {
        let c = &mut self.cpu;
        c.set_af(regs.af); c.set_bc(regs.bc); c.set_de(regs.de); c.set_hl(regs.hl);
        c.ix = regs.ix; c.iy = regs.iy;
        c.sp = 0xC000 - 2;
        self.m.ram[0xBFFE] = RETURN_TO as u8;
        self.m.ram[0xBFFF] = (RETURN_TO >> 8) as u8;
        c.pc = addr;
        run_until_return(c, &mut self.m);
        Regs { af: c.af(), bc: c.bc(), de: c.de(), hl: c.hl(), ix: c.ix, iy: c.iy }
    }

    /// Deliver one maskable interrupt to the ROM and run its handler to completion.
    pub fn rom_interrupt(&mut self) {
        let c = &mut self.cpu;
        c.sp = 0xC000 - 2;
        c.pc = RETURN_TO;
        c.iff1 = true;
        c.interrupt(&mut self.m);
        run_until_return(c, &mut self.m);
    }

    /// Deliver one interrupt to both implementations with the given VSYNC level.
    pub fn interrupt_both(&mut self, vsync: bool) {
        self.m.io.vsync_override = Some(vsync);
        self.cpc.hw.io.vsync_override = Some(vsync);
        let mark = self.m.io.log.len();
        self.rom_interrupt();
        // The ROM's interrupt entry/exit toggles the lower-ROM enable in the gate array
        // (`OUT 7F8x`); the Rust port has no such banking, so ignore those writes.
        let mut keep = self.m.io.log[..mark].to_vec();
        keep.extend(self.m.io.log[mark..].iter().filter(|(p, v)| !(p >> 8 == 0x7F && v & 0xC0 == 0x80)));
        self.m.io.log = keep;
        self.cpc.interrupt();
        self.m.io.vsync_override = None;
        self.cpc.hw.io.vsync_override = None;
    }

    /// Drop the ROM side's gate-array mode/ROM writes made since the last comparison.
    /// The firmware switches the lower ROM off to reach RAM underneath it (`KL LDIR` and
    /// friends); the Rust port reads that RAM directly, so only the ROM emits these.
    pub fn ignore_rom_banking_writes(&mut self) {
        let mark = self.log_mark.0;
        let mut keep = self.m.io.log[..mark].to_vec();
        let mut drop_next = false;
        for &(p, v) in &self.m.io.log[mark..] {
            let gate = p >> 8 == 0x7F && v & 0xC0 == 0x80;
            if gate && drop_next { drop_next = false; continue; }          // the re-enable
            if gate && v & 0x04 != 0 { drop_next = true; continue; }       // lower ROM switched off
            keep.push((p, v));
        }
        self.m.io.log = keep;
    }

    /// Compare all RAM (except the stack page) and the hardware writes since the last call.
    pub fn assert_same(&mut self, what: &str) {
        let mut diffs = vec![];
        for a in 0..0x10000usize {
            // 0xBF00.. is the caller's stack; 0xB105..0xB187 is the ROM's private interrupt stack
            // and its saved SP (implementation residue, not firmware state).
            if (0xBF00..0xC000).contains(&a) || (0xB105..0xB187).contains(&a) { continue; }
            if self.m.ram[a] != self.cpc.ram[a] { diffs.push(a); }
        }
        if !diffs.is_empty() {
            let shown: Vec<String> = diffs.iter().take(24)
                .map(|a| format!("  {a:04X}: rom={:02X} rust={:02X}", self.m.ram[*a], self.cpc.ram[*a])).collect();
            panic!("{what}: {} RAM bytes differ\n{}", diffs.len(), shown.join("\n"));
        }
        let norm = |l: &[(u16, u8)]| l.iter().map(|(p, v)| (p >> 8, *v)).collect::<Vec<_>>();
        let a = norm(&self.m.io.log[self.log_mark.0..]);
        let b = norm(&self.cpc.hw.io.log[self.log_mark.1..]);
        self.log_mark = (self.m.io.log.len(), self.cpc.hw.io.log.len());
        assert_eq!(a, b, "{what}: hardware writes differ (rom vs rust)");
    }
}

fn run_until_return(c: &mut Z80, m: &mut RomMachine) {
    for _ in 0..200_000_000u64 {
        if c.pc == RETURN_TO { return; }
        c.step(m);
    }
    panic!("ROM routine did not return; pc={:04X}", c.pc);
}

/// A tiny deterministic PRNG so the "random" tests are reproducible.
pub struct Rng(pub u64);
impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13; self.0 ^= self.0 >> 7; self.0 ^= self.0 << 17; self.0
    }
    pub fn byte(&mut self) -> u8 { (self.next() >> 24) as u8 }
    pub fn word(&mut self) -> u16 { (self.next() >> 20) as u16 }
    pub fn below(&mut self, n: u32) -> u32 { ((self.next() >> 16) % n as u64) as u32 }
}
