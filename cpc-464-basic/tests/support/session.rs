//! Run the original ROMs (OS + BASIC, in the Z80 emulator) and the Rust port from power
//! on, feeding both the same typed lines, and compare the machines afterwards.
//!
//! The only input BASIC reads is the firmware line editor `EDIT` (`BD3A`).  On the ROM
//! side the emulator intercepts the call: the next line is written into the buffer at
//! `HL`, carry is set (ENTER) and the call returns.  The Rust side gets the same lines
//! through [`Basic::line_input`].  When the lines run out both sides stop at their next
//! `EDIT`.
#![allow(dead_code)]
use super::machine::*;
use super::z80::Z80;
use cpc464_basic::{Basic, LineInput};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

/// The firmware's `EDIT` entry.
const EDIT: u16 = 0xBD3A;

pub fn roms() -> Option<(Vec<u8>, Vec<u8>)> {
    let os = std::fs::read(std::env::var("OS464_ROM").ok()?).ok()?;
    let basic = std::fs::read(std::env::var("BASIC_ROM").ok()?).ok()?;
    Some((os, basic))
}

/// Both machines after a session.
pub struct Session {
    pub rom: RomMachine,
    pub rust: Basic<TestHw>,
    /// What each side's EDIT calls found already in the buffer (AUTO numbers, EDIT ...).
    pub rom_prompts: Vec<Vec<u8>>,
    pub rust_prompts: Vec<Rc<RefCell<Vec<Vec<u8>>>>>,
}

/// One input event: a typed line, or `#TICKS n` (n interrupts before the next line, with
/// the VSYNC input low so that no frame-flyback work is done).
fn parse(line: &str) -> LineInput {
    match line.strip_prefix("#TICKS ") {
        Some(n) => LineInput::Ticks(n.parse().unwrap()),
        None => LineInput::Line(line.as_bytes().to_vec()),
    }
}

/// Set-up of a session beyond the typed lines.
#[derive(Clone, Default)]
pub struct Options {
    /// The cassette motor is already running (so opening a file does not wait for it to
    /// spin up, which needs interrupts).
    pub motor: bool,
    /// The tape that the input `#TAPE` starts playing on both sides.
    pub tape: Vec<(u64, bool)>,
}

/// Power on both sides and type `lines`.
pub fn run(os: &[u8], basic: &[u8], lines: &[&str]) -> Session { run_with(os, basic, lines, &Options::default()) }

/// [`run`] with [`Options`].
pub fn run_with(os: &[u8], basic: &[u8], lines: &[&str], opts: &Options) -> Session {
    // ---- the ROMs ----
    let mut m = RomMachine::new(os, basic);
    if opts.motor { m.io.ppi_c |= 0x10; m.io.motor = true; }
    let mut cpu = Z80::new();
    let mut queue: VecDeque<LineInput> = lines.iter().map(|l| parse(l)).collect();
    let mut rom_prompts = vec![];
    let mut steps: u64 = 0;
    let mut covered = vec![false; 0x4000];
    loop {
        if cpu.pc == EDIT {
            let hl = cpu.hl();
            let mut cur = vec![];
            let mut a = hl;
            while m.ram[a as usize] != 0 { cur.push(m.ram[a as usize]); a = a.wrapping_add(1); }
            rom_prompts.push(cur);
            let mut next = queue.pop_front();
            while next == Some(LineInput::Line(b"#TAPE".to_vec())) {
                m.io.tape_in = play(&opts.tape, cpu.tstates, 50_000);
                next = queue.pop_front();
            }
            while let Some(LineInput::Ticks(n)) = next {
                for _ in 0..n {
                    m.io.vsync_override = Some(false);
                    let mark = m.io.log.len();
                    cpu.iff1 = true;
                    cpu.interrupt(&mut m);
                    while cpu.pc != EDIT { cpu.step(&mut m); }
                    // Interrupt entry/exit banking writes (the Rust side has none).
                    let mut keep = m.io.log[..mark].to_vec();
                    keep.extend(m.io.log[mark..].iter().filter(|(p, v)| !(p >> 8 == 0x7F && v & 0xC0 == 0x80)));
                    m.io.log = keep;
                }
                m.io.vsync_override = None;
                next = queue.pop_front();
            }
            let Some(LineInput::Line(line)) = next else { break };
            for (i, b) in line.iter().take(255).enumerate() { m.ram[hl.wrapping_add(i as u16) as usize] = *b; }
            m.ram[hl.wrapping_add(line.len().min(255) as u16) as usize] = 0;
            cpu.f |= 0x01;
            // RET
            let lo = m.ram[cpu.sp as usize] as u16;
            let hi = m.ram[cpu.sp.wrapping_add(1) as usize] as u16;
            cpu.sp = cpu.sp.wrapping_add(2);
            cpu.pc = hi << 8 | lo;
        }
        if cpu.pc >= 0xC000 && m.io.upper_rom { covered[(cpu.pc - 0xC000) as usize] = true; }
        cpu.step(&mut m);
        steps += 1;
        assert!(steps < 2_000_000_000, "ROM side runs away; pc={:04X}", cpu.pc);
    }
    // COVERAGE=file: append the BASIC ROM addresses executed (for finding untested code).
    if let Ok(path) = std::env::var("COVERAGE") {
        use std::io::Write;
        let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path).unwrap();
        let hits: Vec<String> = covered.iter().enumerate().filter(|(_, c)| **c).map(|(a, _)| format!("{:04X}", a + 0xC000)).collect();
        writeln!(f, "{}", hits.join(" ")).unwrap();
    }
    // ---- the port ----
    let mut rust = Basic::new(os, basic, TestHw::new(basic));
    if opts.motor { rust.cpc.hw.io.ppi_c |= 0x10; rust.cpc.hw.io.motor = true; }
    let tape_load = rust.cpc.hw.tape_load.clone();
    let tape = opts.tape.clone();
    let queue = Rc::new(RefCell::new(lines.iter().map(|l| parse(l)).collect::<VecDeque<_>>()));
    let prompts = Rc::new(RefCell::new(vec![]));
    let (q, p) = (queue.clone(), prompts.clone());
    let pending_ticks = Rc::new(RefCell::new(false));
    let vsync = rust.cpc.hw.vsync.clone();
    rust.line_input = Some(Box::new(move |cur: &[u8]| {
        vsync.set(None);
        // A Ticks answer makes the editor ask again: record the buffer only once.
        if !std::mem::replace(&mut *pending_ticks.borrow_mut(), false) { p.borrow_mut().push(cur.to_vec()); }
        let mut q = q.borrow_mut();
        let mut next = q.pop_front();
        while next == Some(LineInput::Line(b"#TAPE".to_vec())) {
            *tape_load.borrow_mut() = Some(tape.clone());
            next = q.pop_front();
        }
        match next {
            Some(LineInput::Ticks(n)) => { *pending_ticks.borrow_mut() = true; vsync.set(Some(false)); LineInput::Ticks(n) }
            Some(l) => l,
            None => LineInput::Stop,
        }
    }));
    rust.power_on_and_run();
    Session { rom: m, rust, rom_prompts, rust_prompts: vec![prompts] }
}

/// RAM that is not comparable: the Z80 stack page, the firmware's interrupt stack, and
/// the operand of the self-modifying `CALL` in the RAM kernel's ROM-switching jump
/// (`BA3E`), which holds the target of the last `RST` call through it.  Also `B55B`:
/// `SOUND RESET` initialises the sound event block without loading `C`, so its ROM
/// select byte is whatever the caller had in `C` (a firmware quirk; the Rust firmware
/// stores 0).
fn ignored(a: usize) -> bool {
    (0xBF00..0xC000).contains(&a) || (0xB105..0xB187).contains(&a) || (0xBA3F..0xBA41).contains(&a)
        || a == 0xB55B
}

impl Session {
    /// The screen as text (mode 1, first `rows` rows): not checked, for diagnostics.
    pub fn dump(&self, rows: usize) -> (String, String) {
        (dump_mode1(&self.rom.ram, rows, 40), dump_mode1(&self.rust.cpc.ram[..], rows, 40))
    }

    /// Compare RAM and the hardware writes (gate-array ROM banking and ROM selection,
    /// which only the ROM side performs, are left out).
    pub fn assert_same(&self, what: &str) { self.assert_same_except(what, &[], true) }

    /// [`Self::assert_same`], also ignoring the RAM ranges `skip`, and the hardware writes
    /// unless `io`.
    pub fn assert_same_except(&self, what: &str, skip: &[std::ops::Range<usize>], io: bool) {
        let rp = self.rust_prompts[0].borrow();
        assert_eq!(&self.rom_prompts, &*rp, "{what}: the EDIT buffers differ");
        let mut diffs = vec![];
        for a in 0..0x10000usize {
            if ignored(a) || skip.iter().any(|r| r.contains(&a)) { continue; }
            if self.rom.ram[a] != self.rust.cpc.ram[a] { diffs.push(a); }
        }
        if !diffs.is_empty() {
            let shown: Vec<String> = diffs.iter().take(32)
                .map(|a| format!("  {a:04X}: rom={:02X} rust={:02X}", self.rom.ram[*a], self.rust.cpc.ram[*a])).collect();
            if std::env::var("DUMP").is_ok() {
                let (a, b) = self.dump(10);
                eprintln!("ROM screen:\n{a}\nRUST screen:\n{b}");
            }
            panic!("{what}: {} RAM bytes differ\n{}", diffs.len(), shown.join("\n"));
        }
        if !io { return; }
        let norm = |l: &[(u16, u8)]| l.iter()
            .filter(|(p, v)| !(p >> 8 == 0x7F && v & 0xC0 == 0x80) && p & 0x2000 != 0)
            .map(|(p, v)| (p >> 8, *v)).collect::<Vec<_>>();
        let a = norm(&self.rom.io.log);
        let b = norm(&self.rust.cpc.hw.io.log);
        let first = a.iter().zip(&b).position(|(x, y)| x != y);
        assert!(first.is_none() && a.len() == b.len(),
            "{what}: hardware writes differ at {:?} (rom {} writes, rust {})", first, a.len(), b.len());
    }
}

/// Run a session and compare.
pub fn check(lines: &[&str]) {
    let Some((os, basic)) = roms() else { eprintln!("OS464_ROM / BASIC_ROM not set; skipping"); return };
    let s = run(&os, &basic, lines);
    s.assert_same(&format!("{lines:?}"));
}
