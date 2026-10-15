//! Cassette manager: the signal level is compared with the ROM edge by edge.
//!
//! * the ROM and the Rust port both write the same block; the two recorded edge lists
//!   (timestamps in Z80 T-states) must agree;
//! * what the ROM recorded is played back into the Rust reader, and what Rust recorded
//!   into the ROM's reader (`CAS READ` / `CAS CHECK`).
mod support;
use cpc464_os::Regs;
use support::dual::*;

fn r(af: u16, bc: u16, de: u16, hl: u16) -> Regs { Regs { af, bc, de, hl, ..Default::default() } }
fn a(v: u8) -> u16 { (v as u16) << 8 }

/// A booted pair with the cassette motor already running (so the 2 s spin-up wait, which
/// pumps interrupts, is skipped).
fn setup() -> Option<Dual> {
    let rom = rom_path()?;
    let mut d = Dual::boot(&rom);
    d.rom(0x2370, r(0, 0, 0, 0)); d.cpc.cas_initialise();
    d.m.io.ppi_c |= 0x10; d.m.io.motor = true;
    d.cpc.hw.io.ppi_c |= 0x10; d.cpc.hw.io.motor = true;
    d.assert_same("after CAS INITIALISE");
    Some(d)
}

fn pattern(len: usize, seed: u8) -> Vec<u8> { (0..len).map(|i| (i as u8).wrapping_mul(37).wrapping_add(seed) ^ (i >> 3) as u8).collect() }

fn edges_relative(e: &[(u64, bool)]) -> Vec<(u64, bool)> {
    let t0 = e.first().map(|x| x.0).unwrap_or(0);
    e.iter().map(|&(t, l)| (t - t0, l)).collect()
}

#[test]
fn written_blocks_have_the_same_edges() {
    let Some(mut d) = setup() else { eprintln!("OS464_ROM not set; skipping"); return };
    for (len, marker) in [(1usize, 0x2Cu8), (5, 0x16), (64, 0x2C), (300, 0x16)] {
        let data = pattern(len, len as u8);
        d.poke_bytes(0x9000, &data);
        d.m.io.tape_out.clear(); d.cpc.hw.io.tape_out.clear();
        let x = d.rom(0x283F, r(a(marker), 0, len as u16, 0x9000));
        let y = d.cpc.cas_write(0x9000, len as u16, marker);
        assert!(x.carry() && y.is_ok(), "len {len}: ROM carry {} / Rust {:?}", x.carry(), y);
        let (re, ru) = (edges_relative(&d.m.io.tape_out), edges_relative(&d.cpc.hw.io.tape_out));
        assert_eq!(re.len(), ru.len(), "len {len}: number of edges (ROM {} vs Rust {})", re.len(), ru.len());
        if std::env::var("CASDUMP").is_ok() {
            let dl = |e: &Vec<(u64, bool)>| e.windows(2).map(|w| w[1].0 - w[0].0).collect::<Vec<_>>();
            let (dr, du) = (dl(&re), dl(&ru));
            eprintln!("ROM  first deltas {:?}", &dr[..60.min(dr.len())]);
            eprintln!("RUST first deltas {:?}", &du[..60.min(du.len())]);
            let mut sr = dr.clone(); sr.sort(); sr.dedup(); let mut su = du.clone(); su.sort(); su.dedup();
            eprintln!("ROM  distinct {:?}\nRUST distinct {:?}", sr, su);
        }
        // The ROM times its pulses with the Z80 R register and path-dependent loops; the Rust
        // port approximates them with fixed delays.  Levels and edge count must match exactly;
        // each individual pulse width may differ by a bounded amount (well inside what the
        // reader tolerates - see the round-trip tests).
        let mut worst = 0i64;
        for (i, (p, q)) in re.iter().zip(&ru).enumerate() {
            assert_eq!(p.1, q.1, "len {len}: level of edge {i}");
        }
        for (w, v) in re.windows(2).zip(ru.windows(2)) {
            let dt = (w[1].0 - w[0].0) as i64 - (v[1].0 - v[0].0) as i64;
            worst = worst.max(dt.abs());
        }
        eprintln!("len {len}: {} edges, worst single-pulse deviation {worst} T-states (approximate timing)", re.len());
        assert!(worst <= 250, "len {len}: pulse width deviates by {worst} T-states");
        d.ignore_rom_banking_writes();
        d.assert_same(&format!("after writing {len} bytes"));
    }
}

/// Feed a recorded edge list to a machine's tape input, starting `after` T-states from `now`.
fn play(edges: &[(u64, bool)], now: u64, after: u64) -> Vec<(u64, bool)> {
    let t0 = edges.first().map(|x| x.0).unwrap_or(0);
    edges.iter().map(|&(t, l)| (now + after + (t - t0), l)).collect()
}

#[test]
fn rom_written_tape_is_read_by_rust() {
    let Some(mut d) = setup() else { eprintln!("OS464_ROM not set; skipping"); return };
    for (len, marker) in [(1usize, 0x2Cu8), (7, 0x16), (64, 0x2C), (300, 0x16)] {
        let data = pattern(len, 9 + len as u8);
        d.poke_bytes(0x9000, &data);
        d.m.io.tape_out.clear();
        let x = d.rom(0x283F, r(a(marker), 0, len as u16, 0x9000));
        assert!(x.carry(), "ROM write failed");
        let tape = d.m.io.tape_out.clone();
        // play it into the Rust reader
        let now = d.cpc.hw.clock;
        d.cpc.hw.io.tape_in = play(&tape, now, 50_000);
        let before: Vec<u8> = (0..len).map(|i| d.cpc.rd8(0x9400 + i as u16)).collect();
        let res = d.cpc.cas_read(0x9400, len as u16, marker);
        let got: Vec<u8> = (0..len).map(|i| d.cpc.rd8(0x9400 + i as u16)).collect();
        eprintln!("len {len}: Rust read result {res:?}, data ok = {}", got == data);
        assert!(res.is_ok(), "len {len}: Rust failed to read the ROM's tape: {res:?} (memory before: {:?})", &before[..before.len().min(4)]);
        assert_eq!(got, data, "len {len}: data read back");
        d.cpc.hw.io.tape_in.clear();
        // The Rust side has now consumed interrupts/time the ROM side hasn't: re-sync is not
        // needed because only the cassette variables are compared below.
    }
}

#[test]
fn rust_written_tape_is_read_by_rom() {
    let Some(mut d) = setup() else { eprintln!("OS464_ROM not set; skipping"); return };
    for (len, marker) in [(1usize, 0x2Cu8), (7, 0x16), (64, 0x2C), (300, 0x16)] {
        let data = pattern(len, 77 + len as u8);
        d.poke_bytes(0x9000, &data);
        d.cpc.hw.io.tape_out.clear();
        let y = d.cpc.cas_write(0x9000, len as u16, marker);
        assert!(y.is_ok(), "Rust write failed: {y:?}");
        let tape = d.cpc.hw.io.tape_out.clone();
        let now = d.cpu.tstates;
        d.m.io.tape_in = play(&tape, now, 50_000);
        let x = d.rom(0x2836, r(a(marker), 0, len as u16, 0x9400));
        let got: Vec<u8> = (0..len).map(|i| d.peek(0x9400 + i as u16)).collect();
        eprintln!("len {len}: ROM read carry={} A={:02X}, data ok = {}", x.carry(), x.a(), got == data);
        assert!(x.carry(), "len {len}: ROM failed to read Rust's tape (error {})", x.a());
        assert_eq!(got, data, "len {len}: data read back");
        d.m.io.tape_in.clear();
    }
}
