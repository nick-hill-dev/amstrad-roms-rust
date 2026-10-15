//! Differential tests of the sound manager against the original ROM.
mod support;
use cpc464_os::Regs;
use support::dual::*;

const SOUND_QUEUE: u16 = 0x1F9F;

fn queue(d: &mut Dual, block: [u8; 9], what: &str) {
    d.poke_bytes(0x9000, &block);
    let r = d.rom(SOUND_QUEUE, Regs { hl: 0x9000, ..Default::default() });
    let ok = d.cpc.sound_queue(&block);
    assert_eq!(ok, r.af & 1 != 0, "{what}: carry");
    d.assert_same(what);
}

#[test]
fn queue_bell() {
    let Some(rom) = rom_path() else { return };
    let mut d = Dual::boot(&rom);
    queue(&mut d, [0x87, 0, 0, 0x5A, 0, 0, 0x0B, 0x14, 0], "bell");
}

/// Random mixture of sound-manager calls and interrupts, ROM versus Rust.
#[test]
fn sound_fuzz() {
    let Some(rom) = rom_path() else { return };
    let seeds: u64 = std::env::var("SOUND_SEEDS").ok().and_then(|s| s.parse().ok()).unwrap_or(6);
    for seed in 0..seeds {
        let mut d = Dual::boot(&rom);
        let mut rng = Rng(0x5EED_0000 + seed * 7919);
        let mut trace: Vec<String> = vec![];
        for step in 0..400 {
            let what;
            match rng.below(14) {
                0 | 1 | 2 | 3 => {
                    // queue a note
                    let mut b = [0u8; 9];
                    for x in b.iter_mut() { *x = rng.byte(); }
                    // keep notes short and mostly sensible
                    b[0] = (b[0] & 0x7F) | if rng.below(8) == 0 { 0x80 } else { 0 };
                    if rng.below(2) == 0 { b[0] = (b[0] & !0x38) | 0; }       // no rendezvous
                    if rng.below(2) == 0 { b[1] = 0; }                         // default envelopes
                    b[6] &= 0x0F;
                    b[7] = b[7] % 40 + 1; b[8] = 0;
                    what = format!("queue {b:02X?}");
                    queue(&mut d, b, &format!("{what} (step {step}, seed {seed})"));
                }
                4 => {
                    let n = (rng.below(17)) as u8;
                    let mut data = [0u8; 16];
                    // plausible envelope: section count 1..5, then (steps,size,pause) triples
                    data[0] = 1 + rng.below(4) as u8;
                    for k in 1..16 { data[k] = rng.byte(); }
                    for k in (1..15).step_by(3) { data[k] = rng.below(6) as u8 | if rng.below(8) == 0 { 0x80 } else { 0 }; data[(k + 2).min(15)] = 1 + (rng.byte() % 10); }
                    what = format!("ampl_env {n} {data:02X?}");
                    d.poke_bytes(0x9100, &data);
                    let r = d.rom(0x2338, Regs { af: (n as u16) << 8, hl: 0x9100, ..Default::default() });
                    let ok = d.cpc.sound_ampl_envelope(n, &data);
                    assert_eq!(ok, r.af & 1 != 0, "{what}: carry");
                    d.assert_same(&what);
                }
                5 => {
                    let n = (rng.below(17)) as u8;
                    let mut data = [0u8; 16];
                    for k in 0..16 { data[k] = rng.byte(); }
                    data[0] = 1 + rng.below(4) as u8;
                    what = format!("tone_env {n} {data:02X?}");
                    d.poke_bytes(0x9100, &data);
                    let r = d.rom(0x233D, Regs { af: (n as u16) << 8, hl: 0x9100, ..Default::default() });
                    let ok = d.cpc.sound_tone_envelope(n, &data);
                    assert_eq!(ok, r.af & 1 != 0, "{what}: carry");
                    d.assert_same(&what);
                }
                6 => {
                    what = "hold".to_string();
                    let r = d.rom(0x1ECB, Regs::default());
                    let ok = d.cpc.sound_hold();
                    assert_eq!(ok, r.af & 1 != 0, "{what}: carry");
                    d.assert_same(&what);
                }
                7 => {
                    what = "continue".to_string();
                    d.rom(0x1EE6, Regs::default());
                    d.cpc.sound_continue();
                    d.assert_same(&what);
                }
                8 => {
                    let m = rng.byte() & 7;
                    what = format!("release {m}");
                    d.rom(0x204A, Regs { af: (m as u16) << 8, ..Default::default() });
                    d.cpc.sound_release(m);
                    d.assert_same(&what);
                }
                9 => {
                    let m = rng.byte() & 7;
                    what = format!("check {m}");
                    let r = d.rom(0x206C, Regs { af: (m as u16) << 8, ..Default::default() });
                    let a = d.cpc.sound_check(m);
                    assert_eq!(a, (r.af >> 8) as u8, "{what}: result");
                    d.assert_same(&what);
                }
                _ => {
                    let n = 1 + rng.below(4);
                    what = format!("{n} interrupts");
                    for k in 0..n {
                        let vs = rng.below(5) == 0;
                        d.interrupt_both(vs);
                        d.assert_same(&format!("interrupt {k}/{n} (step {step}, seed {seed}; recent: {:?})", trace.iter().rev().take(4).collect::<Vec<_>>()));
                    }
                }
            }
            trace.push(what);
        }
        eprintln!("seed {seed}: {} hardware writes compared", d.m.io.log.len());
    }
}
