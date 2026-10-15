//! Differential test of the interrupt machinery, keyboard scanning, key/character
//! queues, expansion strings, tickers and the clock, driven by a randomly pressed
//! keyboard matrix.
mod support;
use cpc464_os::Regs;
use support::dual::*;

fn n() -> usize { std::env::var("INTERRUPTS_N").ok().and_then(|s| s.parse().ok()).unwrap_or(1500) }
fn r(af: u16, bc: u16, de: u16, hl: u16) -> Regs { Regs { af, bc, de, hl, ..Default::default() } }

#[test]
fn keyboard_and_interrupts_match_rom() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0x1EE7_0000_0000_0003);
    let mut tick = 0u32;
    let mut chars = 0u32;
    for step in 0..n() {
        // ---- change the keys held down ----------------------------------------------
        if rng.below(3) == 0 {
            let row = rng.below(10) as usize;
            let bit = 1u8 << rng.below(8);
            let mut keys = d.m.io.keys;
            if rng.below(2) == 0 { keys[row] ^= bit; } else if rng.below(6) == 0 { keys = [0xFF; 10]; }
            // never let CONTROL+SHIFT+ESC reset the machine
            if keys[8] & 0x04 == 0 && keys[2] & 0x80 == 0 && keys[2] & 0x20 == 0 { keys[2] |= 0x80; }
            d.m.io.keys = keys;
            d.cpc.hw.io.keys = keys;
        }
        // ---- time passes -------------------------------------------------------------
        let what;
        match rng.below(14) {
            0 => {
                what = "read_char".to_string();
                let x = d.rom(0x1A42, r(0, 0, 0, 0));
                let c = d.cpc.km_read_char();
                assert_eq!(x.carry(), c.is_some(), "{what}: found");
                if let Some(c) = c { chars += 1; assert_eq!(x.a(), c, "{what}: char"); }
            }
            1 => {
                what = "read_key".to_string();
                let x = d.rom(0x1B5C, r(0, 0, 0, 0));
                let c = d.cpc.km_read_key();
                assert_eq!(x.carry(), c.is_some(), "{what}: found");
                if let Some(c) = c { chars += 1; assert_eq!(x.a(), c, "{what}: key"); }
            }
            2 => {
                let key = rng.byte() % 80;
                what = format!("test_key {key}");
                let x = d.rom(0x1CBD, r((key as u16) << 8, 0, 0, 0));
                let (down, mods) = d.cpc.km_test_key(key);
                assert_eq!((x.af & 0x40 == 0, x.bc as u8), (down, mods), "{what}");
            }
            3 => {
                what = "get_state / joystick".to_string();
                let x = d.rom(0x1BB3, r(0, 0, 0, 0));
                assert_eq!(((x.hl as u8), (x.hl >> 8) as u8), d.cpc.km_get_state(), "{what}");
                let j = d.rom(0x1C5C, r(0, 0, 0, 0));
                assert_eq!(((j.hl >> 8) as u8, j.hl as u8), d.cpc.km_get_joystick(), "{what}");
            }
            4 => {
                let (s, p) = (rng.byte(), rng.byte());
                what = format!("set_delay {s} {p}");
                d.rom(0x1C6D, r(0, 0, 0, (s as u16) << 8 | p as u16));
                d.cpc.km_set_delay(s, p);
            }
            5 => {
                let key = rng.byte() % 80;
                let rep = if rng.below(2) == 0 { 0xFF } else { 0 };
                what = format!("set_repeat {key} {rep:02X}");
                d.rom(0x1CAB, r((key as u16) << 8, (rep as u16) << 8, 0, 0));
                d.cpc.km_set_repeat(key, rep);
            }
            6 => {
                let t = rng.below(32) as u8;
                let s: Vec<u8> = (0..rng.below(6)).map(|_| 0x20 + rng.byte() % 0x5F).collect();
                what = format!("set_expand {t} {s:?}");
                d.poke_bytes(0x9100, &s);
                let x = d.rom(0x1ABD, r(0, ((t as u16) << 8) | s.len() as u16, 0, 0x9100));
                let ok = d.cpc.km_set_expand(t, &s);
                assert_eq!(x.carry(), ok, "{what}");
            }
            7 => {
                let hi = rng.word(); let lo = rng.word();
                what = format!("time_set {hi:04X}{lo:04X}");
                d.rom(0x00A3, r(0, 0, hi, lo)); d.cpc.kl_time_set(hi, lo);
                let x = d.rom(0x0099, r(0, 0, 0, 0));
                assert_eq!((x.de, x.hl), d.cpc.kl_time_please(), "{what}: read back");
            }
            _ => {
                let k = 1 + rng.below(7);
                what = format!("{k} interrupts");
                for _ in 0..k {
                    tick += 1;
                    d.interrupt_both(tick % 6 < 2);
                    d.assert_same(&format!("interrupt (tick {tick}, step {step})"));
                }
            }
        }
        d.ignore_rom_banking_writes();
        d.assert_same(&format!("step {step}: {what}"));
    }
    eprintln!("compared {tick} interrupts; {chars} keys/characters were read back");
    assert!(chars > 10, "no key presses ever reached the queue");
}
