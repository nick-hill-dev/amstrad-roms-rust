//! Differential test of the kernel's event machinery: frame-fly / fast-ticker / ticker
//! lists, event blocks, the synchronous event queue and the interrupt that drives them.
//! All event routines are a single `RET` in RAM, so only the bookkeeping is observable.
mod support;
use cpc464_os::Regs;
use support::dual::*;

fn n() -> usize { std::env::var("EVENTS_N").ok().and_then(|s| s.parse().ok()).unwrap_or(3000) }
fn r(af: u16, bc: u16, de: u16, hl: u16) -> Regs { Regs { af, bc, de, hl, ..Default::default() } }

const ROUTINE: u16 = 0x9000;
/// Eight blocks: 0-1 frame fly, 2-3 fast tickers, 4-5 tickers, 6-7 plain events.
fn block(i: u32) -> u16 { 0x9200 + 0x20 * i as u16 }

#[test]
fn kernel_events_match_rom() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    d.poke(ROUTINE, 0xC9);
    let mut rng = Rng(0xE7E7_0000_0000_0005);
    // Initialise every event exactly once (re-initialising an event that is still queued
    // would corrupt the queue on the real machine too).
    for i in 0..8u32 {
        let b = block(i);
        let class = (rng.byte() | 0x01) & 0xFF;
        match i {
            0 | 1 => { d.rom(0x0163, r(0, (class as u16) << 8, 0, 0).with_de_hl(ROUTINE, b)); d.cpc.kl_new_frame_fly(b, ROUTINE, 0, class); }
            2 | 3 => { d.rom(0x0176, r(0, (class as u16) << 8, 0, 0).with_de_hl(ROUTINE, b)); d.cpc.kl_new_fast_ticker(b, ROUTINE, 0, class); }
            4 | 5 => { d.rom(0x01D2, r(0, (class as u16) << 8, 0, 0).with_de_hl(ROUTINE, b + 6)); d.cpc.kl_init_event(b + 6, ROUTINE, 0, class); }
            _ => { d.rom(0x01D2, r(0, (class as u16) << 8, 0, 0).with_de_hl(ROUTINE, b)); d.cpc.kl_init_event(b, ROUTINE, 0, class); }
        }
    }
    d.assert_same("after initialising the events");
    let mut tick = 0u32;
    for step in 0..n() {
        let what;
        match rng.below(20) {
            0 => { let b = block(rng.below(2)); what = format!("add_frame_fly {b:04X}");
                d.rom(0x016A, r(0, 0, 0, b)); d.cpc.kl_add_frame_fly(b); }
            1 => { let b = block(rng.below(2)); what = format!("add_frame_fly {b:04X}");
                d.rom(0x016A, r(0, 0, 0, b)); d.cpc.kl_add_frame_fly(b); }
            2 => { let b = block(rng.below(2)); what = format!("del_frame_fly {b:04X}");
                let x = d.rom(0x0170, r(0, 0, 0, b)); assert_eq!(x.carry(), d.cpc.kl_del_frame_fly(b), "{what}"); }
            3 => { let b = block(2 + rng.below(2)); what = format!("add_fast_ticker {b:04X}");
                d.rom(0x017D, r(0, 0, 0, b)); d.cpc.kl_add_fast_ticker(b); }
            4 => { let b = block(2 + rng.below(2)); what = format!("add_fast_ticker {b:04X}");
                d.rom(0x017D, r(0, 0, 0, b)); d.cpc.kl_add_fast_ticker(b); }
            5 => { let b = block(2 + rng.below(2)); what = format!("del_fast_ticker {b:04X}");
                let x = d.rom(0x0183, r(0, 0, 0, b)); assert_eq!(x.carry(), d.cpc.kl_del_fast_ticker(b), "{what}"); }
            6 | 7 => { let b = block(4 + rng.below(2)); let (i, c) = (rng.below(4) as u16, rng.below(4) as u16);
                what = format!("add_ticker {b:04X} {i} {c}");
                d.rom(0x01B3, r(0, c, i, b)); d.cpc.kl_add_ticker(b, i, c); }
            8 => { let b = block(4 + rng.below(2)); what = format!("del_ticker {b:04X}");
                let x = d.rom(0x01C5, r(0, 0, 0, b)); let y = d.cpc.kl_del_ticker(b);
                assert_eq!(x.carry(), y.is_some(), "{what}"); if let Some(v) = y { assert_eq!(x.de, v, "{what}: remaining"); } }
            9 => { let b = block(6 + rng.below(2));
                if rng.below(40) == 0 {          // rare: a disarmed event stays dead until re-initialised
                    what = format!("disarm {b:04X}");
                    d.rom(0x028E, r(0, 0, 0, b)); d.cpc.kl_disarm_event(b);
                } else {
                    what = format!("event {b:04X}");
                    d.rom(0x01E2, r(0, 0, 0, b)); d.cpc.kl_event(b);
                } }
            10 | 11 | 12 => { let b = block(6 + rng.below(2)); what = format!("event {b:04X}");
                d.rom(0x01E2, r(0, 0, 0, b)); d.cpc.kl_event(b); }
            13 => { what = "sync_reset".into(); d.rom(0x0228, r(0, 0, 0, 0)); d.cpc.kl_sync_reset(); }
            14 => { what = "next_sync + do_sync + done_sync".into();
                let x = d.rom(0x0256, r(0, 0, 0, 0));
                let y = d.cpc.kl_next_sync();
                assert_eq!(x.carry(), y.is_some(), "{what}: found");
                if let Some((ev, pri)) = y {
                    assert_eq!((x.hl, x.a()), (ev, pri), "{what}: result");
                    d.rom(0x021A, r(0, 0, 0, ev)); d.cpc.kl_do_sync(ev);
                    d.rom(0x0277, r((pri as u16) << 8, 0, 0, ev)); d.cpc.kl_done_sync(ev, pri);
                } }
            15 if rng.below(25) == 0 => { let b = block(6 + rng.below(2)); what = format!("del_synchronous {b:04X}");
                d.rom(0x0285, r(0, 0, 0, b)); d.cpc.kl_del_synchronous(b); }
            16 if rng.below(25) == 0 => { let b = block(6 + rng.below(2)); what = format!("disarm {b:04X}");
                d.rom(0x028E, r(0, 0, 0, b)); d.cpc.kl_disarm_event(b); }
            15 | 16 => { let b = block(6 + rng.below(2)); what = format!("event {b:04X}");
                d.rom(0x01E2, r(0, 0, 0, b)); d.cpc.kl_event(b); }
            17 => { what = "event disable/enable".into();
                if rng.below(2) == 0 { d.rom(0x0295, r(0, 0, 0, 0)); d.cpc.kl_event_disable(); }
                else { d.rom(0x029B, r(0, 0, 0, 0)); d.cpc.kl_event_enable(); } }
            _ => {
                let k = 1 + rng.below(8);
                what = format!("{k} interrupts");
                for _ in 0..k { tick += 1; d.interrupt_both(tick % 6 < 2); d.assert_same(&format!("interrupt {tick} (step {step})")); }
            }
        }
        d.ignore_rom_banking_writes();
        d.assert_same(&format!("step {step}: {what}"));
    }
    eprintln!("compared {tick} interrupts");
}

trait WithDeHl { fn with_de_hl(self, de: u16, hl: u16) -> Self; }
impl WithDeHl for Regs { fn with_de_hl(mut self, de: u16, hl: u16) -> Self { self.de = de; self.hl = hl; self } }
