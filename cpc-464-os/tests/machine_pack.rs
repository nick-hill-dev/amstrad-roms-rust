//! Differential tests of the machine pack (gate array / CRTC / printer / PSG helpers) and
//! of the RAM-resident ROM-banking routines.  Here the hardware writes are the point, so
//! the gate-array banking writes are compared too.
mod support;
use cpc464_os::Regs;
use support::dual::*;

fn n() -> usize { std::env::var("MACHINE_N").ok().and_then(|s| s.parse().ok()).unwrap_or(2000) }
fn r(af: u16, bc: u16, de: u16, hl: u16) -> Regs { Regs { af, bc, de, hl, ..Default::default() } }
fn a(v: u8) -> u16 { (v as u16) << 8 }

#[test]
fn machine_pack_matches_rom() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0x3AC4_1000_0000_0009);
    for step in 0..n() {
        let what;
        match rng.below(10) {
            0 => { let m = rng.below(3) as u8; what = format!("mc_set_mode {m}"); d.rom(0x0776, r(a(m), 0, 0, 0)); d.cpc.mc_set_mode(m); }
            1 => {
                let base = [0x00u8, 0x40, 0x80, 0xC0][rng.below(4) as usize];
                let off = rng.word() & 0x7FF;
                what = format!("mc_screen_offset {base:02X} {off:03X}");
                d.rom(0x07C6, r(a(base), 0, 0, off)); d.cpc.mc_screen_offset(base, off);
            }
            2 | 3 => {
                let t: Vec<u8> = (0..17).map(|_| 0x40 | (rng.byte() & 0x1F)).collect();
                d.poke_bytes(0x9300, &t);
                if rng.below(2) == 0 { what = format!("mc_set_inks {t:02X?}"); d.rom(0x0799, r(0, 0, 0x9300, 0)); d.cpc.mc_set_inks(&t); }
                else { what = format!("mc_clear_inks {t:02X?}"); d.rom(0x0786, r(0, 0, 0x9300, 0)); d.cpc.mc_clear_inks(&t); }
            }
            4 => {
                what = "mc_reset_printer".into();
                d.rom(0x07E6, r(0, 0, 0, 0)); d.cpc.mc_reset_printer();
            }
            5 | 6 => {
                let ch = rng.byte();
                let busy = rng.below(3) != 0;
                for io in [&mut d.m.io.ppi_b, &mut d.cpc.hw.io.ppi_b] { if busy { *io |= 0x40 } else { *io &= !0x40 } }
                if rng.below(2) == 0 {
                    what = format!("mc_print_char {ch:02X} busy={busy}");
                    let x = d.rom(0x07F2, r(a(ch), 0, 0, 0)); let ok = d.cpc.mc_print_char(ch);
                    assert_eq!(x.carry(), ok, "{what}");
                } else {
                    what = format!("mc_send_printer {ch:02X} busy={busy}");
                    let x = d.rom(0x0807, r(a(ch), 0, 0, 0)); let ok = d.cpc.mc_send_printer(ch);
                    assert_eq!(x.carry(), ok, "{what}");
                }
            }
            7 => {
                what = "mc_busy_printer".into();
                let busy = rng.below(2) == 0;
                for io in [&mut d.m.io.ppi_b, &mut d.cpc.hw.io.ppi_b] { if busy { *io |= 0x40 } else { *io &= !0x40 } }
                let x = d.rom(0x081B, r(0, 0, 0, 0));
                assert_eq!(x.carry(), d.cpc.mc_busy_printer(), "{what}");
            }
            8 => {
                let (reg, val) = (rng.byte() % 16, rng.byte());
                what = format!("mc_sound_register {reg} {val:02X}");
                d.rom(0x0826, r(a(reg), val as u16, 0, 0)); d.cpc.mc_sound_register(reg, val);
            }
            _ => { what = "mc_wait_flyback".into(); d.rom(0x07BA, r(0, 0, 0, 0)); d.cpc.mc_wait_flyback(); }
        }
        d.assert_same(&format!("step {step}: {what}"));
    }
}

#[test]
fn rom_banking_routines_match_rom() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0xB4C1_0000_0000_000B);
    // fill RAM under the lower ROM with something recognisable
    for ad in 0..0x4000u16 { d.poke(ad, (ad ^ (ad >> 7)) as u8 | 1); }
    let mut trace: Vec<String> = vec![];
    let mut saved: Vec<(u8, u8)> = vec![];       // (ROM's A, Rust state) stack for restores
    for step in 0..n() {
        let what;
        match rng.below(12) {
            0 => { what = "u_rom_enable".to_string(); let x = d.rom(0xB900, r(0, 0, 0, 0)); let s = d.cpc.kl_u_rom_enable(); assert_eq!(x.a() & 0x0C, s.0, "{what}"); saved.push((x.a(), s.0)); }
            1 => { what = "u_rom_disable".to_string(); let x = d.rom(0xB903, r(0, 0, 0, 0)); let s = d.cpc.kl_u_rom_disable(); assert_eq!(x.a() & 0x0C, s.0, "{what}"); saved.push((x.a(), s.0)); }
            2 => { what = "l_rom_enable".to_string(); let x = d.rom(0xB906, r(0, 0, 0, 0)); let s = d.cpc.kl_l_rom_enable(); assert_eq!(x.a() & 0x0C, s.0, "{what}"); saved.push((x.a(), s.0)); }
            3 => { what = "l_rom_disable".to_string(); let x = d.rom(0xB909, r(0, 0, 0, 0)); let s = d.cpc.kl_l_rom_disable(); assert_eq!(x.a() & 0x0C, s.0, "{what}"); saved.push((x.a(), s.0)); }
            4 | 5 => {
                what = "rom_restore".to_string();
                if let Some((ra, rs)) = saved.pop() {
                    d.rom(0xB90C, r(a(ra), 0, 0, 0));
                    d.cpc.kl_rom_restore(cpc464_os::kernel::RomState(rs));
                }
            }
            6 => {
                let c = rng.byte();
                what = format!("rom_select {c}");
                let x = d.rom(0xB90F, r(0, c as u16, 0, 0));
                let s = d.cpc.kl_rom_select(c);
                assert_eq!(x.bc as u8, s.rom, "{what}: previous rom");
                d.rom(0xB918, r(0, x.bc, 0, 0));
                d.cpc.kl_rom_deselect(s);
            }
            7 => {
                let c = rng.byte();
                what = format!("probe_rom {c}");
                let x = d.rom(0xB915, r(0, c as u16, 0, 0));
                let (class, ver) = d.cpc.kl_probe_rom(c);
                assert_eq!((x.a(), x.hl), (class, ver), "{what}");
            }
            8 => { what = "curr_selection".to_string(); let x = d.rom(0xB912, r(0, 0, 0, 0)); assert_eq!(x.a(), d.cpc.kl_curr_selection(), "{what}"); }
            9 | 10 => {
                let up = rng.below(2) == 0;
                let len = 1 + rng.below(200) as u16;
                // copy between ordinary RAM and the RAM hidden under the ROMs
                let src = [0x0100u16, 0x1000, 0x3F00, 0x9000, 0xC100][rng.below(5) as usize] + rng.below(0x80) as u16;
                let dst = [0x0300u16, 0x2000, 0x9800, 0xC800][rng.below(4) as usize] + rng.below(0x80) as u16;
                if up {
                    what = format!("ldir {src:04X}->{dst:04X} x{len}");
                    d.rom(0xB91B, r(0, len, dst, src)); d.cpc.kl_ldir(src, dst, len);
                } else {
                    what = format!("lddr {src:04X}->{dst:04X} x{len}");
                    let (s2, d2) = (src + len, dst + len);
                    d.rom(0xB91E, r(0, len, d2, s2)); d.cpc.kl_lddr(s2, d2, len);
                }
            }
            _ => {
                let hl = [0x0010u16, 0x2345, 0x3FFF, 0xC000, 0xFFF0, 0x8000][rng.below(6) as usize];
                what = format!("ram_lam {hl:04X}");
                let x = d.rom(0xBACB, r(0, 0, 0, hl));
                assert_eq!(x.a(), d.cpc.kl_ram_lam(hl), "{what}");
            }
        }
        trace.push(what.clone());
        d.assert_same(&format!("step {step}: {what} (recent: {:?})", trace.iter().rev().take(6).collect::<Vec<_>>()));
    }
}
