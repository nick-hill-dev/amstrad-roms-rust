//! Differential tests of the graphics VDU against the original ROM.
mod support;
use cpc464_os::screen::AccessMode;
use cpc464_os::Regs;
use support::dual::*;

fn n() -> usize { std::env::var("GRAPHICS_N").ok().and_then(|s| s.parse().ok()).unwrap_or(3000) }
fn r(af: u16, bc: u16, de: u16, hl: u16) -> Regs { Regs { af, bc, de, hl, ..Default::default() } }
fn a(v: u8) -> u16 { (v as u16) << 8 }

/// A user coordinate: mostly on screen, sometimes far outside.
fn coord(rng: &mut Rng, span: i32) -> i16 {
    match rng.below(10) {
        0 => (rng.word() as i16),
        1 => -(rng.below(300) as i16),
        _ => (rng.below(span as u32 + 100) as i32 - 50) as i16,
    }
}

#[test]
fn graphics_vdu_matches_rom() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0x62A9_0000_0000_0007);
    let mut drawing = std::collections::BTreeMap::new();
    for i in 0..n() {
        if i % 10 == 0 {
            let mode = rng.below(3) as u8;
            d.rom(0x0ACA, r(a(mode), 0, 0, 0)); d.cpc.scr_set_mode(mode);
            let m = rng.below(4) as u8;
            let am = [AccessMode::Force, AccessMode::Xor, AccessMode::And, AccessMode::Or][m as usize];
            d.rom(0x0C49, r(a(m), 0, 0, 0)); d.cpc.scr_access(am);
            let (p, q) = (rng.byte() % 16, rng.byte() % 16);
            d.rom(0x17F6, r(a(p), 0, 0, 0)); d.cpc.gra_set_pen(p);
            d.rom(0x17FD, r(a(q), 0, 0, 0)); d.cpc.gra_set_paper(q);
        }
        let op = rng.below(16);
        let before: Vec<u8> = d.cpc.ram[0xC000..].to_vec();
        let what;
        let (x, y) = (coord(&mut rng, 640), coord(&mut rng, 400));
        match op {
            0 => { what = format!("move_abs {x},{y}"); d.rom(0x15F4, r(0, 0, x as u16, y as u16)); d.cpc.gra_move_absolute(x, y); }
            1 => { what = format!("move_rel {x},{y}"); d.rom(0x15F1, r(0, 0, x as u16, y as u16)); d.cpc.gra_move_relative(x, y); }
            2 => {
                what = format!("set_origin {x},{y}");
                d.rom(0x1604, r(0, 0, x as u16, y as u16)); d.cpc.gra_set_origin(x, y);
                let g = d.rom(0x1612, r(0, 0, 0, 0));
                assert_eq!((g.de as i16, g.hl as i16), d.cpc.gra_get_origin(), "{what}: get");
            }
            3 => {
                let (x2, y2) = (coord(&mut rng, 640), coord(&mut rng, 400));
                what = format!("window x {x},{x2} y {y},{y2}");
                d.rom(0x1734, r(0, 0, x as u16, x2 as u16)); d.cpc.gra_win_width(x, x2);
                d.rom(0x1779, r(0, 0, y as u16, y2 as u16)); d.cpc.gra_win_height(y, y2);
                let g = d.rom(0x17A6, r(0, 0, 0, 0));
                assert_eq!((g.de as i16, g.hl as i16), d.cpc.gra_get_w_width(), "{what}: get width");
                let g = d.rom(0x17BC, r(0, 0, 0, 0));
                assert_eq!((g.de as i16, g.hl as i16), d.cpc.gra_get_w_height(), "{what}: get height");
            }
            4 => { what = "clear_window".into(); d.rom(0x17C5, r(0, 0, 0, 0)); d.cpc.gra_clear_window(); }
            5 => {
                let (p, q) = (rng.byte() % 16, rng.byte() % 16);
                what = format!("pen {p} paper {q}");
                d.rom(0x17F6, r(a(p), 0, 0, 0)); d.cpc.gra_set_pen(p);
                d.rom(0x17FD, r(a(q), 0, 0, 0)); d.cpc.gra_set_paper(q);
                assert_eq!(d.rom(0x1804, r(0, 0, 0, 0)).a(), d.cpc.gra_get_pen(), "{what}: get pen");
                assert_eq!(d.rom(0x180A, r(0, 0, 0, 0)).a(), d.cpc.gra_get_paper(), "{what}: get paper");
            }
            6 | 7 => { what = format!("plot_abs {x},{y}"); d.rom(0x1813, r(0, 0, x as u16, y as u16)); d.cpc.gra_plot_absolute(x, y); }
            8 => { what = format!("plot_rel {x},{y}"); d.rom(0x1810, r(0, 0, x as u16, y as u16)); d.cpc.gra_plot_relative(x, y); }
            9 | 10 => {
                what = format!("test_abs {x},{y}");
                let t = d.rom(0x1827, r(0, 0, x as u16, y as u16));
                assert_eq!(t.a(), d.cpc.gra_test_absolute(x, y), "{what}");
            }
            11 => {
                what = format!("test_rel {x},{y}");
                let t = d.rom(0x1824, r(0, 0, x as u16, y as u16));
                assert_eq!(t.a(), d.cpc.gra_test_relative(x, y), "{what}");
            }
            12 | 13 => { what = format!("line_abs {x},{y}"); d.rom(0x1839, r(0, 0, x as u16, y as u16)); d.cpc.gra_line_absolute(x, y); }
            14 => { what = format!("line_rel {x},{y}"); d.rom(0x1836, r(0, 0, x as u16, y as u16)); d.cpc.gra_line_relative(x, y); }
            _ => {
                let ch = rng.byte();
                what = format!("wr_char {ch:02X} at cursor");
                d.rom(0x1945, r(a(ch), 0, 0, 0)); d.cpc.gra_wr_char(ch);
            }
        }
        d.ignore_rom_banking_writes();
        d.assert_same(&format!("op #{i}: {what}"));
        if d.cpc.ram[0xC000..] != before[..] { *drawing.entry(op).or_insert(0) += 1; }
    }
    eprintln!("graphics test finished; ops that changed the screen, by op number: {drawing:?}");
    for op in [4, 6, 8, 12, 14, 15] { assert!(drawing.get(&op).copied().unwrap_or(0) > 20, "op {op} almost never drew"); }
}
