//! Differential tests of the screen pack against the original ROM.
mod support;
use cpc464_os::screen::AccessMode;
use cpc464_os::Regs;
use support::dual::*;

fn n() -> usize { std::env::var("SCREEN_N").ok().and_then(|s| s.parse().ok()).unwrap_or(600) }
fn r(af: u16, bc: u16, de: u16, hl: u16) -> Regs { Regs { af, bc, de, hl, ..Default::default() } }
fn a(v: u8) -> u16 { (v as u16) << 8 }
/// An address that is safe to scribble on: the 4000-7FFF block or the normal screen.
fn safe_addr(rng: &mut Rng) -> u16 {
    if rng.below(2) == 0 { 0x4000 + rng.below(0x4000) as u16 } else { 0xC000 + rng.below(0x4000) as u16 }
}
fn w(h: u8, l: u8) -> u16 { (h as u16) << 8 | l as u16 }

/// Put the screen into a random mode / base / offset on both machines.
fn random_screen(d: &mut Dual, rng: &mut Rng) {
    let mode = rng.below(3) as u8;
    d.rom(0x0ACA, r(a(mode), 0, 0, 0));
    d.cpc.scr_set_mode(mode);
    let base = [0x00u8, 0x40, 0xC0][rng.below(3) as usize];
    d.rom(0x0B45, r(a(base), 0, 0, 0));
    d.cpc.scr_set_base(base);
    let off = (rng.word() & 0x7FE) as u16;
    d.rom(0x0B3C, r(0, 0, 0, off));
    d.cpc.scr_set_offset(off);
    // random screen contents for the roll / read-back tests
    for _ in 0..400 {
        let ad = safe_addr(rng);
        let v = rng.byte();
        d.poke(ad, v);
    }
}

#[test]
fn screen_pack_matches_rom() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0x5C4E_E000_0000_0001);
    for i in 0..n() {
        if i % 8 == 0 { random_screen(&mut d, &mut rng); }
        let op = rng.below(24);
        let what: String;
        match op {
            0 => { // SCR SET MODE (+ the whole mode-change side effects)
                let m = rng.below(4) as u8;
                what = format!("set_mode {m}");
                d.rom(0x0ACA, r(a(m), 0, 0, 0)); d.cpc.scr_set_mode(m);
            }
            1 => {
                what = "clear".into();
                d.rom(0x0AF7, r(0, 0, 0, 0)); d.cpc.scr_clear();
            }
            2 => { // CHAR POSITION
                let (c, rw) = (rng.byte() % 80, rng.byte() % 25);
                what = format!("char_position {c},{rw}");
                let x = d.rom(0x0B64, r(0, 0, 0, w(c, rw)));
                let (ad, b) = d.cpc.scr_char_position(c, rw);
                assert_eq!((x.hl, (x.bc >> 8) as u8), (ad, b), "{what}");
            }
            3 => { // DOT POSITION
                let (xx, yy) = (rng.below(700) as u16, rng.below(220) as u16);
                what = format!("dot_position {xx},{yy}");
                let x = d.rom(0x0BA9, r(0, 0, xx, yy));
                let p = d.cpc.scr_dot_position(xx, yy);
                assert_eq!((x.hl, x.bc as u8, (x.bc >> 8) as u8), (p.addr, p.mask, p.ppb_minus_1), "{what}");
            }
            4 => { // ACCESS + WRITE + READ
                let m = rng.below(4) as u8;
                let mode = [AccessMode::Force, AccessMode::Xor, AccessMode::And, AccessMode::Or][m as usize];
                let ad = safe_addr(&mut rng);
                let (ink, mask) = (rng.byte(), rng.byte());
                what = format!("access {m} write {ad:04X} ink {ink:02X} mask {mask:02X}");
                d.rom(0x0C49, r(a(m), 0, 0, 0)); d.cpc.scr_access(mode);
                d.rom(0xBDE8, r(0, w(ink, mask), 0, ad)); d.cpc.scr_write(ad, ink, mask);
                let x = d.rom(0xBDE5, r(0, mask as u16, 0, ad));
                let v = d.cpc.scr_read(ad, mask);
                assert_eq!(x.a(), v, "{what}: read back");
            }
            5 => { // PIXELS
                let ad = safe_addr(&mut rng);
                let (ink, mask) = (rng.byte(), rng.byte());
                what = format!("pixels {ad:04X} {ink:02X} {mask:02X}");
                d.rom(0x0C6B, r(0, w(ink, mask), 0, ad)); d.cpc.scr_pixels(ad, ink, mask);
            }
            6 => { // INK ENCODE / DECODE
                let pen = rng.byte() & 0x0F;
                what = format!("ink_encode {pen}");
                let x = d.rom(0x0C86, r(a(pen), 0, 0, 0));
                assert_eq!(x.a(), d.cpc.scr_ink_encode(pen), "{what}");
                let byte = rng.byte();
                let x = d.rom(0x0CA0, r(a(byte), 0, 0, 0));
                assert_eq!(x.a(), d.cpc.scr_ink_decode(byte), "{what}: decode {byte:02X}");
            }
            7 => { // FLASHING
                let (p1, p2) = (rng.byte(), rng.byte());
                what = format!("set_flashing {p1} {p2}");
                d.rom(0x0CE4, r(0, 0, 0, w(p1, p2))); d.cpc.scr_set_flashing(p1, p2);
                let x = d.rom(0x0CE8, r(0, 0, 0, 0));
                assert_eq!(x.hl, w(p1, p2), "{what}: get");
                assert_eq!(d.cpc.scr_get_flashing(), (p1, p2));
            }
            8 => { // SET/GET INK
                let (pen, c1, c2) = (rng.byte(), rng.byte() % 32, rng.byte() % 32);
                what = format!("set_ink {pen} {c1} {c2}");
                d.rom(0x0CEC, r(a(pen), w(c1, c2), 0, 0)); d.cpc.scr_set_ink(pen, c1, c2);
                let x = d.rom(0x0D14, r(a(pen), 0, 0, 0));
                assert_eq!(x.bc, w(c1, c2), "{what}: get");
                assert_eq!(d.cpc.scr_get_ink(pen), (c1, c2));
            }
            9 => {
                let (c1, c2) = (rng.byte() % 32, rng.byte() % 32);
                what = format!("set_border {c1} {c2}");
                d.rom(0x0CF1, r(0, w(c1, c2), 0, 0)); d.cpc.scr_set_border(c1, c2);
                let x = d.rom(0x0D19, r(0, 0, 0, 0));
                assert_eq!(x.bc, w(c1, c2), "{what}: get");
            }
            10 => { // FILL BOX
                let (c1, c2) = (rng.byte() % 20, rng.byte() % 20);
                let (r1, r2) = (rng.byte() % 25, rng.byte() % 25);
                let (c1, c2) = (c1.min(c2), c1.max(c2));
                let (r1, r2) = (r1.min(r2), r1.max(r2));
                let pat = rng.byte();
                what = format!("fill_box {pat:02X} ({c1},{r1})-({c2},{r2})");
                d.rom(0x0DB3, r(a(pat), 0, w(c2, r2), w(c1, r1)));
                d.cpc.scr_fill_box(pat, (c1, r1), (c2, r2));
            }
            11 => { // FLOOD BOX
                let ad = safe_addr(&mut rng);
                let (wd, ln, pat) = (1 + rng.byte() % 30, 1 + rng.byte() % 40, rng.byte());
                what = format!("flood_box {ad:04X} {wd}x{ln} {pat:02X}");
                d.rom(0x0DB7, r(0, pat as u16, w(wd, ln), ad));
                d.cpc.scr_flood_box(ad, wd, ln, pat);
            }
            12 => { // CHAR INVERT
                let (b, c) = (rng.byte(), rng.byte());
                let (col, row) = (rng.byte() % 20, rng.byte() % 25);
                what = format!("char_invert {b:02X} {c:02X} ({col},{row})");
                d.rom(0x0DDF, r(0, w(b, c), 0, w(col, row)));
                d.cpc.scr_char_invert(b, c, col, row);
            }
            13 => { // HW ROLL
                let (dir, pat) = (rng.byte() & 1, rng.byte());
                what = format!("hw_roll dir {dir} pat {pat:02X}");
                d.rom(0x0DFA, r(a(pat), w(dir, 0), 0, 0));
                d.cpc.scr_hw_roll(dir, pat);
            }
            14 => { // SW ROLL
                let (c1, c2) = (rng.byte() % 20, rng.byte() % 20);
                let (r1, r2) = (rng.byte() % 25, rng.byte() % 25);
                let (c1, c2) = (c1.min(c2), c1.max(c2));
                let (r1, r2) = (r1.min(r2), r1.max(r2));
                let (dir, pat) = (rng.byte() & 1, rng.byte());
                what = format!("sw_roll dir {dir} pat {pat:02X} ({c1},{r1})-({c2},{r2})");
                d.rom(0x0E3E, r(a(pat), w(dir, 0), w(c2, r2), w(c1, r1)));
                d.cpc.scr_sw_roll(dir, pat, (c1, r1), (c2, r2));
            }
            15 => { // UNPACK
                let mut m = [0u8; 8];
                for b in m.iter_mut() { *b = rng.byte(); }
                what = format!("unpack {m:02X?}");
                d.poke_bytes(0x9200, &m);
                d.rom(0x0EF3, r(0, 0, 0x9300, 0x9200));
                let out = d.cpc.scr_unpack(&m);
                for (k, b) in out.iter().enumerate() { d.cpc.wr8(0x9300 + k as u16, *b); }
            }
            16 => { // REPACK
                let pen = rng.byte();
                let (col, row) = (rng.byte() % 20, rng.byte() % 25);
                what = format!("repack pen {pen:02X} ({col},{row})");
                d.rom(0x0F49, r(a(pen), 0, 0x9300, w(col, row)));
                let out = d.cpc.scr_repack(pen, col, row);
                for (k, b) in out.iter().enumerate() { d.cpc.wr8(0x9300 + k as u16, *b); }
            }
            17 | 18 => { // HORIZONTAL
                let ink = rng.byte();
                let (x1, x2) = (rng.below(660) as u16, rng.below(660) as u16);
                let (x1, x2) = (x1.min(x2), x1.max(x2));
                let y = rng.below(210) as u16;
                what = format!("horizontal ink {ink:02X} x {x1}..{x2} y {y} mode {} access {:?} base {:02X} offset {:04X}", d.cpc.scr_get_mode(), d.cpc.scr_access_mode(), d.cpc.rd8(0xB1D4), d.cpc.rd16(0xB1D2));
                d.rom(0x0FC4, r(a(ink), x2, x1, y));
                d.cpc.scr_horizontal(ink, x1, x2, y);
            }
            19 | 20 => { // VERTICAL
                let ink = rng.byte();
                let x = rng.below(660) as u16;
                let (y1, y2) = (rng.below(210) as u16, rng.below(210) as u16);
                let (y1, y2) = (y1.min(y2), y1.max(y2));
                what = format!("vertical ink {ink:02X} x {x} y {y1}..{y2}");
                d.rom(0x102F, r(a(ink), y2, x, y1));
                d.cpc.scr_vertical(ink, x, y1, y2);
            }
            21 => { // OFFSET / BASE / LOCATION
                let off = rng.word() & 0x7FE;
                let base = [0x00u8, 0x40, 0xC0][rng.below(3) as usize];
                what = format!("location base {base:02X} offset {off:03X}");
                d.rom(0x0B45, r(a(base), 0, 0, 0)); d.cpc.scr_set_base(base);
                d.rom(0x0B3C, r(0, 0, 0, off)); d.cpc.scr_set_offset(off);
                let x = d.rom(0x0B50, r(0, 0, 0, 0));
                let (o, b) = d.cpc.scr_get_location();
                assert_eq!((x.hl, x.a()), (o, b), "{what}");
            }
            22 => { // CHAR LIMITS / GET MODE
                what = "char_limits".into();
                let x = d.rom(0x0B57, r(0, 0, 0, 0));
                let (c, rw) = d.cpc.scr_char_limits();
                assert_eq!(((x.bc >> 8) as u8, x.bc as u8), (c, rw), "{what}: (B, C)");
            }
            _ => {
                what = "flash tick".into();
                // Run the flash handler on both machines.
                d.rom(0x0D5B, r(0, 0, 0, 0)); d.cpc.scr_flash_tick();
            }
        }
        d.ignore_rom_banking_writes();
        d.assert_same(&format!("op #{i}: {what}"));
    }
}

#[test]
fn horizontal_lines_small_cases() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    for mode in 0..3u8 {
        d.rom(0x0ACA, r(a(mode), 0, 0, 0)); d.cpc.scr_set_mode(mode);
        for acc in 0..4u8 {
            let am = [AccessMode::Force, AccessMode::Xor, AccessMode::And, AccessMode::Or][acc as usize];
            d.rom(0x0C49, r(a(acc), 0, 0, 0)); d.cpc.scr_access(am);
            for off in [0u16, 0x7E6, 0x7FE, 0x400, 0x7B0] {
                d.rom(0x0B3C, r(0, 0, 0, off)); d.cpc.scr_set_offset(off);
                for x1 in [0u16, 1, 3, 29, 30, 251, 300] {
                    for x2 in x1..x1 + 130 {
                        d.rom(0x0FC4, r(a(0x73), x2, x1, 100));
                        d.cpc.scr_horizontal(0x73, x1, x2, 100);
                        d.ignore_rom_banking_writes();
                        d.assert_same(&format!("mode {mode} access {acc} offset {off:03X} x {x1}..{x2}"));
                    }
                }
            }
        }
    }
}
