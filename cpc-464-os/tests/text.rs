//! Differential tests of the text VDU against the original ROM.
mod support;
use cpc464_os::Regs;
use support::dual::*;

fn regs_a(a: u8) -> Regs { Regs { af: (a as u16) << 8, ..Default::default() } }

#[test]
fn txt_output_matches_rom_for_printable_text() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    for (i, ch) in b"Hello, World! 0123456789 The quick brown fox".iter().enumerate() {
        d.rom(0x1400, regs_a(*ch));
        d.cpc.txt_output(*ch);
        d.assert_same(&format!("char #{i} {:?}", *ch as char));
    }
}

#[test]
fn txt_output_matches_rom_for_random_bytes() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0x1234_5678_9ABC_DEF1);
    let mut log: Vec<u8> = vec![];
    for i in 0..3000 {
        // Favour printable characters and the newline codes, with plenty of control codes.
        let ch = match rng.below(10) {
            0 => rng.byte(),
            1 => rng.byte() & 0x1F,
            _ => 0x20 + (rng.byte() % 0x5F),
        };
        log.push(ch);
        d.rom(0x1400, regs_a(ch));
        d.cpc.txt_output(ch);
        if i % 1 == 0 {
            let tail: Vec<u8> = log.iter().rev().take(12).rev().copied().collect();
            d.assert_same(&format!("after byte #{i} (recent bytes {tail:02X?})"));
        }
    }
}

#[test]
fn txt_rd_char_matches_rom() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0x0BAD_CAFE_1234_0001);
    // Fill the screen with random text, some of it in inverse video.
    for i in 0..600 {
        let ch = 0x20 + (rng.byte() % 0x5F);
        if i % 50 == 25 {
            d.rom(0x1400, regs_a(0x0F)); d.cpc.txt_output(0x0F); // control code needs params
        }
        d.rom(0x1400, regs_a(ch));
        d.cpc.txt_output(ch);
    }
    d.assert_same("after filling the screen");
    for i in 0..400 {
        let (col, row) = (1 + rng.below(44) as u8, 1 + rng.below(28) as u8);
        d.rom(0x1174, Regs { hl: (col as u16) << 8 | row as u16, ..Default::default() });
        d.cpc.txt_set_cursor(col, row);
        let r = d.rom(0x13AB, Regs::default());
        let got = d.cpc.txt_rd_char();
        let what = format!("read #{i} at ({col},{row}): rom A={:02X} carry={}", r.a(), r.carry());
        assert_eq!(r.carry(), got.is_some(), "{what}: found flag (rust {got:?})");
        if r.carry() { assert_eq!(Some(r.a()), got, "{what}"); }
        d.ignore_rom_banking_writes();
        d.assert_same(&what);
    }
}
