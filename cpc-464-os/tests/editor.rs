//! Differential test of the BASIC line editor (`BD3A`) against the original ROM.
//!
//! Keys are fed through a KM expansion string (token `9F`), the cheapest way to hand
//! both machines the same key sequence without the keyboard matrix.
mod support;
use cpc464_os::Regs;
use support::dual::*;
use support::machine::*;

const BUF: u16 = 0x9000;
const SCRIPT: u16 = 0x9100;

const SPECIAL: &[u8] = &[
    0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB,
    0xE0, 0xE1, 0xEF, 0x7F, 0x10,
];

fn n_trials() -> usize { std::env::var("EDITOR_N").ok().and_then(|s| s.parse().ok()).unwrap_or(300) }

#[test]
fn line_editor_matches_rom() {
    let Some(rom) = rom_path() else { eprintln!("OS464_ROM not set; skipping"); return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0xED17_0001_2345_6789);
    for trial in 0..n_trials() {
        // ---- random window and cursor position ------------------------------------
        let desc_win;
        if rng.below(2) == 0 {
            let (c1, c2) = (1 + rng.below(40) as u8, 1 + rng.below(40) as u8);
            let (r1, r2) = (1 + rng.below(25) as u8, 1 + rng.below(25) as u8);
            d.rom(0x120C, Regs { hl: (c1 as u16) << 8 | r1 as u16, de: (c2 as u16) << 8 | r2 as u16, ..Default::default() });
            d.cpc.txt_win_enable(c1, c2, r1, r2);
            d.rom(0x1400, Regs { af: 0x0C00, ..Default::default() });
            d.cpc.txt_output(0x0C);
            desc_win = format!("window cols {c1}..{c2} rows {r1}..{r2}");
        } else {
            desc_win = "same window".into();
        }
        let (col, row) = (1 + rng.below(12) as u8, 1 + rng.below(12) as u8);
        d.rom(0x1174, Regs { hl: (col as u16) << 8 | row as u16, ..Default::default() });
        d.cpc.txt_set_cursor(col, row);

        // ---- initial text ----------------------------------------------------------
        let len = if rng.below(4) == 0 { 0 } else { rng.below(60) as usize };
        let mut text: Vec<u8> = (0..len).map(|_| 0x20 + (rng.byte() % 0x5F)).collect();
        d.poke_bytes(BUF, &text);
        d.poke(BUF + len as u16, 0);

        // ---- key script ------------------------------------------------------------
        let nkeys = 1 + rng.below(28) as usize;
        let mut keys: Vec<u8> = (0..nkeys).map(|_| match rng.below(10) {
            0..=3 => 0x20 + (rng.byte() % 0x5F),
            _ => SPECIAL[rng.below(SPECIAL.len() as u32) as usize],
        }).collect();
        keys.push(if rng.below(3) == 0 { 0xFC } else { 0x0D });
        // Debug aid: EDITOR_TRIAL=n EDITOR_KEYS=k cuts trial n's script to k keys (+ ENTER).
        if let (Ok(t), Ok(k)) = (std::env::var("EDITOR_TRIAL"), std::env::var("EDITOR_KEYS")) {
            if t.parse() == Ok(trial) { keys.truncate(k.parse().unwrap()); keys.push(0x0D); }
        }
        d.poke_bytes(SCRIPT, &keys);
        let r = d.rom(0x1ABD, Regs { bc: 0x9F00 | keys.len() as u16, hl: SCRIPT, ..Default::default() });
        let ok = d.cpc.km_set_expand(0x9F, &keys);
        assert_eq!(r.carry(), ok, "trial {trial}: KM SET EXPAND result");
        d.poke(0xB4DE, 0);
        d.poke(0xB4DF, 0x9F);
        // set_expand already cancelled/armed; force both machines to the same state
        d.ignore_rom_banking_writes();
        d.assert_same(&format!("trial {trial}: before editing"));

        // ---- run -------------------------------------------------------------------
        let what = format!(
            "trial {trial}: {desc_win}, cursor ({col},{row}), text {:?}, keys {keys:02X?}",
            String::from_utf8_lossy(&text)
        );
        let rr = if std::env::var("EDTRACE").is_ok() && trial + 1 == n_trials() {
            // Debug aid: print the PCs executed inside the COPY-key handler.
            let c = &mut d.cpu;
            c.set_hl(BUF); c.set_af(0); c.set_bc(0); c.set_de(0);
            c.sp = 0xC000 - 2;
            d.m.ram[0xBFFE] = RETURN_TO as u8; d.m.ram[0xBFFF] = (RETURN_TO >> 8) as u8;
            c.pc = 0x2A98;
            while c.pc != RETURN_TO {
                if c.pc == 0x2AC6 { eprintln!("PC key {:02X} bc={:04X} hl={:04X} copy={:02X}{:02X} scroll={} cur={:02X}{:02X}", c.a, c.bc(), c.hl(), d.m.ram[0xB8DF], d.m.ram[0xB8DE], d.m.ram[0xB28C], d.m.ram[0xB286], d.m.ram[0xB285]); }
                if [0x2D85,0x2D94,0x2D9B,0x2DA0,0x2D67,0x2B93,0x2D50].contains(&c.pc) { eprintln!("PC T {:04X} AF={:04X} BC={:04X} DE={:04X} HL={:04X}", c.pc, c.af(), c.bc(), c.de(), c.hl()); }
                if false && (0x2CEA..0x2D30).contains(&c.pc) || (0x13AB..0x1400).contains(&c.pc) {
                    eprintln!("PC {:04X} AF={:04X} BC={:04X} DE={:04X} HL={:04X}", c.pc, c.af(), c.bc(), c.de(), c.hl());
                }
                let before = (d.m.ram[0xB8DE], d.m.ram[0xB8DF]);
                let pc0 = c.pc;
                c.step(&mut d.m);
                let after = (d.m.ram[0xB8DE], d.m.ram[0xB8DF]);
                if before != after { eprintln!("PC copy write at {pc0:04X}: {:02X}{:02X} -> {:02X}{:02X} (A={:02X} C={:02X} HL={:04X})", before.1, before.0, after.1, after.0, c.a, c.c, c.hl()); }
            }
            Regs { af: c.af(), bc: c.bc(), de: c.de(), hl: c.hl(), ix: c.ix, iy: c.iy }
        } else { d.rom(0x2A98, Regs { hl: BUF, ..Default::default() }) };
        let out = d.cpc.km_edit_line(BUF);
        text.clear();
        assert_eq!(rr.a(), out.key, "{what}: returned key");
        assert_eq!(rr.carry(), out.carry, "{what}: carry");
        assert_eq!(rr.af & 0x40 != 0, out.zero, "{what}: zero");
        d.ignore_rom_banking_writes();
        d.assert_same(&what);
    }
}
