//! Differential test: cold start of the original ROM (run in a Z80 emulator) versus
//! `Cpc::power_on`.  Needs the ROM image: set `OS464_ROM=/path/to/OS464.ROM`.
mod support;
use cpc464_os::Cpc;
use support::machine::*;
use support::z80::Z80;

fn rom() -> Option<Vec<u8>> {
    let path = std::env::var("OS464_ROM").ok()?;
    std::fs::read(path).ok()
}

#[test]
fn cold_start_matches_the_rom() {
    let Some(rom) = rom() else { eprintln!("OS464_ROM not set; skipping"); return };
    // ---- the real thing ----
    let mut m = RomMachine::new(&rom);
    let mut cpu = Z80::new();
    run_until(&mut cpu, &mut m, 0x0095, 50_000_000).expect("ROM boot");
    // ---- the port ----
    let mut cpc = Cpc::new(&rom, TestHw::new());
    let target = cpc.power_on();
    assert_eq!(target.entry, 0xC006);

    if std::env::var("DUMP").is_ok() {
        eprintln!("ROM:\n{}", dump_mode1(&m.ram, 8, 20));
        eprintln!("RUST:\n{}", dump_mode1(&cpc.ram[..], 8, 20));
    }
    // Compare RAM regions that are firmware state.
    let mut diffs = vec![];
    for a in 0..0x10000usize {
        if (0xBF00..0xC000).contains(&a) { continue; } // stack area of the real run
        if m.ram[a] != cpc.ram[a] { diffs.push(a); }
    }
    let mut pages = std::collections::BTreeMap::new();
    for a in &diffs { *pages.entry(a >> 8).or_insert(0) += 1; }
    eprintln!("diff bytes per 256-byte page: {:?}", pages.iter().map(|(p, n)| format!("{:02X}xx:{}", p, n)).collect::<Vec<_>>());
    let shown: Vec<String> = diffs.iter().take(40).map(|a| format!("{a:04X}: rom={:02X} rust={:02X}", m.ram[*a], cpc.ram[*a])).collect();
    assert!(diffs.is_empty(), "{} bytes differ:\n{}", diffs.len(), shown.join("\n"));

    // The hardware sees the same writes, in the same order (compared on the bits the
    // hardware decodes: the high byte of the port plus the value).
    let norm = |l: &[(u16, u8)]| l.iter().map(|(p, v)| (p >> 8, *v)).collect::<Vec<_>>();
    let rom_log = norm(&m.io.log[1..]); // [0] is the reset vector's own `OUT 7F89` before 0580
    let rust_log = norm(&cpc.hw.io.log);
    let first = rom_log.iter().zip(&rust_log).position(|(a, b)| a != b);
    assert_eq!(first, None, "first differing hardware write; rom has {} rust has {}", rom_log.len(), rust_log.len());
    assert_eq!(rom_log.len(), rust_log.len());
}
