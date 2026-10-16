//! Diagnostics (not a test of anything): `LINES="a|b" cargo test --test debug -- --nocapture`
//! prints a RAM range of both machines after the session.
mod support;
use support::session::{roms, run};

#[test]
fn show() {
    let Ok(lines) = std::env::var("LINES") else { return };
    let Some((os, basic)) = roms() else { return };
    let lines: Vec<&str> = lines.split('|').collect();
    let s = run(&os, &basic, &lines);
    let from = u16::from_str_radix(&std::env::var("FROM").unwrap_or("AE40".into()), 16).unwrap() as usize;
    let to = u16::from_str_radix(&std::env::var("TO").unwrap_or("AE80".into()), 16).unwrap() as usize;
    for a in (from..to).step_by(16) {
        let r: Vec<String> = (a..a + 16).map(|i| format!("{:02X}", s.rom.ram[i])).collect();
        let u: Vec<String> = (a..a + 16).map(|i| {
            let (x, y) = (s.rom.ram[i], s.rust.cpc.ram[i]);
            if x == y { "..".into() } else { format!("{y:02X}") }
        }).collect();
        println!("{a:04X} rom  {}\n     rust {}", r.join(" "), u.join(" "));
    }
    let (a, b) = s.dump(4);
    if std::env::var("SCREEN").is_ok() { println!("{a}\n{b}"); }
}
