//! Differential tests of the cassette statements.  The motor is already running and
//! file names start with `!` (no messages), so no key presses or interrupts are needed.
//! What one side records is played back into both for loading.
//!
//! The firmware waits for the motor to run up with interrupts enabled; the emulated ROM
//! gets no interrupts, while the Rust firmware runs one per 10.8 ms of waiting.  So the
//! state the interrupts maintain (the clock, the 1/50 s divider, the keyboard scan and
//! repeat state) is not compared here, nor are the hardware writes (the extra interrupts
//! scan the keyboard and update the sound chip); the tape signal is compared instead.
//! Needs `OS464_ROM` and `BASIC_ROM`.
mod support;
use support::session::{roms, run_with, Options, Session};

/// The interrupt-maintained state (see the module documentation).
const INTERRUPT_STATE: [std::ops::Range<usize>; 5] = [0xB103..0xB104, 0xB187..0xB18C, 0xB192..0xB193, 0xB1F0..0xB200, 0xB4E0..0xB560];

/// The recorded tape signal of each side must have the same edges (levels and number;
/// the pulse widths are only approximately equal, see the firmware's own tests).
fn same_tape(s: &Session, what: &str) {
    let (a, b) = (&s.rom.io.tape_out, &s.rust.cpc.hw.io.tape_out);
    assert_eq!(a.len(), b.len(), "{what}: number of tape edges");
    for (i, (p, q)) in a.iter().zip(b).enumerate() { assert_eq!(p.1, q.1, "{what}: level of edge {i}"); }
}

fn session(lines: &[&str], tape: Vec<(u64, bool)>) -> Option<Session> {
    let (os, basic) = roms()?;
    Some(run_with(&os, &basic, lines, &Options { motor: true, tape }))
}

/// Save with `save`, check, then load the ROM's recording back with `load`.
fn save_and_load(save: &[&str], load: &[&str]) {
    let Some(s) = session(save, vec![]) else { eprintln!("ROMs not set; skipping"); return };
    s.assert_same_except(&format!("{save:?}"), &INTERRUPT_STATE, false);
    same_tape(&s, "save");
    let tape = s.rom.io.tape_out.clone();
    let t = session(load, tape).unwrap();
    t.assert_same_except(&format!("{load:?}"), &INTERRUPT_STATE, false);
}

#[test]
fn save_and_load_program() {
    save_and_load(&["10 PRINT \"saved\":a=1", "20 GOTO 10", "SAVE \"!PROG\""], &["#TAPE", "LOAD \"!PROG\"", "LIST"]);
}

#[test]
fn save_ascii_and_load() {
    save_and_load(&["10 PRINT \"ascii\"", "20 REM x", "SAVE \"!ASC\",A"], &["#TAPE", "LOAD \"!ASC\"", "LIST"]);
}

#[test]
fn save_protected_and_run() {
    save_and_load(&["10 a=42", "SAVE \"!P\",P"], &["#TAPE", "RUN \"!P\"", "LIST", "PRINT a"]);
}

#[test]
fn save_binary_and_load_above_himem() {
    save_and_load(&["FOR i=0 TO 9:POKE &A000+i,i*3:NEXT", "SAVE \"!BIN\",B,&A000,10,&A005"],
                  &["MEMORY &8FFF", "#TAPE", "LOAD \"!BIN\",&9000", "PRINT PEEK(&9009)"]);
}

#[test]
fn load_binary_into_basic_memory_fails() {
    save_and_load(&["SAVE \"!BIN\",B,&A000,10"], &["#TAPE", "LOAD \"!BIN\",&1000"]);
}

#[test]
fn openout_print_and_input() {
    save_and_load(&["OPENOUT \"!DATA\":PRINT #9,12,\"text\";34:WRITE #9,\"q,r\",5:PRINT #9,\"line, with comma\":CLOSEOUT"],
                  &["10 OPENIN \"!DATA\":INPUT #9,a,b$,c,d$,e:LINE INPUT #9,f$:PRINT a;b$;c;d$;e;f$;EOF:CLOSEIN", "#TAPE", "RUN"]);
}

#[test]
fn merge_and_chain() {
    save_and_load(&["20 PRINT \"twenty\"", "30 b=b+1", "SAVE \"!M\""], &["10 PRINT \"ten\"", "30 a=1", "#TAPE", "MERGE \"!M\"", "LIST"]);
    save_and_load(&["20 PRINT \"twenty\";v", "30 v=v+1", "SAVE \"!C\""], &["10 v=5:DIM x(3):x(2)=7:s$=\"keep\":CHAIN \"!C\",20", "#TAPE", "RUN", "PRINT v;x(2);s$"]);
    save_and_load(&["20 PRINT \"twenty\";v", "SAVE \"!C\""], &["10 v=5:CHAIN MERGE \"!C\",20,DELETE 10", "#TAPE", "RUN", "LIST"]);
}

#[test]
fn openin_wrong_type() {
    save_and_load(&["10 REM", "SAVE \"!T\""], &["#TAPE", "OPENIN \"!T\""]);
}
