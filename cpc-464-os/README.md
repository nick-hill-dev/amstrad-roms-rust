# cpc464-os

A commented, readable **Rust reconstruction of the Amstrad CPC464 lower ROM** (firmware v1.0, 16 KiB `OS464.ROM`).
It is a hand-reconstructed *behavioural port*, not a mechanical translation: each routine is a Rust function,
registers became parameters/return values, flags became `bool`/`Option`, firmware variables keep their original
RAM addresses (`sysvars`), and hardware goes through the `Hardware` trait. Every function names its ROM address.

See `src/lib.rs` for the ROM map and module table.

## Testing against the real ROM

The tests run the original ROM in a small Z80 emulator next to the Rust port from the same state, then compare
all RAM (except the stack page and the ROM's private interrupt stack) and every hardware port write.
You must supply the ROM yourself:

    OS464_ROM=/path/to/OS464.ROM cargo test

Without `OS464_ROM` the differential tests skip. Trial counts: `MATHS_N EDITOR_N SCREEN_N GRAPHICS_N INTERRUPTS_N EVENTS_N MACHINE_N`.

## What is verified

Byte-exact vs. the ROM (RAM + port writes), on random and edge-case inputs:
boot/cold start, text VDU output and char reading, sound manager, the whole maths package (integer, real,
transcendentals, including ROM quirks), line editor, screen pack, graphics VDU, keyboard scan/queues/expansions,
clock and interrupt handling, kernel events/tickers/sync queue, machine pack, RAM-resident ROM-banking routines.

## What is NOT verified / approximate

* **Cassette pulse timing is approximate.** The ROM times pulses with the Z80 `R` register and path-dependent
  loops. The Rust port produces the same edges and levels (same count) with single pulses within ~160 T-states,
  and tapes round-trip in both directions (ROM writes -> Rust reads, Rust writes -> ROM reads).
* Not differentially tested: `kl_find_command`, `kl_rom_walk`, `kl_init_back`, `kl_log_ext`, `kl_choke_off`,
  far-call / low-jump (`rst_*`) paths, cassette file-level routines (open/in_char/out_char/catalog/direct), `cas_check`,
  GRA/TXT paths with user-patched indirections.
