# cpc464-basic

A commented, readable **Rust reconstruction of Locomotive BASIC 1.0**, the Amstrad CPC464 upper ROM
(16 KiB `BASIC1-0.ROM`). Like [`cpc464-os`](../cpc-464-os/README.md) it is a hand-reconstructed *behavioural
port*, not a mechanical translation: each routine is a Rust function named after what it does and documented with
its ROM address, registers became parameters/return values, flags became `bool`/`Option`, and BASIC's workspace,
program, variables, strings and stack all live in RAM at their original addresses (`vars`), so a program that
`PEEK`s and `POKE`s BASIC's own state still works.

The interpreter runs *on top of* the Rust firmware, exactly as the ROM runs on top of the lower ROM: every
firmware call goes through the jumpblock wrappers in `fw`, which call the `cpc464-os` routine - or, if a program
has patched the jumpblock entry, the patched code through the `Hardware` trait.

See `src/lib.rs` for the ROM map, the module table and the modelling decisions (non-local exits as `Exit`, the
text pointer, banking, the Z80 stack).

    let mut basic = Basic::new(&os_rom, &basic_rom, my_hardware);
    basic.line_input = Some(Box::new(|_| LineInput::Line(b"PRINT 1+1".to_vec())));  // or leave None: the firmware editor
    basic.power_on_and_run();

## Testing against the real ROM

The tests boot the original OS and BASIC ROMs in a small Z80 emulator next to the Rust port, type the same lines
into both (the ROM's line editor `BD3A` is intercepted; the port is fed through `Basic::line_input`), and compare
all RAM and every hardware port write afterwards. You must supply both ROMs yourself:

    OS464_ROM=/path/to/OS464.ROM BASIC_ROM=/path/to/BASIC1-0.ROM cargo test --release

Without the variables the differential tests skip. A line `#TICKS n` lets `n` interrupts happen on both sides
while BASIC waits for input (for `AFTER`/`EVERY`/`SOUND`); `#TAPE` starts playing a recorded tape (cassette tests).
For digging into a difference, `tests/debug.rs` prints a RAM range of both machines after a session:

    LINES='a=5|PRINT a' FROM=AE40 TO=AE80 cargo test --release --test debug -- --nocapture

## What is verified

Byte-exact vs. the ROM (all RAM including the screen, plus port writes), in over 200 tests that together execute
about 95% of the ROM's instructions:

* cold start to `Ready`, the direct-mode loop, `AUTO`, `EDIT` (including the edit offered after a *Syntax error*),
  `NEW`, `CLEAR`, `TRON`/`TROFF`, `STOP`/`END`/`CONT`;
* the tokeniser and `LIST` (keywords, numbers in every form, strings, comments, variables, line references);
  storing, replacing and deleting lines, `DELETE`, `RENUM` (with undefined-line reports), line references
  rewritten as addresses and back;
* expressions: every operator and priority, integer overflow to real, soft *Overflow* / *Division by zero*,
  comparisons of numbers and strings; number parsing and formatting (`PRINT`, `STR$`, `PRINT USING` with every
  field type, `DEC$`, `HEX$`, `BIN$`, `ROUND`);
* variables, arrays (auto-dimensioned, multi-dimensional, `ERASE`), `DEFINT`/`DEFSTR`/`DEFREAL`, `DEF FN` with
  parameters, `@`;
* strings: every string function, `MID$` assignment, temporaries, string space full, garbage collection (also
  in the middle of an expression and inside `FN`), `MEMORY` moving the string space;
* `FOR`/`NEXT` (integer and real, nested, multi-variable `NEXT`), `WHILE`/`WEND`, `GOSUB`/`RETURN`, `ON`,
  `IF`/`THEN`/`ELSE` (nested), and their errors;
* errors and messages, `ON ERROR GOTO`, `RESUME` (all three forms), `ERR`/`ERL`, errors inside the handler;
* `INPUT` (prompts, `?Redo from start`, several items, quoted strings, arrays), `LINE INPUT`, `READ`/`DATA`/`RESTORE`;
* screen, graphics, sound (`SOUND`, `ENV`, `ENT`, `SQ`, `RELEASE`), keyboard (`KEY`, `KEY DEF`, `SPEED`), `PEEK`/`POKE`,
  `INP`/`OUT`, `CALL`, unknown RSX names, `SYMBOL`/`SYMBOL AFTER`, `MEMORY`;
* events with interrupts delivered: `AFTER`, `EVERY`, `REMAIN`, `DI`/`EI`, priorities, the restart of the interrupted
  statement after the event's `RETURN`, `ON SQ` arming;
* the cassette (motor running, `!` names): `SAVE` (tokenised, `,A`, `,P`, `,B`), `LOAD` (programs and binaries,
  *Memory full* inside BASIC's memory), `RUN "file"`, `MERGE`, `CHAIN` (with a start line, `CHAIN MERGE ... DELETE`,
  variables kept), `OPENOUT`/`PRINT #9`/`WRITE #9`/`CLOSEOUT`, `OPENIN`/`INPUT #9`/`LINE INPUT #9`/`EOF`/`CLOSEIN`,
  *File type error*. The recorded tape signal is compared edge by edge (levels and count), and what the ROM saved is
  played back into both sides.

## What is NOT verified / approximate

* **Not compared in RAM:** the Z80 stack page `BF00-BFFF`, the firmware's interrupt stack `B105-B186`, the operand of
  the RAM kernel's self-modifying `CALL` (`BA3F-BA40`), and `B55B` - `SOUND RESET` initialises the sound event block
  without loading `C`, so its ROM byte is whatever the caller had in `C` (a firmware quirk the Rust firmware does not
  model). The ROM side's gate-array ROM-banking and ROM-select writes are left out of the port comparison (the port
  has no banking to switch).
* **Maths error flags are reconstructed.** On failure the firmware's maths routines leave `A` (`FF`/`FE` overflow,
  `00` division by zero, `01` improper argument), which decides BASIC's error; the Rust firmware only reports
  success, so `fw::real_function` / `fw::real_power` rebuild the flags from the case (measured on the ROM).
* **Cassette tests** do not compare the interrupt-maintained state (clock, 1/50 s divider, keyboard state) nor the
  port writes: the ROM waits for the motor with interrupts enabled, and the emulated ROM gets none while the Rust
  firmware runs one per 10.8 ms of waiting.
* Not differentially tested: the printer (`PRINT #8`, `WIDTH` wrapping), ESC handling (the break pause,
  `ON BREAK GOSUB`, *Break*), `WAIT`, RSX commands that exist (needs a background ROM), `CALL` into real code (the
  port hands calls to `Hardware::far_call`), `CAT`, user-patched jumpblock entries or BASIC's RAM hooks
  (`AC01-AC1B`), and programs that switch ROMs with `OUT`.

## Quirks kept

* `FOR i=1 TO 3:FOR j=1 TO 0:NEXT j,i` never steps `i`: the "don't add the step" flag set by `FOR` is still clear
  for the rest of the `NEXT` list.
* `DEC$` checks for its `(` although the function dispatcher has already taken it, so BASIC 1.0 wants an
  unbalanced `DEC$((n,"##.#")`; both `DEC$(n,"##.#")` and `DEC$((n,"##.#"))` are *Syntax error*s.
* `READ` of the last item of `DATA 1,2:PRINT` is a *Syntax error* (the item must end at a comma or the end of the line).
* `KEY n,...` accepts any `n` below `&D4` (the intended limit goes into the wrong register); `KM SET EXPAND` decides.
* `PRINT`'s separator table claims five entries and holds four; the fifth is read from the `CALL CEFB` after it,
  so `SPEED` dispatches to the expression evaluator, on the `SPEED` token itself: a *Syntax error*, the same as
  without the phantom entry.
* The tokeniser keeps digits after a variable and a space as plain text, and `GOTO 0` stores the digit as text.

## Change to `cpc464-os` made for this port

* `BIN TO REAL` scaled by 2^-8 (`BD94`) takes a guard byte in `C`, which BASIC's number parser uses;
  `bin_to_real_a8_guard` honours it (`bin_to_real_a8` is unchanged: guard 0).
* `SOUND ARM EVENT`: when the channel has room, the ROM still stores the low byte of the event address; the Rust
  firmware now does too.
