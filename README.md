# amstrad-roms-rust

Readable, commented Rust reconstructions of the Amstrad CPC464's ROMs, each tested byte for byte against the
original running in a Z80 emulator:

* [cpc-464-os](cpc-464-os/README.md) - the lower ROM, firmware v1.0 (`OS464.ROM`).
* [cpc-464-basic](cpc-464-basic/README.md) - the upper ROM, Locomotive BASIC 1.0 (`BASIC1-0.ROM`), running on
  top of the firmware port.

The ROM images are not included; the tests read them from `OS464_ROM` and `BASIC_ROM`.
