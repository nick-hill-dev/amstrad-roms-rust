//! Differential tests: power on, BASIC's cold start, the `Ready` prompt.
//! Needs `OS464_ROM` and `BASIC_ROM` (paths to the ROM images).
mod support;
use support::session::check;

#[test]
fn cold_start_to_ready() { check(&[]); }

#[test]
fn an_empty_line() { check(&[""]); }
