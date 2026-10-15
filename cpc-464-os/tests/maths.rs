//! Differential tests of the arithmetic package against the original ROM.
mod support;
use cpc464_os::maths::{self, Flags};
use cpc464_os::Regs;
use support::dual::*;

fn flags(af: u16) -> Flags { Flags { carry: af & 1 != 0, zero: af & 0x40 != 0, sign: af & 0x80 != 0 } }

fn special16(rng: &mut Rng) -> u16 {
    match rng.below(12) {
        0 => 0, 1 => 1, 2 => 2, 3 => 0x7FFF, 4 => 0x8000, 5 => 0xFFFF, 6 => 0x00FF, 7 => 0x0100,
        8 => rng.byte() as u16, 9 => (rng.byte() as u16) << 8, _ => rng.word(),
    }
}

#[test]
fn integer_routines() {
    let Some(rom) = rom_path() else { return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0xC0FFEE);
    for i in 0..4000 {
        let (hl, de) = (special16(&mut rng), special16(&mut rng));
        let b = rng.byte();
        let ctx = format!("#{i} hl={hl:04X} de={de:04X} b={b:02X}");
        let regs = Regs { hl, de, bc: (b as u16) << 8, af: (b as u16) << 8, ..Default::default() };

        // BDA3: abs + remember sign
        let r = d.rom(0x3708, regs);
        let (m, bb) = maths::int_abs_split(hl);
        assert_eq!((r.hl, (r.bc >> 8) as u8), (m, bb), "BDA3 {ctx}");

        // BDA9: apply sign (B in the high byte of BC)
        let r = d.rom(0x3715, regs);
        let (v, ok, a) = maths::int_apply_sign(hl, b);
        assert_eq!(r.hl, v, "BDA9 hl {ctx}");
        assert_eq!(r.af & 1 != 0, ok, "BDA9 carry {ctx}");
        if !ok || true { assert_eq!(r.a(), a, "BDA9 a {ctx}"); }

        for (name, addr, f) in [
            ("add", 0x3728u16, maths::int_add as fn(u16, u16) -> (u16, bool)),
            ("sub", 0x3731, maths::int_sub),
            ("rsub", 0x3730, maths::int_sub_reverse),
            ("mul", 0x3739, maths::int_mul),
            ("mulu", 0x3750, maths::int_mul_unsigned),
        ] {
            let r = d.rom(addr, regs);
            let (v, ok) = f(hl, de);
            let carry = r.af & 1 != 0;
            // BDBE reports overflow with carry SET; the Rust function returns "overflow" there.
            let rust_carry = if name == "mulu" { ok } else { ok };
            assert_eq!(carry, rust_carry, "{name} carry {ctx}");
            assert_eq!(r.hl, v, "{name} hl {ctx}");
        }

        // division / modulus
        for (name, addr, f) in [
            ("div", 0x377Au16, maths::int_div as fn(u16, u16) -> Option<(u16, bool)>),
            ("mod", 0x3781, maths::int_mod),
        ] {
            let r = d.rom(addr, regs);
            match f(hl, de) {
                None => assert!(r.af & 1 == 0 && r.af & 0x40 != 0, "{name} by zero {ctx}"),
                Some((v, ok)) => {
                    assert_eq!(r.af & 1 != 0, ok, "{name} carry {ctx}");
                    assert_eq!(r.hl, v, "{name} hl {ctx}");
                }
            }
        }
        // unsigned divide
        let r = d.rom(0x378C, regs);
        match maths::int_div_unsigned_checked(hl, de) {
            None => assert!(r.af & 0x40 != 0 && r.af & 1 == 0, "divu zero {ctx}"),
            Some((q, rem)) => { assert_eq!((r.hl, r.de), (q, rem), "divu {ctx}"); assert!(r.af & 1 != 0); }
        }
        // negate
        let r = d.rom(0x37D4, regs);
        let (v, ok) = maths::int_negate(hl);
        assert_eq!((r.hl, r.af & 1 != 0), (v, ok), "neg {ctx}");
        // sign
        let r = d.rom(0x37E0, regs);
        let (a, f) = maths::int_sign(hl);
        assert_eq!((r.a(), flags(r.af).carry, flags(r.af).zero), (a, f.carry, f.zero), "sign {ctx}");
        // compare
        let r = d.rom(0x37E9, regs);
        let (a, f) = maths::int_compare(hl, de);
        assert_eq!((r.a(), flags(r.af).carry, flags(r.af).zero), (a, f.carry, f.zero), "cmp {ctx}");
    }
}

// ---------------------------------------------------------------------------------------
// Floating-point core
// ---------------------------------------------------------------------------------------

const A: u16 = 0x9000;
const B: u16 = 0x9010;

fn rand_exp(rng: &mut Rng) -> u8 {
    const SPECIAL: [u8; 14] = [0, 1, 2, 0x7F, 0x80, 0x81, 0x82, 0x84, 0x90, 0xA0, 0xA1, 0xFE, 0xFF, 0x20];
    if rng.below(3) == 0 { SPECIAL[rng.below(SPECIAL.len() as u32) as usize] } else { 0x70 + (rng.byte() % 0x40) }
}

fn rand_real(rng: &mut Rng) -> [u8; 5] {
    let mut r = [rng.byte(), rng.byte(), rng.byte(), rng.byte(), rand_exp(rng)];
    match rng.below(6) {
        0 => { r[0] = 0; r[1] = 0; r[2] = 0; r[3] &= 0x80; }            // exact powers of two
        1 => { r[0] = 0xFF; r[1] = 0xFF; r[2] = 0xFF; r[3] |= 0x7F; }  // all ones (rounding carries)
        2 => { r[0] = 0; r[1] = 0; }
        _ => {}
    }
    r
}

fn rand_pair(rng: &mut Rng) -> ([u8; 5], [u8; 5]) {
    let a = rand_real(rng);
    let mut b = rand_real(rng);
    match rng.below(5) {
        0 => { b[4] = a[4].wrapping_add(rng.below(5) as u8).wrapping_sub(2); }     // near exponent
        1 => { b = a; if rng.below(2) == 0 { b[3] ^= 0x80; } if rng.below(2) == 0 { b[0] ^= rng.byte() & 3; } }
        2 => { b[4] = a[4].wrapping_add(rng.below(40) as u8).wrapping_sub(20); }
        _ => {}
    }
    (a, b)
}

fn setup(d: &mut Dual, a: &[u8; 5], b: &[u8; 5]) {
    d.poke_bytes(A, a);
    d.poke_bytes(B, b);
}

#[test]
fn real_binary_operations() {
    let Some(rom) = rom_path() else { return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0xFACE_0FF);
    let ops: [(&str, u16); 5] = [("add", 0x333F), ("sub", 0x3337), ("rsub", 0x333B), ("mul", 0x3415), ("div", 0x349E)];
    let n: u32 = std::env::var("MATHS_N").ok().and_then(|s| s.parse().ok()).unwrap_or(3000);
    for i in 0..n {
        let (a, b) = rand_pair(&mut rng);
        for (name, addr) in ops {
            setup(&mut d, &a, &b);
            let ctx = format!("{name} #{i} a={a:02X?} b={b:02X?}");
            let r = d.rom(addr, Regs { hl: A, de: B, ..Default::default() });
            let (carry, zero) = match name {
                "add" => (d.cpc.real_add(A, B), None),
                "sub" => (d.cpc.real_sub(A, B), None),
                "rsub" => (d.cpc.real_sub_reverse(A, B), None),
                "mul" => (d.cpc.real_mul(A, B), None),
                _ => { let f = d.cpc.real_div(A, B); (f.carry, Some(f.zero)) }
            };
            assert_eq!(r.af & 1 != 0, carry, "{name} carry {ctx}");
            if let Some(z) = zero { if !carry { assert_eq!(r.af & 0x40 != 0, z, "{name} zero {ctx}"); } }
            d.assert_same(&ctx);
        }
    }
}

#[test]
fn real_unary_operations() {
    let Some(rom) = rom_path() else { return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0xBEEF_1234);
    let n: u32 = std::env::var("MATHS_N").ok().and_then(|s| s.parse().ok()).unwrap_or(3000);
    for i in 0..n {
        let (a, b) = rand_pair(&mut rng);
        let ctx = format!("#{i} a={a:02X?} b={b:02X?}");

        // compare (does not modify memory)
        setup(&mut d, &a, &b);
        let r = d.rom(0x359A, Regs { hl: A, de: B, ..Default::default() });
        let (ra, rf) = d.cpc.real_compare(A, B);
        assert_eq!((r.a(), flags(r.af)), (ra, rf), "compare {ctx}");
        d.assert_same(&format!("compare {ctx}"));

        // sign
        let r = d.rom(0x35E8, Regs { hl: A, ..Default::default() });
        let (ra, rf) = d.cpc.real_sign(A);
        assert_eq!((r.a(), flags(r.af)), (ra, rf), "sign {ctx}");

        // negate
        d.rom(0x35F8, Regs { hl: A, ..Default::default() });
        d.cpc.real_negate(A);
        d.assert_same(&format!("negate {ctx}"));

        // copy
        setup(&mut d, &a, &b);
        let r = d.rom(0x2E18, Regs { hl: A, de: B, ..Default::default() });
        let ra = d.cpc.real_copy(A, B);
        assert_eq!((r.a(), r.af & 1 != 0), (ra, true), "copy {ctx}");
        d.assert_same(&format!("copy {ctx}"));

        // multiply by 2^n (not for n that triggers the compare-fallthrough flags: carry only)
        setup(&mut d, &a, &b);
        let n2 = rng.byte();
        let r = d.rom(0x3578, Regs { hl: A, af: (n2 as u16) << 8, de: B, ..Default::default() });
        let c = d.cpc.real_mul_pow2(A, n2);
        // On overflow the ROM falls through into the compare routine, so the flags are junk.
        let overflowed = n2 & 0x80 == 0 && a[4].checked_add(n2).is_none();
        if !overflowed { assert_eq!(r.af & 1 != 0, c, "mul2n carry {ctx} n={n2:02X}"); }
        d.assert_same(&format!("mul2n {ctx} n={n2:02X}"));

        // real -> int
        setup(&mut d, &a, &b);
        let r = d.rom(0x2E66, Regs { hl: A, ..Default::default() });
        let (hl, f) = d.cpc.real_to_int(A);
        assert_eq!(flags(r.af).carry, f.carry, "r2i carry {ctx}");
        if f.carry { assert_eq!((r.hl, flags(r.af)), (hl, f), "r2i {ctx}"); }
        d.assert_same(&format!("r2i {ctx}"));

        // real -> binary (fix / round / floor)
        for (name, addr) in [("fix", 0x2EA1u16), ("bin", 0x2E8E), ("int", 0x2EAC)] {
            setup(&mut d, &a, &b);
            let r = d.rom(addr, Regs { hl: A, ..Default::default() });
            let (fr, f) = match name {
                "fix" => d.cpc.real_fix(A),
                "bin" => d.cpc.real_to_bin(A),
                _ => d.cpc.real_int(A),
            };
            assert_eq!(flags(r.af).carry, f.carry, "{name} carry {ctx}");
            if f.carry { assert_eq!(((r.bc >> 8) as u8 & 0x80, r.bc as u8), (fr.b & 0x80, fr.c), "{name} bc {ctx}"); }
            d.assert_same(&format!("{name} {ctx}"));
        }

        // int -> real
        let v = special16(&mut rng);
        let sg = rng.byte();
        let r = d.rom(0x2E29, Regs { hl: v, de: A, af: (sg as u16) << 8, ..Default::default() });
        let h = d.cpc.int_to_real(v, A, sg);
        assert_eq!(r.hl, h, "i2r hl {ctx} v={v:04X}");
        d.assert_same(&format!("i2r {ctx} v={v:04X} sg={sg:02X}"));

        // binary -> real
        setup(&mut d, &a, &b);
        let sg = rng.byte();
        let r = d.rom(0x2E55, Regs { hl: A, af: (sg as u16) << 8, ..Default::default() });
        let h = d.cpc.bin_to_real(A, sg);
        assert_eq!(r.hl, h, "b2r hl {ctx}");
        d.assert_same(&format!("b2r {ctx} sg={sg:02X}"));
        setup(&mut d, &a, &b);
        let r = d.rom(0x2E5E, Regs { hl: A, af: (sg as u16) << 8, ..Default::default() });
        let h = d.cpc.bin_to_real_a8(A, sg);
        assert_eq!(r.hl, h, "b2r8 hl {ctx}");
        d.assert_same(&format!("b2r8 {ctx} sg={sg:02X}"));
    }
}

// ---------------------------------------------------------------------------------------
// Transcendental functions
// ---------------------------------------------------------------------------------------

fn moderate_real(rng: &mut Rng) -> [u8; 5] {
    let mut r = rand_real(rng);
    if rng.below(4) != 0 { r[4] = 0x72 + (rng.byte() % 0x1E); }      // roughly 2^-15 .. 2^13
    r
}

#[test]
fn transcendental_functions() {
    let Some(rom) = rom_path() else { return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0x7A55_0001);
    let n: u32 = std::env::var("MATHS_N").ok().and_then(|s| s.parse().ok()).unwrap_or(400);
    type F = fn(&mut cpc464_os::Cpc<support::machine::TestHw>, u16) -> bool;
    let unary: [(&str, u16, F); 8] = [
        ("ln", 0x3014, |c, h| c.real_ln(h)),
        ("log10", 0x300F, |c, h| c.real_log10(h)),
        ("exp", 0x3090, |c, h| c.real_exp(h)),
        ("sqr", 0x310A, |c, h| c.real_sqrt(h)),
        ("sin", 0x31BC, |c, h| c.real_sin(h)),
        ("cos", 0x31B2, |c, h| c.real_cos(h)),
        ("tan", 0x3231, |c, h| c.real_tan(h)),
        ("atn", 0x3241, |c, h| c.real_atn(h)),
    ];
    for i in 0..n {
        let a = moderate_real(&mut rng);
        let b = moderate_real(&mut rng);
        let degrees = (rng.below(3) == 0) as u8;
        d.rom(0x31AE, Regs { af: (degrees as u16) << 8, ..Default::default() });
        d.cpc.real_set_angle_mode(degrees);
        for (name, addr, f) in unary {
            setup(&mut d, &a, &b);
            let ctx = format!("{name} #{i} deg={degrees} a={a:02X?}");
            let r = d.rom(addr, Regs { hl: A, de: B, ..Default::default() });
            let c = f(&mut d.cpc, A);
            assert_eq!(r.af & 1 != 0, c, "{name} carry {ctx}");
            d.assert_same(&ctx);
        }
        // x ^ y
        setup(&mut d, &a, &b);
        let ctx = format!("power #{i} a={a:02X?} b={b:02X?}");
        let r = d.rom(0x310D, Regs { hl: A, de: B, ..Default::default() });
        let c = d.cpc.real_power(A, B);
        assert_eq!(r.af & 1 != 0, c, "power carry {ctx}");
        d.assert_same(&ctx);
        // small integer exponents
        let mut y = [0u8; 5];
        let k = (rng.below(45) as u16) as u16;
        d.cpc.int_to_real(k, B, if rng.below(2) == 0 { 0x80 } else { 0 });
        d.rom(0x2E29, Regs { hl: k, de: B, af: (d.cpc.rd8(B + 3) as u16 & 0x80) << 8, ..Default::default() });
        for j in 0..5 { y[j] = d.cpc.rd8(B + j as u16); }
        d.poke_bytes(A, &a);
        let ctx = format!("intpower #{i} a={a:02X?} y={y:02X?}");
        let r = d.rom(0x310D, Regs { hl: A, de: B, ..Default::default() });
        let c = d.cpc.real_power(A, B);
        assert_eq!(r.af & 1 != 0, c, "intpower carry {ctx}");
        d.assert_same(&ctx);

        // powers of ten and decimal preparation
        setup(&mut d, &a, &b);
        let p = rng.byte();
        let ctx = format!("pow10 #{i} a={a:02X?} p={p:02X}");
        let r = d.rom(0x2F1D, Regs { hl: A, af: (p as u16) << 8, ..Default::default() });
        let c = d.cpc.real_pow10(A, p);
        assert_eq!(r.af & 1 != 0, c, "pow10 carry {ctx}");
        d.assert_same(&ctx);
        setup(&mut d, &a, &b);
        let ctx = format!("decprep #{i} a={a:02X?}");
        let r = d.rom(0x2EB6, Regs { hl: A, de: 0x1234, ..Default::default() });
        let dp = d.cpc.real_decimal_prepare(A);
        assert_eq!((r.hl, (r.bc >> 8) as u8, r.bc as u8, r.de as u8), (dp.hl, dp.b, dp.c, dp.e), "decprep regs {ctx}");
        d.assert_same(&ctx);
    }
}

#[test]
fn random_numbers_and_constants() {
    let Some(rom) = rom_path() else { return };
    let mut d = Dual::boot(&rom);
    let mut rng = Rng(0x1357_9BDF);
    d.rom(0x2F94, Regs::default());
    d.cpc.random_reset();
    d.assert_same("reset seed");
    for i in 0..300 {
        let (a, b) = rand_pair(&mut rng);
        setup(&mut d, &a, &b);
        let r = d.rom(0x2FB7, Regs { hl: A, ..Default::default() });
        let h = d.cpc.random_new(A);
        assert_eq!(r.hl, h);
        d.assert_same(&format!("next #{i}"));
        let r = d.rom(0x2FE6, Regs { hl: A, ..Default::default() });
        let h = d.cpc.random_to_real(A);
        assert_eq!(r.hl, h);
        d.assert_same(&format!("to_real #{i}"));
        if i % 7 == 0 {
            setup(&mut d, &a, &b);
            d.rom(0x2FA1, Regs { hl: A, ..Default::default() });
            d.cpc.random_seed_from_real(A);
            d.assert_same(&format!("seed_from_real #{i} a={a:02X?}"));
        }
    }
    d.rom(0x31A3, Regs { hl: A, ..Default::default() });
    d.cpc.real_pi(A);
    d.assert_same("pi");
}
