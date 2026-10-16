//! A small, self-contained Z80 interpreter used to run the *original* ROM so the Rust
//! port can be compared with it.  It implements the whole documented instruction set
//! plus the common undocumented index-half-register forms, tracks T-states and the
//! `R` register, and talks to its machine through the [`Bus`] trait.
#![allow(dead_code)]

pub trait Bus {
    fn rd(&mut self, addr: u16) -> u8;
    fn wr(&mut self, addr: u16, v: u8);
    /// `IN`: `tstates` is the CPU clock at the moment of the access.
    fn input(&mut self, port: u16, tstates: u64) -> u8;
    fn output(&mut self, port: u16, v: u8, tstates: u64);
}

pub const FS: u8 = 0x80;
pub const FZ: u8 = 0x40;
pub const FY: u8 = 0x20;
pub const FH: u8 = 0x10;
pub const FX: u8 = 0x08;
pub const FP: u8 = 0x04;
pub const FN: u8 = 0x02;
pub const FC: u8 = 0x01;

#[derive(Clone, Default)]
pub struct Z80 {
    pub a: u8, pub f: u8, pub b: u8, pub c: u8, pub d: u8, pub e: u8, pub h: u8, pub l: u8,
    pub a2: u8, pub f2: u8, pub b2: u8, pub c2: u8, pub d2: u8, pub e2: u8, pub h2: u8, pub l2: u8,
    pub ix: u16, pub iy: u16, pub sp: u16, pub pc: u16,
    pub i: u8, pub r: u8,
    pub iff1: bool, pub iff2: bool, pub im: u8, pub halted: bool,
    pub tstates: u64,
    /// Index prefix in effect while decoding: 0 none, 0xDD or 0xFD.
    prefix: u8,
}

fn parity(v: u8) -> bool { v.count_ones() % 2 == 0 }

impl Z80 {
    pub fn new() -> Self { Z80 { sp: 0xFFFF, ..Default::default() } }

    // ---- register helpers --------------------------------------------------------
    pub fn af(&self) -> u16 { (self.a as u16) << 8 | self.f as u16 }
    pub fn bc(&self) -> u16 { (self.b as u16) << 8 | self.c as u16 }
    pub fn de(&self) -> u16 { (self.d as u16) << 8 | self.e as u16 }
    pub fn hl(&self) -> u16 { (self.h as u16) << 8 | self.l as u16 }
    pub fn set_af(&mut self, v: u16) { self.a = (v >> 8) as u8; self.f = v as u8; }
    pub fn set_bc(&mut self, v: u16) { self.b = (v >> 8) as u8; self.c = v as u8; }
    pub fn set_de(&mut self, v: u16) { self.d = (v >> 8) as u8; self.e = v as u8; }
    pub fn set_hl(&mut self, v: u16) { self.h = (v >> 8) as u8; self.l = v as u8; }

    fn idx(&self) -> u16 { if self.prefix == 0xDD { self.ix } else { self.iy } }
    fn set_idx(&mut self, v: u16) { if self.prefix == 0xDD { self.ix = v } else { self.iy = v } }
    /// HL, or IX/IY under a prefix.
    fn hl_x(&self) -> u16 { if self.prefix != 0 { self.idx() } else { self.hl() } }
    fn set_hl_x(&mut self, v: u16) { if self.prefix != 0 { self.set_idx(v) } else { self.set_hl(v) } }

    fn fetch<B: Bus>(&mut self, bus: &mut B) -> u8 {
        let v = bus.rd(self.pc);
        self.pc = self.pc.wrapping_add(1);
        v
    }
    fn fetch16<B: Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.fetch(bus) as u16;
        lo | (self.fetch(bus) as u16) << 8
    }
    /// Opcode fetch (M1 cycle): bumps R.
    fn fetch_op<B: Bus>(&mut self, bus: &mut B) -> u8 {
        self.r = (self.r & 0x80) | (self.r.wrapping_add(1) & 0x7F);
        self.fetch(bus)
    }
    fn rd16<B: Bus>(&mut self, bus: &mut B, a: u16) -> u16 {
        let lo = bus.rd(a) as u16;
        lo | (bus.rd(a.wrapping_add(1)) as u16) << 8
    }
    fn wr16<B: Bus>(&mut self, bus: &mut B, a: u16, v: u16) {
        bus.wr(a, v as u8);
        bus.wr(a.wrapping_add(1), (v >> 8) as u8);
    }
    fn push<B: Bus>(&mut self, bus: &mut B, v: u16) {
        self.sp = self.sp.wrapping_sub(1);
        bus.wr(self.sp, (v >> 8) as u8);
        self.sp = self.sp.wrapping_sub(1);
        bus.wr(self.sp, v as u8);
    }
    fn pop<B: Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = bus.rd(self.sp) as u16;
        self.sp = self.sp.wrapping_add(1);
        let hi = bus.rd(self.sp) as u16;
        self.sp = self.sp.wrapping_add(1);
        hi << 8 | lo
    }

    /// 8-bit register by code (0=B..5=L, 7=A); 4/5 are IXH/IXL under a prefix.
    fn reg(&self, n: u8) -> u8 {
        match n {
            0 => self.b, 1 => self.c, 2 => self.d, 3 => self.e,
            4 => if self.prefix != 0 { (self.idx() >> 8) as u8 } else { self.h },
            5 => if self.prefix != 0 { self.idx() as u8 } else { self.l },
            7 => self.a,
            _ => unreachable!(),
        }
    }
    fn set_reg(&mut self, n: u8, v: u8) {
        match n {
            0 => self.b = v, 1 => self.c = v, 2 => self.d = v, 3 => self.e = v,
            4 => if self.prefix != 0 { let x = self.idx() & 0xFF | (v as u16) << 8; self.set_idx(x) } else { self.h = v },
            5 => if self.prefix != 0 { let x = self.idx() & 0xFF00 | v as u16; self.set_idx(x) } else { self.l = v },
            7 => self.a = v,
            _ => unreachable!(),
        }
    }
    /// Plain (non-index) register access, used when the other operand is `(IX+d)`.
    fn reg_plain(&self, n: u8) -> u8 {
        match n { 4 => self.h, 5 => self.l, _ => self.reg(n) }
    }
    fn set_reg_plain(&mut self, n: u8, v: u8) {
        match n { 4 => self.h = v, 5 => self.l = v, _ => self.set_reg(n, v) }
    }
    /// 16-bit register pair by code: 0=BC 1=DE 2=HL(IX) 3=SP.
    fn rp(&self, n: u8) -> u16 {
        match n { 0 => self.bc(), 1 => self.de(), 2 => self.hl_x(), _ => self.sp }
    }
    fn set_rp(&mut self, n: u8, v: u16) {
        match n { 0 => self.set_bc(v), 1 => self.set_de(v), 2 => self.set_hl_x(v), _ => self.sp = v }
    }
    /// Pair code for PUSH/POP: 3 = AF.
    fn rp2(&self, n: u8) -> u16 { if n == 3 { self.af() } else { self.rp(n) } }
    fn set_rp2(&mut self, n: u8, v: u16) { if n == 3 { self.set_af(v) } else { self.set_rp(n, v) } }

    fn cond(&self, n: u8) -> bool {
        match n {
            0 => self.f & FZ == 0, 1 => self.f & FZ != 0,
            2 => self.f & FC == 0, 3 => self.f & FC != 0,
            4 => self.f & FP == 0, 5 => self.f & FP != 0,
            6 => self.f & FS == 0, _ => self.f & FS != 0,
        }
    }

    // ---- ALU -----------------------------------------------------------------------
    fn szxy(v: u8) -> u8 { (v & (FS | FY | FX)) | if v == 0 { FZ } else { 0 } }

    fn add8(&mut self, b: u8, carry: bool) -> u8 {
        let a = self.a;
        let r = a as u16 + b as u16 + carry as u16;
        let res = r as u8;
        let mut f = Self::szxy(res);
        if (a & 0xF) + (b & 0xF) + carry as u8 > 0xF { f |= FH; }
        if (a ^ b) & 0x80 == 0 && (a ^ res) & 0x80 != 0 { f |= FP; }
        if r > 0xFF { f |= FC; }
        self.f = f;
        res
    }
    fn sub8(&mut self, b: u8, carry: bool) -> u8 {
        let a = self.a;
        let r = a as i16 - b as i16 - carry as i16;
        let res = r as u8;
        let mut f = Self::szxy(res) | FN;
        if ((a & 0xF) as i16 - (b & 0xF) as i16 - carry as i16) < 0 { f |= FH; }
        if (a ^ b) & 0x80 != 0 && (a ^ res) & 0x80 != 0 { f |= FP; }
        if r < 0 { f |= FC; }
        self.f = f;
        res
    }
    fn alu(&mut self, op: u8, v: u8) {
        match op {
            0 => { let r = self.add8(v, false); self.a = r; }
            1 => { let c = self.f & FC != 0; let r = self.add8(v, c); self.a = r; }
            2 => { let r = self.sub8(v, false); self.a = r; }
            3 => { let c = self.f & FC != 0; let r = self.sub8(v, c); self.a = r; }
            4 => { self.a &= v; self.f = Self::szxy(self.a) | FH | if parity(self.a) { FP } else { 0 }; }
            5 => { self.a ^= v; self.f = Self::szxy(self.a) | if parity(self.a) { FP } else { 0 }; }
            6 => { self.a |= v; self.f = Self::szxy(self.a) | if parity(self.a) { FP } else { 0 }; }
            _ => {
                // CP: flags from A - v, but X/Y from the operand.
                let save = self.a;
                self.sub8(v, false);
                self.a = save;
                self.f = (self.f & !(FY | FX)) | (v & (FY | FX));
            }
        }
    }
    fn inc8(&mut self, v: u8) -> u8 {
        let r = v.wrapping_add(1);
        let mut f = (self.f & FC) | Self::szxy(r);
        if v & 0xF == 0xF { f |= FH; }
        if v == 0x7F { f |= FP; }
        self.f = f;
        r
    }
    fn dec8(&mut self, v: u8) -> u8 {
        let r = v.wrapping_sub(1);
        let mut f = (self.f & FC) | Self::szxy(r) | FN;
        if v & 0xF == 0 { f |= FH; }
        if v == 0x80 { f |= FP; }
        self.f = f;
        r
    }
    fn add16(&mut self, a: u16, b: u16) -> u16 {
        let r = a as u32 + b as u32;
        let res = r as u16;
        let mut f = self.f & (FS | FZ | FP);
        f |= ((res >> 8) as u8) & (FY | FX);
        if (a & 0xFFF) + (b & 0xFFF) > 0xFFF { f |= FH; }
        if r > 0xFFFF { f |= FC; }
        self.f = f;
        res
    }
    fn adc16(&mut self, a: u16, b: u16) -> u16 {
        let c = (self.f & FC) as u32;
        let r = a as u32 + b as u32 + c;
        let res = r as u16;
        let mut f = ((res >> 8) as u8) & (FS | FY | FX);
        if res == 0 { f |= FZ; }
        if (a & 0xFFF) as u32 + (b & 0xFFF) as u32 + c > 0xFFF { f |= FH; }
        if (a ^ b) & 0x8000 == 0 && (a ^ res) & 0x8000 != 0 { f |= FP; }
        if r > 0xFFFF { f |= FC; }
        self.f = f;
        res
    }
    fn sbc16(&mut self, a: u16, b: u16) -> u16 {
        let c = (self.f & FC) as i32;
        let r = a as i32 - b as i32 - c;
        let res = r as u16;
        let mut f = (((res >> 8) as u8) & (FS | FY | FX)) | FN;
        if res == 0 { f |= FZ; }
        if ((a & 0xFFF) as i32 - (b & 0xFFF) as i32 - c) < 0 { f |= FH; }
        if (a ^ b) & 0x8000 != 0 && (a ^ res) & 0x8000 != 0 { f |= FP; }
        if r < 0 { f |= FC; }
        self.f = f;
        res
    }

    // rotate / shift group (CB 00-3F)
    fn rot(&mut self, op: u8, v: u8) -> u8 {
        let c_in = self.f & FC != 0;
        let (r, c) = match op {
            0 => (v.rotate_left(1), v & 0x80 != 0),                 // RLC
            1 => (v.rotate_right(1), v & 1 != 0),                   // RRC
            2 => (v << 1 | c_in as u8, v & 0x80 != 0),              // RL
            3 => (v >> 1 | (c_in as u8) << 7, v & 1 != 0),          // RR
            4 => (v << 1, v & 0x80 != 0),                           // SLA
            5 => (v >> 1 | v & 0x80, v & 1 != 0),                   // SRA
            6 => (v << 1 | 1, v & 0x80 != 0),                       // SLL (undocumented)
            _ => (v >> 1, v & 1 != 0),                              // SRL
        };
        self.f = Self::szxy(r) | if parity(r) { FP } else { 0 } | c as u8;
        r
    }

    fn daa(&mut self) {
        let a = self.a;
        let mut adj = 0u8;
        let mut carry = self.f & FC != 0;
        if self.f & FH != 0 || (a & 0xF) > 9 { adj |= 0x06; }
        if carry || a > 0x99 { adj |= 0x60; carry = true; }
        let res = if self.f & FN != 0 { a.wrapping_sub(adj) } else { a.wrapping_add(adj) };
        let h = if self.f & FN != 0 { self.f & FH != 0 && (a & 0xF) < 6 } else { (a & 0xF) > 9 };
        self.a = res;
        self.f = Self::szxy(res) | if parity(res) { FP } else { 0 } | (self.f & FN) | carry as u8 | if h { FH } else { 0 };
    }

    /// Execute one instruction; returns the T-states it took.
    pub fn step<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let start = self.tstates;
        if self.halted {
            self.tstates += 4;
            self.r = (self.r & 0x80) | (self.r.wrapping_add(1) & 0x7F);
            return 4;
        }
        self.prefix = 0;
        let mut op = self.fetch_op(bus);
        let mut t: u32 = 0;
        while op == 0xDD || op == 0xFD {
            self.prefix = op;
            t += 4;
            op = self.fetch_op(bus);
        }
        t += self.exec(bus, op);
        self.tstates += t as u64;
        (self.tstates - start) as u32
    }

    /// Address of the memory operand for `(HL)` / `(IX+d)`; extra T-states for the index.
    fn mem_addr<B: Bus>(&mut self, bus: &mut B, t: &mut u32) -> u16 {
        if self.prefix != 0 {
            let d = self.fetch(bus) as i8;
            *t += 8;
            self.idx().wrapping_add(d as u16)
        } else {
            self.hl()
        }
    }

    fn exec<B: Bus>(&mut self, bus: &mut B, op: u8) -> u32 {
        let x = op >> 6;
        let y = (op >> 3) & 7;
        let z = op & 7;
        let p = y >> 1;
        let q = y & 1;
        let mut t: u32;
        match x {
            0 => match z {
                0 => match y {
                    0 => t = 4,
                    1 => { std::mem::swap(&mut self.a, &mut self.a2); std::mem::swap(&mut self.f, &mut self.f2); t = 4; }
                    2 => { // DJNZ
                        let d = self.fetch(bus) as i8;
                        self.b = self.b.wrapping_sub(1);
                        if self.b != 0 { self.pc = self.pc.wrapping_add(d as u16); t = 13 } else { t = 8 }
                    }
                    3 => { let d = self.fetch(bus) as i8; self.pc = self.pc.wrapping_add(d as u16); t = 12; }
                    _ => {
                        let d = self.fetch(bus) as i8;
                        if self.cond(y - 4) { self.pc = self.pc.wrapping_add(d as u16); t = 12 } else { t = 7 }
                    }
                },
                1 => if q == 0 {
                    let nn = self.fetch16(bus);
                    self.set_rp(p, nn);
                    t = 10;
                } else {
                    let r = self.rp(p);
                    let h = self.hl_x();
                    let v = self.add16(h, r);
                    self.set_hl_x(v);
                    t = 11;
                },
                2 => match (q, p) {
                    (0, 0) => { let a = self.bc(); bus.wr(a, self.a); t = 7; }
                    (0, 1) => { let a = self.de(); bus.wr(a, self.a); t = 7; }
                    (0, 2) => { let nn = self.fetch16(bus); let v = self.hl_x(); self.wr16(bus, nn, v); t = 16; }
                    (0, _) => { let nn = self.fetch16(bus); bus.wr(nn, self.a); t = 13; }
                    (1, 0) => { let a = self.bc(); self.a = bus.rd(a); t = 7; }
                    (1, 1) => { let a = self.de(); self.a = bus.rd(a); t = 7; }
                    (1, 2) => { let nn = self.fetch16(bus); let v = self.rd16(bus, nn); self.set_hl_x(v); t = 16; }
                    _ => { let nn = self.fetch16(bus); self.a = bus.rd(nn); t = 13; }
                },
                3 => {
                    let v = if q == 0 { self.rp(p).wrapping_add(1) } else { self.rp(p).wrapping_sub(1) };
                    self.set_rp(p, v);
                    t = 6;
                }
                4 | 5 => {
                    let inc = z == 4;
                    if y == 6 {
                        let mut tt = 0;
                        let a = self.mem_addr(bus, &mut tt);
                        let v = bus.rd(a);
                        let r = if inc { self.inc8(v) } else { self.dec8(v) };
                        bus.wr(a, r);
                        t = 11 + tt;
                        if self.prefix != 0 { t -= 0; }
                    } else {
                        let v = self.reg(y);
                        let r = if inc { self.inc8(v) } else { self.dec8(v) };
                        self.set_reg(y, r);
                        t = 4;
                    }
                }
                6 => {
                    if y == 6 {
                        let mut tt = 0;
                        let a = self.mem_addr(bus, &mut tt);
                        let n = self.fetch(bus);
                        bus.wr(a, n);
                        t = 10 + if self.prefix != 0 { 5 } else { 0 };
                        let _ = tt;
                    } else {
                        let n = self.fetch(bus);
                        self.set_reg(y, n);
                        t = 7;
                    }
                }
                _ => {
                    t = 4;
                    match y {
                        0 => { let a = self.a; self.a = a.rotate_left(1); self.f = (self.f & (FS | FZ | FP)) | (self.a & (FY | FX)) | (a >> 7); }
                        1 => { let a = self.a; self.a = a.rotate_right(1); self.f = (self.f & (FS | FZ | FP)) | (self.a & (FY | FX)) | (a & 1); }
                        2 => { let a = self.a; let c = self.f & FC; self.a = a << 1 | c; self.f = (self.f & (FS | FZ | FP)) | (self.a & (FY | FX)) | (a >> 7); }
                        3 => { let a = self.a; let c = self.f & FC; self.a = a >> 1 | c << 7; self.f = (self.f & (FS | FZ | FP)) | (self.a & (FY | FX)) | (a & 1); }
                        4 => self.daa(),
                        5 => { self.a = !self.a; self.f = (self.f & (FS | FZ | FP | FC)) | FH | FN | (self.a & (FY | FX)); }
                        6 => { self.f = (self.f & (FS | FZ | FP)) | FC | (self.a & (FY | FX)); }
                        _ => {
                            let c = self.f & FC;
                            self.f = (self.f & (FS | FZ | FP)) | (self.a & (FY | FX)) | if c != 0 { FH } else { 0 } | (c ^ 1);
                        }
                    }
                }
            },
            1 => {
                if op == 0x76 { self.halted = true; self.pc = self.pc.wrapping_sub(1); return 4; }
                if y == 6 {
                    let mut tt = 0;
                    let a = self.mem_addr(bus, &mut tt);
                    let v = self.reg_plain(z);
                    bus.wr(a, v);
                    t = 7 + tt;
                } else if z == 6 {
                    let mut tt = 0;
                    let a = self.mem_addr(bus, &mut tt);
                    let v = bus.rd(a);
                    self.set_reg_plain(y, v);
                    t = 7 + tt;
                } else {
                    let v = self.reg(z);
                    self.set_reg(y, v);
                    t = 4;
                }
            }
            2 => {
                if z == 6 {
                    let mut tt = 0;
                    let a = self.mem_addr(bus, &mut tt);
                    let v = bus.rd(a);
                    self.alu(y, v);
                    t = 7 + tt;
                } else {
                    let v = self.reg(z);
                    self.alu(y, v);
                    t = 4;
                }
            }
            _ => match z {
                0 => { // RET cc
                    if self.cond(y) { self.pc = self.pop(bus); t = 11 } else { t = 5 }
                }
                1 => if q == 0 {
                    let v = self.pop(bus);
                    self.set_rp2(p, v);
                    t = 10;
                } else {
                    match p {
                        0 => { self.pc = self.pop(bus); t = 10; }
                        1 => {
                            std::mem::swap(&mut self.b, &mut self.b2); std::mem::swap(&mut self.c, &mut self.c2);
                            std::mem::swap(&mut self.d, &mut self.d2); std::mem::swap(&mut self.e, &mut self.e2);
                            std::mem::swap(&mut self.h, &mut self.h2); std::mem::swap(&mut self.l, &mut self.l2);
                            t = 4;
                        }
                        2 => { self.pc = self.hl_x(); t = 4; }
                        _ => { self.sp = self.hl_x(); t = 6; }
                    }
                },
                2 => { let nn = self.fetch16(bus); if self.cond(y) { self.pc = nn; } t = 10; }
                3 => match y {
                    0 => { self.pc = self.fetch16(bus); t = 10; }
                    1 => { t = self.exec_cb(bus); }
                    2 => { let n = self.fetch(bus); let port = (self.a as u16) << 8 | n as u16; bus.output(port, self.a, self.tstates + 8); t = 11; }
                    3 => { let n = self.fetch(bus); let port = (self.a as u16) << 8 | n as u16; self.a = bus.input(port, self.tstates + 8); t = 11; }
                    4 => { // EX (SP),HL
                        let sp = self.sp;
                        let v = self.rd16(bus, sp);
                        let h = self.hl_x();
                        self.wr16(bus, sp, h);
                        self.set_hl_x(v);
                        t = 19;
                    }
                    5 => { let d = self.de(); let h = self.hl(); self.set_de(h); self.set_hl(d); t = 4; }
                    6 => { self.iff1 = false; self.iff2 = false; t = 4; }
                    _ => { self.iff1 = true; self.iff2 = true; t = 4; }
                },
                4 => {
                    let nn = self.fetch16(bus);
                    if self.cond(y) { let pc = self.pc; self.push(bus, pc); self.pc = nn; t = 17 } else { t = 10 }
                }
                5 => if q == 0 {
                    let v = self.rp2(p);
                    self.push(bus, v);
                    t = 11;
                } else {
                    match p {
                        0 => { let nn = self.fetch16(bus); let pc = self.pc; self.push(bus, pc); self.pc = nn; t = 17; }
                        2 => { t = self.exec_ed(bus); }
                        _ => panic!("bad prefix opcode {op:02X} at {:04X}", self.pc.wrapping_sub(1)),
                    }
                },
                6 => { let n = self.fetch(bus); self.alu(y, n); t = 7; }
                _ => { let pc = self.pc; self.push(bus, pc); self.pc = (y as u16) * 8; t = 11; }
            },
        }
        t
    }

    fn exec_cb<B: Bus>(&mut self, bus: &mut B) -> u32 {
        // With an index prefix the displacement comes *before* the final opcode byte and the
        // opcode fetch does not bump R.
        if self.prefix != 0 {
            let d = self.fetch(bus) as i8;
            let op = self.fetch(bus);
            let a = self.idx().wrapping_add(d as u16);
            let v = bus.rd(a);
            let x = op >> 6; let y = (op >> 3) & 7; let z = op & 7;
            let (res, write) = match x {
                0 => (self.rot(y, v), true),
                1 => { self.bit(y, v, (a >> 8) as u8); (v, false) }
                2 => (v & !(1 << y), true),
                _ => (v | (1 << y), true),
            };
            if write {
                bus.wr(a, res);
                if z != 6 { self.set_reg_plain(z, res); }
            }
            return if x == 1 { 16 } else { 19 };
        }
        let op = self.fetch_op(bus);
        let x = op >> 6; let y = (op >> 3) & 7; let z = op & 7;
        let v = if z == 6 { bus.rd(self.hl()) } else { self.reg(z) };
        let (res, write) = match x {
            0 => (self.rot(y, v), true),
            1 => { self.bit(y, v, if z == 6 { self.h } else { v }); (v, false) }
            2 => (v & !(1 << y), true),
            _ => (v | (1 << y), true),
        };
        if write {
            if z == 6 { let a = self.hl(); bus.wr(a, res); } else { self.set_reg(z, res); }
        }
        if z == 6 { if x == 1 { 12 } else { 15 } } else { 8 }
    }

    fn bit(&mut self, n: u8, v: u8, xy_src: u8) {
        let set = v & (1 << n) != 0;
        let mut f = (self.f & FC) | FH | (xy_src & (FY | FX));
        if !set { f |= FZ | FP; }
        if n == 7 && set { f |= FS; }
        self.f = f;
    }

    fn exec_ed<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let op = self.fetch_op(bus);
        let x = op >> 6; let y = (op >> 3) & 7; let z = op & 7; let p = y >> 1; let q = y & 1;
        let saved_prefix = self.prefix;
        self.prefix = 0; // DD/FD before ED are ignored
        let t = match (x, z) {
            (1, 0) => { // IN r,(C)
                let v = bus.input(self.bc(), self.tstates + 8);
                if y != 6 { self.set_reg(y, v); }
                self.f = (self.f & FC) | Self::szxy(v) | if parity(v) { FP } else { 0 };
                12
            }
            (1, 1) => { // OUT (C),r
                let v = if y == 6 { 0 } else { self.reg(y) };
                bus.output(self.bc(), v, self.tstates + 8);
                12
            }
            (1, 2) => {
                let hl = self.hl(); let r = self.rp(p);
                let v = if q == 0 { self.sbc16(hl, r) } else { self.adc16(hl, r) };
                self.set_hl(v);
                15
            }
            (1, 3) => {
                let nn = self.fetch16(bus);
                if q == 0 { let v = self.rp(p); self.wr16(bus, nn, v); } else { let v = self.rd16(bus, nn); self.set_rp(p, v); }
                20
            }
            (1, 4) => { let v = self.a; self.a = 0; let r = self.sub8(v, false); self.a = r; 8 }
            (1, 5) => { self.pc = self.pop(bus); self.iff1 = self.iff2; 14 }
            (1, 6) => { self.im = match y & 3 { 0 | 1 => 0, 2 => 1, _ => 2 }; 8 }
            (1, 7) => match y {
                0 => { self.i = self.a; 9 }
                1 => { self.r = self.a; 9 }
                2 => { self.a = self.i; self.f = (self.f & FC) | Self::szxy(self.a) | if self.iff2 { FP } else { 0 }; 9 }
                3 => { self.a = self.r; self.f = (self.f & FC) | Self::szxy(self.a) | if self.iff2 { FP } else { 0 }; 9 }
                4 => { // RRD
                    let a = self.hl(); let m = bus.rd(a);
                    let na = (self.a & 0xF0) | (m & 0x0F);
                    bus.wr(a, (self.a << 4) | (m >> 4));
                    self.a = na;
                    self.f = (self.f & FC) | Self::szxy(na) | if parity(na) { FP } else { 0 };
                    18
                }
                5 => { // RLD
                    let a = self.hl(); let m = bus.rd(a);
                    let na = (self.a & 0xF0) | (m >> 4);
                    bus.wr(a, (m << 4) | (self.a & 0x0F));
                    self.a = na;
                    self.f = (self.f & FC) | Self::szxy(na) | if parity(na) { FP } else { 0 };
                    18
                }
                _ => 8,
            },
            (2, _) if z <= 3 && y >= 4 => self.block(bus, y, z),
            _ => 8,
        };
        self.prefix = saved_prefix;
        t
    }

    fn block<B: Bus>(&mut self, bus: &mut B, y: u8, z: u8) -> u32 {
        let dir: i16 = if y & 1 == 0 { 1 } else { -1 };
        let repeat = y >= 6;
        match z {
            0 => { // LDI/LDD/LDIR/LDDR
                let v = bus.rd(self.hl());
                bus.wr(self.de(), v);
                self.set_hl(self.hl().wrapping_add(dir as u16));
                self.set_de(self.de().wrapping_add(dir as u16));
                self.set_bc(self.bc().wrapping_sub(1));
                let n = v.wrapping_add(self.a);
                self.f = (self.f & (FS | FZ | FC)) | (n & FX) | ((n << 4) & FY) | if self.bc() != 0 { FP } else { 0 };
                if repeat && self.bc() != 0 { self.pc = self.pc.wrapping_sub(2); 21 } else { 16 }
            }
            1 => { // CPI/CPD/CPIR/CPDR
                let v = bus.rd(self.hl());
                let c = self.f & FC;
                let save = self.a;
                self.sub8(v, false);
                self.a = save;
                self.set_hl(self.hl().wrapping_add(dir as u16));
                self.set_bc(self.bc().wrapping_sub(1));
                self.f = (self.f & !(FC | FP)) | c | if self.bc() != 0 { FP } else { 0 };
                if repeat && self.bc() != 0 && self.f & FZ == 0 { self.pc = self.pc.wrapping_sub(2); 21 } else { 16 }
            }
            2 => { // INI/IND/INIR/INDR
                let v = bus.input(self.bc(), self.tstates + 8);
                bus.wr(self.hl(), v);
                self.set_hl(self.hl().wrapping_add(dir as u16));
                self.b = self.b.wrapping_sub(1);
                self.f = (self.f & FC) | FN | Self::szxy(self.b);
                if repeat && self.b != 0 { self.pc = self.pc.wrapping_sub(2); 21 } else { 16 }
            }
            _ => { // OUTI/OUTD/OTIR/OTDR
                let v = bus.rd(self.hl());
                self.b = self.b.wrapping_sub(1);
                bus.output(self.bc(), v, self.tstates + 8);
                self.set_hl(self.hl().wrapping_add(dir as u16));
                self.f = (self.f & FC) | FN | Self::szxy(self.b);
                if repeat && self.b != 0 { self.pc = self.pc.wrapping_sub(2); 21 } else { 16 }
            }
        }
    }

    /// Maskable interrupt (IM 1): pushes PC and jumps to `0038`.  Returns T-states.
    pub fn interrupt<B: Bus>(&mut self, bus: &mut B) -> u32 {
        self.halted = false;
        self.iff1 = false;
        self.iff2 = false;
        let pc = self.pc;
        self.push(bus, pc);
        self.pc = 0x38;
        self.r = (self.r & 0x80) | (self.r.wrapping_add(1) & 0x7F);
        self.tstates += 13;
        13
    }
}
