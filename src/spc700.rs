//! SPC700 (APU 内蔵 CPU) コア。
//!
//! `step` は 1 命令実行して消費サイクル数 (SPC700 クロック @1.024MHz) を返す。

pub trait SpcBus {
    fn read(&mut self, addr: u16) -> u8;
    fn write(&mut self, addr: u16, v: u8);
}

pub const F_N: u8 = 0x80;
pub const F_V: u8 = 0x40;
pub const F_P: u8 = 0x20;
pub const F_B: u8 = 0x10;
pub const F_H: u8 = 0x08;
pub const F_I: u8 = 0x04;
pub const F_Z: u8 = 0x02;
pub const F_C: u8 = 0x01;

/// 基本サイクル数 (分岐成立時は +2)
#[rustfmt::skip]
const CYCLES: [u32; 256] = [
    2,8,4,5,3,4,3,6,2,6,5,4,5,4,6,8, // 0x
    2,8,4,5,4,5,5,6,5,5,6,5,2,2,4,6, // 1x
    2,8,4,5,3,4,3,6,2,6,5,4,5,4,5,2, // 2x (2F BRA は分岐成立 +2 で計 4)
    2,8,4,5,4,5,5,6,5,5,6,5,2,2,3,8, // 3x
    2,8,4,5,3,4,3,6,2,6,4,4,5,4,6,6, // 4x
    2,8,4,5,4,5,5,6,5,5,4,5,2,2,4,3, // 5x
    2,8,4,5,3,4,3,6,2,6,4,4,5,4,5,5, // 6x
    2,8,4,5,4,5,5,6,5,5,5,5,2,2,3,6, // 7x
    2,8,4,5,3,4,3,6,2,6,5,4,5,2,4,5, // 8x
    2,8,4,5,4,5,5,6,5,5,5,5,2,2,12,5,// 9x
    3,8,4,5,3,4,3,6,2,6,4,4,5,2,4,4, // Ax
    2,8,4,5,4,5,5,6,5,5,5,5,2,2,3,4, // Bx
    3,8,4,5,4,5,4,7,2,5,6,4,5,2,4,9, // Cx
    2,8,4,5,5,6,6,7,4,5,5,5,2,2,6,3, // Dx
    2,8,4,5,3,4,3,6,2,4,5,3,4,3,4,7, // Ex (EF SLEEP)
    2,8,4,5,4,5,5,6,3,4,5,4,2,2,4,7, // Fx (FF STOP)
];

pub struct Spc700 {
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub sp: u8,
    pub pc: u16,
    pub psw: u8,
    pub stopped: bool,
    extra: u32, // 分岐成立などの追加サイクル
}

impl Default for Spc700 {
    fn default() -> Self {
        Spc700 {
            a: 0,
            x: 0,
            y: 0,
            sp: 0xEF,
            pc: 0xFFC0,
            psw: 0,
            stopped: false,
            extra: 0,
        }
    }
}

impl Spc700 {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset<B: SpcBus>(&mut self, bus: &mut B) {
        *self = Self::default();
        let lo = bus.read(0xFFFE) as u16;
        let hi = bus.read(0xFFFF) as u16;
        self.pc = (hi << 8) | lo;
    }

    #[inline]
    fn flag(&self, f: u8) -> bool {
        self.psw & f != 0
    }

    #[inline]
    fn set_flag(&mut self, f: u8, v: bool) {
        if v {
            self.psw |= f;
        } else {
            self.psw &= !f;
        }
    }

    fn set_nz(&mut self, v: u8) -> u8 {
        self.set_flag(F_N, v & 0x80 != 0);
        self.set_flag(F_Z, v == 0);
        v
    }

    fn set_nz16(&mut self, v: u16) -> u16 {
        self.set_flag(F_N, v & 0x8000 != 0);
        self.set_flag(F_Z, v == 0);
        v
    }

    fn fetch<B: SpcBus>(&mut self, bus: &mut B) -> u8 {
        let v = bus.read(self.pc);
        self.pc = self.pc.wrapping_add(1);
        v
    }

    fn fetch16<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.fetch(bus) as u16;
        let hi = self.fetch(bus) as u16;
        (hi << 8) | lo
    }

    #[inline]
    fn dp_base(&self) -> u16 {
        if self.flag(F_P) {
            0x0100
        } else {
            0x0000
        }
    }

    /// dp オペランドのアドレス
    fn dp<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        let off = self.fetch(bus);
        self.dp_base() | off as u16
    }

    fn dp_x<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        let off = self.fetch(bus).wrapping_add(self.x);
        self.dp_base() | off as u16
    }

    fn dp_y<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        let off = self.fetch(bus).wrapping_add(self.y);
        self.dp_base() | off as u16
    }

    /// [dp+X]
    fn ind_dp_x<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        let p = self.dp_x(bus);
        let lo = bus.read(p) as u16;
        let hi = bus.read((p & 0xFF00) | (p as u8).wrapping_add(1) as u16) as u16;
        (hi << 8) | lo
    }

    /// [dp]+Y
    fn ind_dp_y<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        let p = self.dp(bus);
        let lo = bus.read(p) as u16;
        let hi = bus.read((p & 0xFF00) | (p as u8).wrapping_add(1) as u16) as u16;
        ((hi << 8) | lo).wrapping_add(self.y as u16)
    }

    fn abs<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        self.fetch16(bus)
    }

    fn abs_x<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        self.fetch16(bus).wrapping_add(self.x as u16)
    }

    fn abs_y<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        self.fetch16(bus).wrapping_add(self.y as u16)
    }

    /// (X) のアドレス
    fn at_x(&self) -> u16 {
        self.dp_base() | self.x as u16
    }

    fn at_y(&self) -> u16 {
        self.dp_base() | self.y as u16
    }

    // ---- スタック ----------------------------------------------------------

    fn push<B: SpcBus>(&mut self, bus: &mut B, v: u8) {
        bus.write(0x0100 | self.sp as u16, v);
        self.sp = self.sp.wrapping_sub(1);
    }

    fn pull<B: SpcBus>(&mut self, bus: &mut B) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        bus.read(0x0100 | self.sp as u16)
    }

    fn push16<B: SpcBus>(&mut self, bus: &mut B, v: u16) {
        self.push(bus, (v >> 8) as u8);
        self.push(bus, v as u8);
    }

    fn pull16<B: SpcBus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.pull(bus) as u16;
        let hi = self.pull(bus) as u16;
        (hi << 8) | lo
    }

    // ---- ALU ---------------------------------------------------------------

    fn adc8(&mut self, a: u8, b: u8) -> u8 {
        let c = (self.psw & F_C) as u16;
        let r = a as u16 + b as u16 + c;
        self.set_flag(F_C, r > 0xFF);
        self.set_flag(F_H, (a & 0x0F) as u16 + (b & 0x0F) as u16 + c > 0x0F);
        self.set_flag(F_V, !(a ^ b) & (a ^ r as u8) & 0x80 != 0);
        self.set_nz(r as u8)
    }

    fn sbc8(&mut self, a: u8, b: u8) -> u8 {
        self.adc8(a, !b)
    }

    fn cmp8(&mut self, a: u8, b: u8) {
        let r = a.wrapping_sub(b);
        self.set_flag(F_C, a >= b);
        self.set_nz(r);
    }

    fn or8(&mut self, a: u8, b: u8) -> u8 {
        self.set_nz(a | b)
    }

    fn and8(&mut self, a: u8, b: u8) -> u8 {
        self.set_nz(a & b)
    }

    fn eor8(&mut self, a: u8, b: u8) -> u8 {
        self.set_nz(a ^ b)
    }

    fn asl8(&mut self, v: u8) -> u8 {
        self.set_flag(F_C, v & 0x80 != 0);
        self.set_nz(v << 1)
    }

    fn lsr8(&mut self, v: u8) -> u8 {
        self.set_flag(F_C, v & 1 != 0);
        self.set_nz(v >> 1)
    }

    fn rol8(&mut self, v: u8) -> u8 {
        let c = self.psw & F_C;
        self.set_flag(F_C, v & 0x80 != 0);
        self.set_nz((v << 1) | c)
    }

    fn ror8(&mut self, v: u8) -> u8 {
        let c = (self.psw & F_C) << 7;
        self.set_flag(F_C, v & 1 != 0);
        self.set_nz((v >> 1) | c)
    }

    fn inc8(&mut self, v: u8) -> u8 {
        self.set_nz(v.wrapping_add(1))
    }

    fn dec8(&mut self, v: u8) -> u8 {
        self.set_nz(v.wrapping_sub(1))
    }

    // ---- 汎用パターン --------------------------------------------------------

    /// A ← A op mem
    fn alu_a<B: SpcBus, F: Fn(&mut Self, u8, u8) -> u8>(&mut self, bus: &mut B, addr: u16, f: F) {
        let v = bus.read(addr);
        self.a = f(self, self.a, v);
    }

    /// mem ← mem op (別 mem)  (OR dd,ds など)
    fn alu_dp_dp<B: SpcBus, F: Fn(&mut Self, u8, u8) -> u8>(&mut self, bus: &mut B, f: F) {
        let src = self.dp(bus);
        let s = bus.read(src);
        let dst = self.dp(bus);
        let d = bus.read(dst);
        let r = f(self, d, s);
        bus.write(dst, r);
    }

    fn alu_dp_imm<B: SpcBus, F: Fn(&mut Self, u8, u8) -> u8>(&mut self, bus: &mut B, f: F) {
        let imm = self.fetch(bus);
        let dst = self.dp(bus);
        let d = bus.read(dst);
        let r = f(self, d, imm);
        bus.write(dst, r);
    }

    fn alu_x_y<B: SpcBus, F: Fn(&mut Self, u8, u8) -> u8>(&mut self, bus: &mut B, f: F) {
        let s = bus.read(self.at_y());
        let dst = self.at_x();
        let d = bus.read(dst);
        let r = f(self, d, s);
        bus.write(dst, r);
    }

    /// CMP 系 (書き戻しなし)
    fn cmp_dp_dp<B: SpcBus>(&mut self, bus: &mut B) {
        let src = self.dp(bus);
        let s = bus.read(src);
        let dst = self.dp(bus);
        let d = bus.read(dst);
        self.cmp8(d, s);
    }

    fn cmp_dp_imm<B: SpcBus>(&mut self, bus: &mut B) {
        let imm = self.fetch(bus);
        let dst = self.dp(bus);
        let d = bus.read(dst);
        self.cmp8(d, imm);
    }

    fn cmp_x_y<B: SpcBus>(&mut self, bus: &mut B) {
        let s = bus.read(self.at_y());
        let d = bus.read(self.at_x());
        self.cmp8(d, s);
    }

    fn rmw<B: SpcBus, F: Fn(&mut Self, u8) -> u8>(&mut self, bus: &mut B, addr: u16, f: F) {
        let v = bus.read(addr);
        let r = f(self, v);
        bus.write(addr, r);
    }

    fn branch<B: SpcBus>(&mut self, bus: &mut B, cond: bool) {
        let rel = self.fetch(bus) as i8;
        if cond {
            self.pc = self.pc.wrapping_add(rel as u16);
            self.extra += 2;
        }
    }

    /// dp.bit 分岐 (BBS/BBC)
    fn branch_bit<B: SpcBus>(&mut self, bus: &mut B, bit: u8, set: bool) {
        let addr = self.dp(bus);
        let v = bus.read(addr);
        let cond = ((v >> bit) & 1 != 0) == set;
        self.branch(bus, cond);
    }

    /// abs.bit オペランド: (アドレス, ビット番号)
    fn abs_bit<B: SpcBus>(&mut self, bus: &mut B) -> (u16, u8) {
        let v = self.fetch16(bus);
        (v & 0x1FFF, (v >> 13) as u8)
    }

    fn call<B: SpcBus>(&mut self, bus: &mut B, target: u16) {
        self.push16(bus, self.pc);
        self.pc = target;
    }

    // ---- 実行 -----------------------------------------------------------------

    pub fn step<B: SpcBus>(&mut self, bus: &mut B) -> u32 {
        if self.stopped {
            return 2;
        }
        self.extra = 0;
        let op = self.fetch(bus);
        self.execute(bus, op);
        CYCLES[op as usize] + self.extra
    }

    fn execute<B: SpcBus>(&mut self, bus: &mut B, op: u8) {
        match op {
            // ---- TCALL / SET1 / CLR1 / BBS / BBC (列 1,2,3) ----
            0x01 | 0x11 | 0x21 | 0x31 | 0x41 | 0x51 | 0x61 | 0x71 | 0x81 | 0x91 | 0xA1
            | 0xB1 | 0xC1 | 0xD1 | 0xE1 | 0xF1 => {
                let n = (op >> 4) as u16;
                let vec = 0xFFDE - n * 2;
                let lo = bus.read(vec) as u16;
                let hi = bus.read(vec + 1) as u16;
                let t = (hi << 8) | lo;
                self.call(bus, t);
            }
            0x02 | 0x22 | 0x42 | 0x62 | 0x82 | 0xA2 | 0xC2 | 0xE2 => {
                let bit = op >> 5;
                let a = self.dp(bus);
                let v = bus.read(a);
                bus.write(a, v | (1 << bit));
            }
            0x12 | 0x32 | 0x52 | 0x72 | 0x92 | 0xB2 | 0xD2 | 0xF2 => {
                let bit = op >> 5;
                let a = self.dp(bus);
                let v = bus.read(a);
                bus.write(a, v & !(1 << bit));
            }
            0x03 | 0x23 | 0x43 | 0x63 | 0x83 | 0xA3 | 0xC3 | 0xE3 => {
                let bit = op >> 5;
                self.branch_bit(bus, bit, true);
            }
            0x13 | 0x33 | 0x53 | 0x73 | 0x93 | 0xB3 | 0xD3 | 0xF3 => {
                let bit = op >> 5;
                self.branch_bit(bus, bit, false);
            }

            // ---- OR ----
            0x04 => { let a = self.dp(bus); self.alu_a(bus, a, Self::or8); }
            0x05 => { let a = self.abs(bus); self.alu_a(bus, a, Self::or8); }
            0x06 => { let a = self.at_x(); self.alu_a(bus, a, Self::or8); }
            0x07 => { let a = self.ind_dp_x(bus); self.alu_a(bus, a, Self::or8); }
            0x08 => { let v = self.fetch(bus); self.a = self.or8(self.a, v); }
            0x09 => self.alu_dp_dp(bus, Self::or8),
            0x14 => { let a = self.dp_x(bus); self.alu_a(bus, a, Self::or8); }
            0x15 => { let a = self.abs_x(bus); self.alu_a(bus, a, Self::or8); }
            0x16 => { let a = self.abs_y(bus); self.alu_a(bus, a, Self::or8); }
            0x17 => { let a = self.ind_dp_y(bus); self.alu_a(bus, a, Self::or8); }
            0x18 => self.alu_dp_imm(bus, Self::or8),
            0x19 => self.alu_x_y(bus, Self::or8),

            // ---- AND ----
            0x24 => { let a = self.dp(bus); self.alu_a(bus, a, Self::and8); }
            0x25 => { let a = self.abs(bus); self.alu_a(bus, a, Self::and8); }
            0x26 => { let a = self.at_x(); self.alu_a(bus, a, Self::and8); }
            0x27 => { let a = self.ind_dp_x(bus); self.alu_a(bus, a, Self::and8); }
            0x28 => { let v = self.fetch(bus); self.a = self.and8(self.a, v); }
            0x29 => self.alu_dp_dp(bus, Self::and8),
            0x34 => { let a = self.dp_x(bus); self.alu_a(bus, a, Self::and8); }
            0x35 => { let a = self.abs_x(bus); self.alu_a(bus, a, Self::and8); }
            0x36 => { let a = self.abs_y(bus); self.alu_a(bus, a, Self::and8); }
            0x37 => { let a = self.ind_dp_y(bus); self.alu_a(bus, a, Self::and8); }
            0x38 => self.alu_dp_imm(bus, Self::and8),
            0x39 => self.alu_x_y(bus, Self::and8),

            // ---- EOR ----
            0x44 => { let a = self.dp(bus); self.alu_a(bus, a, Self::eor8); }
            0x45 => { let a = self.abs(bus); self.alu_a(bus, a, Self::eor8); }
            0x46 => { let a = self.at_x(); self.alu_a(bus, a, Self::eor8); }
            0x47 => { let a = self.ind_dp_x(bus); self.alu_a(bus, a, Self::eor8); }
            0x48 => { let v = self.fetch(bus); self.a = self.eor8(self.a, v); }
            0x49 => self.alu_dp_dp(bus, Self::eor8),
            0x54 => { let a = self.dp_x(bus); self.alu_a(bus, a, Self::eor8); }
            0x55 => { let a = self.abs_x(bus); self.alu_a(bus, a, Self::eor8); }
            0x56 => { let a = self.abs_y(bus); self.alu_a(bus, a, Self::eor8); }
            0x57 => { let a = self.ind_dp_y(bus); self.alu_a(bus, a, Self::eor8); }
            0x58 => self.alu_dp_imm(bus, Self::eor8),
            0x59 => self.alu_x_y(bus, Self::eor8),

            // ---- CMP A ----
            0x64 => { let a = self.dp(bus); let v = bus.read(a); self.cmp8(self.a, v); }
            0x65 => { let a = self.abs(bus); let v = bus.read(a); self.cmp8(self.a, v); }
            0x66 => { let v = bus.read(self.at_x()); self.cmp8(self.a, v); }
            0x67 => { let a = self.ind_dp_x(bus); let v = bus.read(a); self.cmp8(self.a, v); }
            0x68 => { let v = self.fetch(bus); self.cmp8(self.a, v); }
            0x69 => self.cmp_dp_dp(bus),
            0x74 => { let a = self.dp_x(bus); let v = bus.read(a); self.cmp8(self.a, v); }
            0x75 => { let a = self.abs_x(bus); let v = bus.read(a); self.cmp8(self.a, v); }
            0x76 => { let a = self.abs_y(bus); let v = bus.read(a); self.cmp8(self.a, v); }
            0x77 => { let a = self.ind_dp_y(bus); let v = bus.read(a); self.cmp8(self.a, v); }
            0x78 => self.cmp_dp_imm(bus),
            0x79 => self.cmp_x_y(bus),

            // ---- ADC ----
            0x84 => { let a = self.dp(bus); self.alu_a(bus, a, Self::adc8); }
            0x85 => { let a = self.abs(bus); self.alu_a(bus, a, Self::adc8); }
            0x86 => { let a = self.at_x(); self.alu_a(bus, a, Self::adc8); }
            0x87 => { let a = self.ind_dp_x(bus); self.alu_a(bus, a, Self::adc8); }
            0x88 => { let v = self.fetch(bus); self.a = self.adc8(self.a, v); }
            0x89 => self.alu_dp_dp(bus, Self::adc8),
            0x94 => { let a = self.dp_x(bus); self.alu_a(bus, a, Self::adc8); }
            0x95 => { let a = self.abs_x(bus); self.alu_a(bus, a, Self::adc8); }
            0x96 => { let a = self.abs_y(bus); self.alu_a(bus, a, Self::adc8); }
            0x97 => { let a = self.ind_dp_y(bus); self.alu_a(bus, a, Self::adc8); }
            0x98 => self.alu_dp_imm(bus, Self::adc8),
            0x99 => self.alu_x_y(bus, Self::adc8),

            // ---- SBC ----
            0xA4 => { let a = self.dp(bus); self.alu_a(bus, a, Self::sbc8); }
            0xA5 => { let a = self.abs(bus); self.alu_a(bus, a, Self::sbc8); }
            0xA6 => { let a = self.at_x(); self.alu_a(bus, a, Self::sbc8); }
            0xA7 => { let a = self.ind_dp_x(bus); self.alu_a(bus, a, Self::sbc8); }
            0xA8 => { let v = self.fetch(bus); self.a = self.sbc8(self.a, v); }
            0xA9 => self.alu_dp_dp(bus, Self::sbc8),
            0xB4 => { let a = self.dp_x(bus); self.alu_a(bus, a, Self::sbc8); }
            0xB5 => { let a = self.abs_x(bus); self.alu_a(bus, a, Self::sbc8); }
            0xB6 => { let a = self.abs_y(bus); self.alu_a(bus, a, Self::sbc8); }
            0xB7 => { let a = self.ind_dp_y(bus); self.alu_a(bus, a, Self::sbc8); }
            0xB8 => self.alu_dp_imm(bus, Self::sbc8),
            0xB9 => self.alu_x_y(bus, Self::sbc8),

            // ---- CMP X / CMP Y ----
            0xC8 => { let v = self.fetch(bus); self.cmp8(self.x, v); }
            0x3E => { let a = self.dp(bus); let v = bus.read(a); self.cmp8(self.x, v); }
            0x1E => { let a = self.abs(bus); let v = bus.read(a); self.cmp8(self.x, v); }
            0xAD => { let v = self.fetch(bus); self.cmp8(self.y, v); }
            0x7E => { let a = self.dp(bus); let v = bus.read(a); self.cmp8(self.y, v); }
            0x5E => { let a = self.abs(bus); let v = bus.read(a); self.cmp8(self.y, v); }

            // ---- MOV (レジスタへ: NZ 変化) ----
            0xE8 => { let v = self.fetch(bus); self.a = self.set_nz(v); }
            0xE4 => { let a = self.dp(bus); let v = bus.read(a); self.a = self.set_nz(v); }
            0xF4 => { let a = self.dp_x(bus); let v = bus.read(a); self.a = self.set_nz(v); }
            0xE5 => { let a = self.abs(bus); let v = bus.read(a); self.a = self.set_nz(v); }
            0xF5 => { let a = self.abs_x(bus); let v = bus.read(a); self.a = self.set_nz(v); }
            0xF6 => { let a = self.abs_y(bus); let v = bus.read(a); self.a = self.set_nz(v); }
            0xE6 => { let v = bus.read(self.at_x()); self.a = self.set_nz(v); }
            0xBF => {
                // MOV A,(X)+
                let v = bus.read(self.at_x());
                self.a = self.set_nz(v);
                self.x = self.x.wrapping_add(1);
            }
            0xE7 => { let a = self.ind_dp_x(bus); let v = bus.read(a); self.a = self.set_nz(v); }
            0xF7 => { let a = self.ind_dp_y(bus); let v = bus.read(a); self.a = self.set_nz(v); }
            0xCD => { let v = self.fetch(bus); self.x = self.set_nz(v); }
            0xF8 => { let a = self.dp(bus); let v = bus.read(a); self.x = self.set_nz(v); }
            0xF9 => { let a = self.dp_y(bus); let v = bus.read(a); self.x = self.set_nz(v); }
            0xE9 => { let a = self.abs(bus); let v = bus.read(a); self.x = self.set_nz(v); }
            0x8D => { let v = self.fetch(bus); self.y = self.set_nz(v); }
            0xEB => { let a = self.dp(bus); let v = bus.read(a); self.y = self.set_nz(v); }
            0xFB => { let a = self.dp_x(bus); let v = bus.read(a); self.y = self.set_nz(v); }
            0xEC => { let a = self.abs(bus); let v = bus.read(a); self.y = self.set_nz(v); }

            // ---- MOV (レジスタ間) ----
            0x7D => { self.a = self.set_nz(self.x); }
            0xDD => { self.a = self.set_nz(self.y); }
            0x5D => { self.x = self.set_nz(self.a); }
            0xFD => { self.y = self.set_nz(self.a); }
            0x9D => { self.x = self.set_nz(self.sp); }
            0xBD => { self.sp = self.x; }

            // ---- MOV (メモリへ: フラグ不変) ----
            0xC4 => { let a = self.dp(bus); bus.write(a, self.a); }
            0xD4 => { let a = self.dp_x(bus); bus.write(a, self.a); }
            0xC5 => { let a = self.abs(bus); bus.write(a, self.a); }
            0xD5 => { let a = self.abs_x(bus); bus.write(a, self.a); }
            0xD6 => { let a = self.abs_y(bus); bus.write(a, self.a); }
            0xC6 => { bus.write(self.at_x(), self.a); }
            0xAF => {
                // MOV (X)+,A
                bus.write(self.at_x(), self.a);
                self.x = self.x.wrapping_add(1);
            }
            0xC7 => { let a = self.ind_dp_x(bus); bus.write(a, self.a); }
            0xD7 => { let a = self.ind_dp_y(bus); bus.write(a, self.a); }
            0xD8 => { let a = self.dp(bus); bus.write(a, self.x); }
            0xD9 => { let a = self.dp_y(bus); bus.write(a, self.x); }
            0xC9 => { let a = self.abs(bus); bus.write(a, self.x); }
            0xCB => { let a = self.dp(bus); bus.write(a, self.y); }
            0xDB => { let a = self.dp_x(bus); bus.write(a, self.y); }
            0xCC => { let a = self.abs(bus); bus.write(a, self.y); }
            0x8F => {
                // MOV dp,#imm
                let imm = self.fetch(bus);
                let a = self.dp(bus);
                bus.read(a); // ダミーリード (実機挙動)
                bus.write(a, imm);
            }
            0xFA => {
                // MOV dp,dp
                let src = self.dp(bus);
                let v = bus.read(src);
                let dst = self.dp(bus);
                bus.write(dst, v);
            }

            // ---- シフト / INC / DEC ----
            0x0B => { let a = self.dp(bus); self.rmw(bus, a, Self::asl8); }
            0x1B => { let a = self.dp_x(bus); self.rmw(bus, a, Self::asl8); }
            0x0C => { let a = self.abs(bus); self.rmw(bus, a, Self::asl8); }
            0x1C => { self.a = self.asl8(self.a); }
            0x2B => { let a = self.dp(bus); self.rmw(bus, a, Self::rol8); }
            0x3B => { let a = self.dp_x(bus); self.rmw(bus, a, Self::rol8); }
            0x2C => { let a = self.abs(bus); self.rmw(bus, a, Self::rol8); }
            0x3C => { self.a = self.rol8(self.a); }
            0x4B => { let a = self.dp(bus); self.rmw(bus, a, Self::lsr8); }
            0x5B => { let a = self.dp_x(bus); self.rmw(bus, a, Self::lsr8); }
            0x4C => { let a = self.abs(bus); self.rmw(bus, a, Self::lsr8); }
            0x5C => { self.a = self.lsr8(self.a); }
            0x6B => { let a = self.dp(bus); self.rmw(bus, a, Self::ror8); }
            0x7B => { let a = self.dp_x(bus); self.rmw(bus, a, Self::ror8); }
            0x6C => { let a = self.abs(bus); self.rmw(bus, a, Self::ror8); }
            0x7C => { self.a = self.ror8(self.a); }
            0xAB => { let a = self.dp(bus); self.rmw(bus, a, Self::inc8); }
            0xBB => { let a = self.dp_x(bus); self.rmw(bus, a, Self::inc8); }
            0xAC => { let a = self.abs(bus); self.rmw(bus, a, Self::inc8); }
            0xBC => { self.a = self.inc8(self.a); }
            0x8B => { let a = self.dp(bus); self.rmw(bus, a, Self::dec8); }
            0x9B => { let a = self.dp_x(bus); self.rmw(bus, a, Self::dec8); }
            0x8C => { let a = self.abs(bus); self.rmw(bus, a, Self::dec8); }
            0x9C => { self.a = self.dec8(self.a); }
            0x3D => { self.x = self.inc8(self.x); }
            0x1D => { self.x = self.dec8(self.x); }
            0xFC => { self.y = self.inc8(self.y); }
            0xDC => { self.y = self.dec8(self.y); }

            // ---- 16bit 演算 ----
            0xBA => {
                // MOVW YA,dp
                let a = self.dp(bus);
                let lo = bus.read(a) as u16;
                let hi = bus.read((a & 0xFF00) | (a as u8).wrapping_add(1) as u16) as u16;
                let v = self.set_nz16((hi << 8) | lo);
                self.a = v as u8;
                self.y = (v >> 8) as u8;
            }
            0xDA => {
                // MOVW dp,YA (フラグ不変)
                let a = self.dp(bus);
                bus.read(a); // ダミーリード
                bus.write(a, self.a);
                bus.write((a & 0xFF00) | (a as u8).wrapping_add(1) as u16, self.y);
            }
            0x3A => {
                // INCW dp
                let a = self.dp(bus);
                let hi_a = (a & 0xFF00) | (a as u8).wrapping_add(1) as u16;
                let lo = bus.read(a);
                let (lo2, carry) = lo.overflowing_add(1);
                bus.write(a, lo2);
                let hi = bus.read(hi_a).wrapping_add(carry as u8);
                bus.write(hi_a, hi);
                self.set_nz16(((hi as u16) << 8) | lo2 as u16);
            }
            0x1A => {
                // DECW dp
                let a = self.dp(bus);
                let hi_a = (a & 0xFF00) | (a as u8).wrapping_add(1) as u16;
                let lo = bus.read(a);
                let (lo2, borrow) = lo.overflowing_sub(1);
                bus.write(a, lo2);
                let hi = bus.read(hi_a).wrapping_sub(borrow as u8);
                bus.write(hi_a, hi);
                self.set_nz16(((hi as u16) << 8) | lo2 as u16);
            }
            0x7A => {
                // ADDW YA,dp
                let a = self.dp(bus);
                let lo = bus.read(a);
                let hi = bus.read((a & 0xFF00) | (a as u8).wrapping_add(1) as u16);
                self.set_flag(F_C, false);
                let rl = self.adc8(self.a, lo);
                let rh = self.adc8(self.y, hi);
                self.a = rl;
                self.y = rh;
                self.set_flag(F_Z, rl == 0 && rh == 0);
            }
            0x9A => {
                // SUBW YA,dp
                let a = self.dp(bus);
                let lo = bus.read(a);
                let hi = bus.read((a & 0xFF00) | (a as u8).wrapping_add(1) as u16);
                self.set_flag(F_C, true);
                let rl = self.sbc8(self.a, lo);
                let rh = self.sbc8(self.y, hi);
                self.a = rl;
                self.y = rh;
                self.set_flag(F_Z, rl == 0 && rh == 0);
            }
            0x5A => {
                // CMPW YA,dp
                let a = self.dp(bus);
                let lo = bus.read(a) as u16;
                let hi = bus.read((a & 0xFF00) | (a as u8).wrapping_add(1) as u16) as u16;
                let w = (hi << 8) | lo;
                let ya = ((self.y as u16) << 8) | self.a as u16;
                let r = ya.wrapping_sub(w);
                self.set_flag(F_C, ya >= w);
                self.set_nz16(r);
            }
            0xCF => {
                // MUL YA
                let r = self.y as u16 * self.a as u16;
                self.a = r as u8;
                self.y = (r >> 8) as u8;
                let y = self.y;
                self.set_nz(y);
            }
            0x9E => {
                // DIV YA,X
                let ya = ((self.y as u16) << 8) | self.a as u16;
                let x = self.x as u16;
                self.set_flag(F_H, (self.x & 0x0F) <= (self.y & 0x0F));
                self.set_flag(F_V, self.y >= self.x);
                if (self.y as u16) < (x << 1) {
                    self.a = (ya / x.max(1)) as u8;
                    self.y = (ya % x.max(1)) as u8;
                } else {
                    self.a = (255 - (ya - (x << 9)) / (256 - x)) as u8;
                    self.y = (x + (ya - (x << 9)) % (256 - x)) as u8;
                }
                let a = self.a;
                self.set_nz(a);
            }

            // ---- DAA / DAS / XCN ----
            0xDF => {
                // DAA
                if self.flag(F_C) || self.a > 0x99 {
                    self.a = self.a.wrapping_add(0x60);
                    self.set_flag(F_C, true);
                }
                if self.flag(F_H) || self.a & 0x0F > 0x09 {
                    self.a = self.a.wrapping_add(0x06);
                }
                let a = self.a;
                self.set_nz(a);
            }
            0xBE => {
                // DAS
                if !self.flag(F_C) || self.a > 0x99 {
                    self.a = self.a.wrapping_sub(0x60);
                    self.set_flag(F_C, false);
                }
                if !self.flag(F_H) || self.a & 0x0F > 0x09 {
                    self.a = self.a.wrapping_sub(0x06);
                }
                let a = self.a;
                self.set_nz(a);
            }
            0x9F => {
                // XCN A
                self.a = self.a.rotate_right(4);
                let a = self.a;
                self.set_nz(a);
            }

            // ---- 1bit 演算 ----
            0x0A => { let (a, b) = self.abs_bit(bus); let v = bus.read(a); let c = self.flag(F_C) | ((v >> b) & 1 != 0); self.set_flag(F_C, c); }
            0x2A => { let (a, b) = self.abs_bit(bus); let v = bus.read(a); let c = self.flag(F_C) | ((v >> b) & 1 == 0); self.set_flag(F_C, c); }
            0x4A => { let (a, b) = self.abs_bit(bus); let v = bus.read(a); let c = self.flag(F_C) & ((v >> b) & 1 != 0); self.set_flag(F_C, c); }
            0x6A => { let (a, b) = self.abs_bit(bus); let v = bus.read(a); let c = self.flag(F_C) & ((v >> b) & 1 == 0); self.set_flag(F_C, c); }
            0x8A => { let (a, b) = self.abs_bit(bus); let v = bus.read(a); let c = self.flag(F_C) ^ ((v >> b) & 1 != 0); self.set_flag(F_C, c); }
            0xAA => { let (a, b) = self.abs_bit(bus); let v = bus.read(a); self.set_flag(F_C, (v >> b) & 1 != 0); }
            0xCA => {
                let (a, b) = self.abs_bit(bus);
                let v = bus.read(a);
                let r = if self.flag(F_C) { v | (1 << b) } else { v & !(1 << b) };
                bus.write(a, r);
            }
            0xEA => {
                let (a, b) = self.abs_bit(bus);
                let v = bus.read(a);
                bus.write(a, v ^ (1 << b));
            }
            0x0E => {
                // TSET1 !abs
                let a = self.abs(bus);
                let v = bus.read(a);
                let r = self.a.wrapping_sub(v);
                self.set_nz(r);
                bus.read(a); // ダミーリード
                bus.write(a, v | self.a);
            }
            0x4E => {
                // TCLR1 !abs
                let a = self.abs(bus);
                let v = bus.read(a);
                let r = self.a.wrapping_sub(v);
                self.set_nz(r);
                bus.read(a);
                bus.write(a, v & !self.a);
            }

            // ---- 分岐 / ジャンプ / コール ----
            0x10 => { let c = !self.flag(F_N); self.branch(bus, c); }
            0x30 => { let c = self.flag(F_N); self.branch(bus, c); }
            0x50 => { let c = !self.flag(F_V); self.branch(bus, c); }
            0x70 => { let c = self.flag(F_V); self.branch(bus, c); }
            0x90 => { let c = !self.flag(F_C); self.branch(bus, c); }
            0xB0 => { let c = self.flag(F_C); self.branch(bus, c); }
            0xD0 => { let c = !self.flag(F_Z); self.branch(bus, c); }
            0xF0 => { let c = self.flag(F_Z); self.branch(bus, c); }
            0x2F => { self.branch(bus, true); }
            0x2E => {
                // CBNE dp,rel
                let a = self.dp(bus);
                let v = bus.read(a);
                let c = self.a != v;
                self.branch(bus, c);
            }
            0xDE => {
                // CBNE dp+X,rel
                let a = self.dp_x(bus);
                let v = bus.read(a);
                let c = self.a != v;
                self.branch(bus, c);
            }
            0x6E => {
                // DBNZ dp,rel
                let a = self.dp(bus);
                let v = bus.read(a).wrapping_sub(1);
                bus.write(a, v);
                self.branch(bus, v != 0);
            }
            0xFE => {
                // DBNZ Y,rel
                self.y = self.y.wrapping_sub(1);
                let c = self.y != 0;
                self.branch(bus, c);
            }
            0x5F => { self.pc = self.fetch16(bus); }
            0x1F => {
                // JMP [!abs+X]
                let p = self.abs_x(bus);
                let lo = bus.read(p) as u16;
                let hi = bus.read(p.wrapping_add(1)) as u16;
                self.pc = (hi << 8) | lo;
            }
            0x3F => { let t = self.fetch16(bus); self.call(bus, t); }
            0x4F => {
                // PCALL
                let u = self.fetch(bus) as u16;
                self.call(bus, 0xFF00 | u);
            }
            0x6F => { self.pc = self.pull16(bus); }
            0x7F => {
                self.psw = self.pull(bus);
                self.pc = self.pull16(bus);
            }
            0x0F => {
                // BRK
                self.push16(bus, self.pc);
                self.push(bus, self.psw);
                self.set_flag(F_B, true);
                self.set_flag(F_I, false);
                let lo = bus.read(0xFFDE) as u16;
                let hi = bus.read(0xFFDF) as u16;
                self.pc = (hi << 8) | lo;
            }

            // ---- スタック ----
            0x0D => { let v = self.psw; self.push(bus, v); }
            0x2D => { let v = self.a; self.push(bus, v); }
            0x4D => { let v = self.x; self.push(bus, v); }
            0x6D => { let v = self.y; self.push(bus, v); }
            0x8E => { self.psw = self.pull(bus); }
            0xAE => { self.a = self.pull(bus); }
            0xCE => { self.x = self.pull(bus); }
            0xEE => { self.y = self.pull(bus); }

            // ---- フラグ操作 ----
            0x60 => { self.set_flag(F_C, false); }
            0x80 => { self.set_flag(F_C, true); }
            0xED => { let c = !self.flag(F_C); self.set_flag(F_C, c); }
            0xE0 => { self.set_flag(F_V, false); self.set_flag(F_H, false); }
            0x20 => { self.set_flag(F_P, false); }
            0x40 => { self.set_flag(F_P, true); }
            0xA0 => { self.set_flag(F_I, true); }
            0xC0 => { self.set_flag(F_I, false); }

            // ---- その他 ----
            0x00 => {}
            0xEF | 0xFF => {
                // SLEEP / STOP
                self.stopped = true;
            }
        }
    }
}
