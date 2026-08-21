//! 65C816 CPU コア。
//!
//! サイクルはバス側で計上する: `Bus::read`/`write` がアクセス先に応じた
//! マスターサイクルを加算し、`Bus::idle` が内部サイクル (6 マスターサイクル) を
//! 加算する。CPU は命令ごとに正しい回数だけバスを呼ぶことに専念する。

pub const FLAG_C: u8 = 0x01;
pub const FLAG_Z: u8 = 0x02;
pub const FLAG_I: u8 = 0x04;
pub const FLAG_D: u8 = 0x08;
pub const FLAG_X: u8 = 0x10;
pub const FLAG_M: u8 = 0x20;
pub const FLAG_V: u8 = 0x40;
pub const FLAG_N: u8 = 0x80;

pub trait Bus {
    fn read(&mut self, addr: u32) -> u8;
    fn write(&mut self, addr: u32, data: u8);
    /// CPU 内部サイクル 1 回 (SNES では 6 マスターサイクル)。
    fn idle(&mut self);
    /// NMI エッジ。true を一度返したら消費される。
    fn nmi_pending(&mut self) -> bool {
        false
    }
    /// IRQ ライン (レベルトリガ)。
    fn irq_level(&self) -> bool {
        false
    }
}

pub struct Cpu {
    pub a: u16,
    pub x: u16,
    pub y: u16,
    pub s: u16,
    pub d: u16,
    pub pc: u16,
    pub dbr: u8,
    pub pbr: u8,
    pub p: u8,
    pub e: bool,
    pub stopped: bool,
    pub waiting: bool,
}

impl Default for Cpu {
    fn default() -> Self {
        Cpu {
            a: 0,
            x: 0,
            y: 0,
            s: 0x01FF,
            d: 0,
            pc: 0,
            dbr: 0,
            pbr: 0,
            p: FLAG_M | FLAG_X | FLAG_I,
            e: true,
            stopped: false,
            waiting: false,
        }
    }
}

impl Cpu {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset<B: Bus>(&mut self, bus: &mut B) {
        *self = Cpu::default();
        self.pc = self.read16_bank0(bus, 0xFFFC);
    }

    // ---- フラグ操作 ----------------------------------------------------

    #[inline]
    fn flag(&self, f: u8) -> bool {
        self.p & f != 0
    }

    #[inline]
    fn set_flag(&mut self, f: u8, v: bool) {
        if v {
            self.p |= f;
        } else {
            self.p &= !f;
        }
    }

    /// アキュムレータ/メモリが 8bit か
    #[inline]
    pub fn m8(&self) -> bool {
        self.e || self.flag(FLAG_M)
    }

    /// インデックスレジスタが 8bit か
    #[inline]
    pub fn x8(&self) -> bool {
        self.e || self.flag(FLAG_X)
    }

    fn set_nz8(&mut self, v: u8) {
        self.set_flag(FLAG_Z, v == 0);
        self.set_flag(FLAG_N, v & 0x80 != 0);
    }

    fn set_nz16(&mut self, v: u16) {
        self.set_flag(FLAG_Z, v == 0);
        self.set_flag(FLAG_N, v & 0x8000 != 0);
    }

    fn set_nz(&mut self, v: u16, eight: bool) {
        if eight {
            self.set_nz8(v as u8);
        } else {
            self.set_nz16(v);
        }
    }

    /// P レジスタ書き換え (REP/SEP/PLP/RTI)。E モードでは M/X は常に 1。
    fn set_p(&mut self, v: u8) {
        self.p = v;
        if self.e {
            self.p |= FLAG_M | FLAG_X;
        }
        if self.flag(FLAG_X) {
            self.x &= 0x00FF;
            self.y &= 0x00FF;
        }
    }

    // ---- フェッチ / メモリアクセス --------------------------------------

    #[inline]
    fn fetch<B: Bus>(&mut self, bus: &mut B) -> u8 {
        let v = bus.read(((self.pbr as u32) << 16) | self.pc as u32);
        self.pc = self.pc.wrapping_add(1);
        v
    }

    fn fetch16<B: Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.fetch(bus) as u16;
        let hi = self.fetch(bus) as u16;
        (hi << 8) | lo
    }

    fn fetch24<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let lo = self.fetch(bus) as u32;
        let mid = self.fetch(bus) as u32;
        let hi = self.fetch(bus) as u32;
        (hi << 16) | (mid << 8) | lo
    }

    /// addr の次のアドレス。wrap16 = true なら同一バンク内で 16bit ラップ。
    #[inline]
    fn next_addr(addr: u32, wrap16: bool) -> u32 {
        if wrap16 {
            (addr & 0xFF_0000) | (addr as u16).wrapping_add(1) as u32
        } else {
            (addr + 1) & 0xFF_FFFF
        }
    }

    fn read16<B: Bus>(&mut self, bus: &mut B, addr: u32, wrap16: bool) -> u16 {
        let lo = bus.read(addr) as u16;
        let hi = bus.read(Self::next_addr(addr, wrap16)) as u16;
        (hi << 8) | lo
    }

    fn read16_bank0<B: Bus>(&mut self, bus: &mut B, addr: u16) -> u16 {
        let lo = bus.read(addr as u32) as u16;
        let hi = bus.read(addr.wrapping_add(1) as u32) as u16;
        (hi << 8) | lo
    }

    /// 幅つきデータ読み出し
    fn read_w<B: Bus>(&mut self, bus: &mut B, addr: u32, wrap16: bool, eight: bool) -> u16 {
        if eight {
            bus.read(addr) as u16
        } else {
            self.read16(bus, addr, wrap16)
        }
    }

    fn write_w<B: Bus>(&mut self, bus: &mut B, addr: u32, v: u16, wrap16: bool, eight: bool) {
        bus.write(addr, v as u8);
        if !eight {
            bus.write(Self::next_addr(addr, wrap16), (v >> 8) as u8);
        }
    }

    /// 直接ページのポインタ読み (16bit)。バンク 0 内で 16bit ラップ。
    /// (E モード DL=0 でもポインタ読み自体はページをまたぐ — 実機検証済み)
    fn read_ptr16<B: Bus>(&mut self, bus: &mut B, p: u32) -> u16 {
        let lo = bus.read(p) as u16;
        let hi = bus.read((p as u16).wrapping_add(1) as u32) as u16;
        (hi << 8) | lo
    }

    // ---- スタック --------------------------------------------------------

    fn push8<B: Bus>(&mut self, bus: &mut B, v: u8) {
        bus.write(self.s as u32, v);
        self.s = self.s.wrapping_sub(1);
        if self.e {
            self.s = 0x0100 | (self.s & 0xFF);
        }
    }

    fn pull8<B: Bus>(&mut self, bus: &mut B) -> u8 {
        self.s = self.s.wrapping_add(1);
        if self.e {
            self.s = 0x0100 | (self.s & 0xFF);
        }
        bus.read(self.s as u32)
    }

    fn push16<B: Bus>(&mut self, bus: &mut B, v: u16) {
        self.push8(bus, (v >> 8) as u8);
        self.push8(bus, v as u8);
    }

    fn pull16<B: Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.pull8(bus) as u16;
        let hi = self.pull8(bus) as u16;
        (hi << 8) | lo
    }

    fn push_w<B: Bus>(&mut self, bus: &mut B, v: u16, eight: bool) {
        if eight {
            self.push8(bus, v as u8);
        } else {
            self.push16(bus, v);
        }
    }

    fn pull_w<B: Bus>(&mut self, bus: &mut B, eight: bool) -> u16 {
        if eight {
            self.pull8(bus) as u16
        } else {
            self.pull16(bus)
        }
    }

    // 65816 新命令 (PEA/PEI/PER/PHD/PLD/PHB/PLB/JSL/RTL) は E モードでも
    // スタックをページ 1 内でラップさせず、命令終了時に S 上位を 0x01 に戻す。

    fn push8_n<B: Bus>(&mut self, bus: &mut B, v: u8) {
        bus.write(self.s as u32, v);
        self.s = self.s.wrapping_sub(1);
    }

    fn pull8_n<B: Bus>(&mut self, bus: &mut B) -> u8 {
        self.s = self.s.wrapping_add(1);
        bus.read(self.s as u32)
    }

    fn push16_n<B: Bus>(&mut self, bus: &mut B, v: u16) {
        self.push8_n(bus, (v >> 8) as u8);
        self.push8_n(bus, v as u8);
    }

    fn pull16_n<B: Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.pull8_n(bus) as u16;
        let hi = self.pull8_n(bus) as u16;
        (hi << 8) | lo
    }

    fn stack_fixup(&mut self) {
        if self.e {
            self.s = 0x0100 | (self.s & 0xFF);
        }
    }

    // ---- アドレッシングモード --------------------------------------------
    // 戻り値は 24bit 実効アドレス。

    fn am_abs<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let a = self.fetch16(bus) as u32;
        ((self.dbr as u32) << 16) | a
    }

    /// abs,X / abs,Y。write: 書き込みまたは RMW ならページ跨ぎに関係なくペナルティ。
    fn am_abs_idx<B: Bus>(&mut self, bus: &mut B, idx: u16, write: bool) -> u32 {
        let base = self.fetch16(bus) as u32;
        let eff = ((self.dbr as u32) << 16).wrapping_add(base).wrapping_add(idx as u32) & 0xFF_FFFF;
        if write || !self.x8() || (base & 0xFF00) != ((base + idx as u32) & 0xFF00) {
            bus.idle();
        }
        eff
    }

    fn am_long<B: Bus>(&mut self, bus: &mut B) -> u32 {
        self.fetch24(bus)
    }

    fn am_long_x<B: Bus>(&mut self, bus: &mut B) -> u32 {
        (self.fetch24(bus) + self.x as u32) & 0xFF_FFFF
    }

    fn am_dp<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let off = self.fetch(bus) as u16;
        if self.d & 0xFF != 0 {
            bus.idle();
        }
        self.d.wrapping_add(off) as u32
    }

    fn am_dp_idx<B: Bus>(&mut self, bus: &mut B, idx: u16) -> u32 {
        let off = self.fetch(bus) as u16;
        if self.d & 0xFF != 0 {
            bus.idle();
        }
        bus.idle();
        if self.e && self.d & 0xFF == 0 {
            (self.d | (off.wrapping_add(idx) & 0xFF)) as u32
        } else {
            self.d.wrapping_add(off).wrapping_add(idx) as u32
        }
    }

    /// (d)
    fn am_ind<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let p = self.am_dp(bus);
        let ptr = self.read_ptr16(bus, p);
        ((self.dbr as u32) << 16) | ptr as u32
    }

    /// (d,x)
    fn am_ind_x<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let p = self.am_dp_idx(bus, self.x);
        let ptr = self.read_ptr16(bus, p);
        ((self.dbr as u32) << 16) | ptr as u32
    }

    /// (d),y
    fn am_ind_y<B: Bus>(&mut self, bus: &mut B, write: bool) -> u32 {
        let p = self.am_dp(bus);
        let ptr = self.read_ptr16(bus, p) as u32;
        let eff = ((self.dbr as u32) << 16).wrapping_add(ptr).wrapping_add(self.y as u32) & 0xFF_FFFF;
        if write || !self.x8() || (ptr & 0xFF00) != ((ptr + self.y as u32) & 0xFF00) {
            bus.idle();
        }
        eff
    }

    /// [d]
    fn am_ind_long<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let p = self.am_dp(bus);
        let lo = bus.read(p) as u32;
        let mid = bus.read((p as u16).wrapping_add(1) as u32) as u32;
        let hi = bus.read((p as u16).wrapping_add(2) as u32) as u32;
        (hi << 16) | (mid << 8) | lo
    }

    /// [d],y
    fn am_ind_long_y<B: Bus>(&mut self, bus: &mut B) -> u32 {
        (self.am_ind_long(bus) + self.y as u32) & 0xFF_FFFF
    }

    /// sr,S
    fn am_sr<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let off = self.fetch(bus) as u16;
        bus.idle();
        self.s.wrapping_add(off) as u32
    }

    /// (sr,S),y
    fn am_sr_y<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let p = self.am_sr(bus);
        let ptr = self.read16(bus, p, true) as u32;
        bus.idle();
        ((self.dbr as u32) << 16).wrapping_add(ptr).wrapping_add(self.y as u32) & 0xFF_FFFF
    }

    fn imm_m<B: Bus>(&mut self, bus: &mut B) -> u16 {
        if self.m8() {
            self.fetch(bus) as u16
        } else {
            self.fetch16(bus)
        }
    }

    fn imm_x<B: Bus>(&mut self, bus: &mut B) -> u16 {
        if self.x8() {
            self.fetch(bus) as u16
        } else {
            self.fetch16(bus)
        }
    }

    fn read_m<B: Bus>(&mut self, bus: &mut B, addr: u32, wrap16: bool) -> u16 {
        let eight = self.m8();
        self.read_w(bus, addr, wrap16, eight)
    }

    // ---- 演算 -------------------------------------------------------------

    fn lda(&mut self, v: u16) {
        if self.m8() {
            self.a = (self.a & 0xFF00) | (v & 0xFF);
            self.set_nz8(v as u8);
        } else {
            self.a = v;
            self.set_nz16(v);
        }
    }

    fn ora(&mut self, v: u16) {
        let r = self.a | v;
        self.lda(if self.m8() { (self.a & 0xFF00) | (r & 0xFF) } else { r });
    }

    fn and(&mut self, v: u16) {
        let r = self.a & v;
        self.lda(r);
    }

    fn eor(&mut self, v: u16) {
        let r = self.a ^ v;
        self.lda(if self.m8() { (self.a & 0xFF00) | (r & 0xFF) } else { r });
    }

    fn adc(&mut self, v: u16) {
        if self.m8() {
            self.adc8(v as u8);
        } else {
            self.adc16(v);
        }
    }

    fn adc8(&mut self, v: u8) {
        let a = self.a as u8;
        let c = (self.p & FLAG_C) as i32;
        let mut r: i32;
        if self.flag(FLAG_D) {
            r = (a as i32 & 0x0F) + (v as i32 & 0x0F) + c;
            if r > 0x09 {
                r += 0x06;
            }
            let c2 = (r > 0x0F) as i32;
            r = (a as i32 & 0xF0) + (v as i32 & 0xF0) + (c2 << 4) + (r & 0x0F);
        } else {
            r = a as i32 + v as i32 + c;
        }
        self.set_flag(FLAG_V, !(a ^ v) & (a ^ r as u8) & 0x80 != 0);
        if self.flag(FLAG_D) && r > 0x9F {
            r += 0x60;
        }
        self.set_flag(FLAG_C, r > 0xFF);
        let r8 = r as u8;
        self.set_nz8(r8);
        self.a = (self.a & 0xFF00) | r8 as u16;
    }

    fn adc16(&mut self, v: u16) {
        let a = self.a as i32;
        let v = v as i32;
        let c = (self.p & FLAG_C) as i32;
        let mut r: i32;
        if self.flag(FLAG_D) {
            r = (a & 0x000F) + (v & 0x000F) + c;
            if r > 0x0009 {
                r += 0x0006;
            }
            let mut cc = (r > 0x000F) as i32;
            r = (a & 0x00F0) + (v & 0x00F0) + (cc << 4) + (r & 0x000F);
            if r > 0x009F {
                r += 0x0060;
            }
            cc = (r > 0x00FF) as i32;
            r = (a & 0x0F00) + (v & 0x0F00) + (cc << 8) + (r & 0x00FF);
            if r > 0x09FF {
                r += 0x0600;
            }
            cc = (r > 0x0FFF) as i32;
            r = (a & 0xF000) + (v & 0xF000) + (cc << 12) + (r & 0x0FFF);
        } else {
            r = a + v + c;
        }
        self.set_flag(FLAG_V, !(a ^ v) & (a ^ r) & 0x8000 != 0);
        if self.flag(FLAG_D) && r > 0x9FFF {
            r += 0x6000;
        }
        self.set_flag(FLAG_C, r > 0xFFFF);
        self.a = r as u16;
        self.set_nz16(self.a);
    }

    fn sbc(&mut self, v: u16) {
        if self.m8() {
            self.sbc8(v as u8);
        } else {
            self.sbc16(v);
        }
    }

    fn sbc8(&mut self, v: u8) {
        let a = self.a as u8;
        let v = !v;
        let c = (self.p & FLAG_C) as i32;
        let mut r: i32;
        if self.flag(FLAG_D) {
            r = (a as i32 & 0x0F) + (v as i32 & 0x0F) + c;
            if r <= 0x0F {
                r -= 0x06;
            }
            let c2 = (r > 0x0F) as i32;
            r = (a as i32 & 0xF0) + (v as i32 & 0xF0) + (c2 << 4) + (r & 0x0F);
        } else {
            r = a as i32 + v as i32 + c;
        }
        self.set_flag(FLAG_V, !(a ^ v) & (a ^ r as u8) & 0x80 != 0);
        if self.flag(FLAG_D) && r <= 0xFF {
            r -= 0x60;
        }
        self.set_flag(FLAG_C, r > 0xFF);
        let r8 = r as u8;
        self.set_nz8(r8);
        self.a = (self.a & 0xFF00) | r8 as u16;
    }

    fn sbc16(&mut self, v: u16) {
        let a = self.a as i32;
        let v = (!v) as i32;
        let c = (self.p & FLAG_C) as i32;
        let mut r: i32;
        if self.flag(FLAG_D) {
            r = (a & 0x000F) + (v & 0x000F) + c;
            if r <= 0x000F {
                r -= 0x0006;
            }
            let mut cc = (r > 0x000F) as i32;
            r = (a & 0x00F0) + (v & 0x00F0) + (cc << 4) + (r & 0x000F);
            if r <= 0x00FF {
                r -= 0x0060;
            }
            cc = (r > 0x00FF) as i32;
            r = (a & 0x0F00) + (v & 0x0F00) + (cc << 8) + (r & 0x00FF);
            if r <= 0x0FFF {
                r -= 0x0600;
            }
            cc = (r > 0x0FFF) as i32;
            r = (a & 0xF000) + (v & 0xF000) + (cc << 12) + (r & 0x0FFF);
        } else {
            r = a + v + c;
        }
        self.set_flag(FLAG_V, !(a ^ v) & (a ^ r) & 0x8000 != 0);
        if self.flag(FLAG_D) && r <= 0xFFFF {
            r -= 0x6000;
        }
        self.set_flag(FLAG_C, r > 0xFFFF);
        self.a = r as u16;
        self.set_nz16(self.a);
    }

    fn compare(&mut self, reg: u16, v: u16, eight: bool) {
        let (reg, v) = if eight { (reg & 0xFF, v & 0xFF) } else { (reg, v) };
        let r = reg.wrapping_sub(v);
        self.set_flag(FLAG_C, reg >= v);
        self.set_nz(r, eight);
    }

    fn bit(&mut self, v: u16) {
        if self.m8() {
            self.set_flag(FLAG_Z, (self.a as u8) & (v as u8) == 0);
            self.set_flag(FLAG_N, v & 0x80 != 0);
            self.set_flag(FLAG_V, v & 0x40 != 0);
        } else {
            self.set_flag(FLAG_Z, self.a & v == 0);
            self.set_flag(FLAG_N, v & 0x8000 != 0);
            self.set_flag(FLAG_V, v & 0x4000 != 0);
        }
    }

    // シフト/インクリメント系: 値を受け取り結果を返す (A と RMW で共用)
    fn asl_val(&mut self, v: u16) -> u16 {
        if self.m8() {
            let v = v as u8;
            self.set_flag(FLAG_C, v & 0x80 != 0);
            let r = v << 1;
            self.set_nz8(r);
            r as u16
        } else {
            self.set_flag(FLAG_C, v & 0x8000 != 0);
            let r = v << 1;
            self.set_nz16(r);
            r
        }
    }

    fn lsr_val(&mut self, v: u16) -> u16 {
        let v = if self.m8() { v & 0xFF } else { v };
        self.set_flag(FLAG_C, v & 1 != 0);
        let r = v >> 1;
        self.set_nz(r, self.m8());
        r
    }

    fn rol_val(&mut self, v: u16) -> u16 {
        let c = (self.p & FLAG_C) as u16;
        if self.m8() {
            let v = v as u8;
            self.set_flag(FLAG_C, v & 0x80 != 0);
            let r = (v << 1) | c as u8;
            self.set_nz8(r);
            r as u16
        } else {
            self.set_flag(FLAG_C, v & 0x8000 != 0);
            let r = (v << 1) | c;
            self.set_nz16(r);
            r
        }
    }

    fn ror_val(&mut self, v: u16) -> u16 {
        let c = (self.p & FLAG_C) as u16;
        if self.m8() {
            let v = v as u8;
            self.set_flag(FLAG_C, v & 1 != 0);
            let r = (v >> 1) | ((c as u8) << 7);
            self.set_nz8(r);
            r as u16
        } else {
            self.set_flag(FLAG_C, v & 1 != 0);
            let r = (v >> 1) | (c << 15);
            self.set_nz16(r);
            r
        }
    }

    fn inc_val(&mut self, v: u16) -> u16 {
        let r = if self.m8() {
            (v as u8).wrapping_add(1) as u16
        } else {
            v.wrapping_add(1)
        };
        self.set_nz(r, self.m8());
        r
    }

    fn dec_val(&mut self, v: u16) -> u16 {
        let r = if self.m8() {
            (v as u8).wrapping_sub(1) as u16
        } else {
            v.wrapping_sub(1)
        };
        self.set_nz(r, self.m8());
        r
    }

    fn tsb_val(&mut self, v: u16) -> u16 {
        let a = if self.m8() { self.a & 0xFF } else { self.a };
        self.set_flag(FLAG_Z, a & v == 0);
        v | a
    }

    fn trb_val(&mut self, v: u16) -> u16 {
        let a = if self.m8() { self.a & 0xFF } else { self.a };
        self.set_flag(FLAG_Z, a & v == 0);
        v & !a
    }

    /// Read-Modify-Write。16bit 時は上位バイトから書き戻す (実機準拠)。
    fn rmw<B: Bus, F: FnOnce(&mut Self, u16) -> u16>(
        &mut self,
        bus: &mut B,
        addr: u32,
        wrap16: bool,
        f: F,
    ) {
        if self.m8() {
            let v = bus.read(addr) as u16;
            bus.idle();
            let r = f(self, v);
            bus.write(addr, r as u8);
        } else {
            let hi_addr = Self::next_addr(addr, wrap16);
            let lo = bus.read(addr) as u16;
            let hi = bus.read(hi_addr) as u16;
            bus.idle();
            let r = f(self, (hi << 8) | lo);
            bus.write(hi_addr, (r >> 8) as u8);
            bus.write(addr, r as u8);
        }
    }

    fn branch<B: Bus>(&mut self, bus: &mut B, cond: bool) {
        let off = self.fetch(bus) as i8;
        if cond {
            bus.idle();
            let old = self.pc;
            self.pc = self.pc.wrapping_add(off as u16);
            if self.e && (old & 0xFF00) != (self.pc & 0xFF00) {
                bus.idle();
            }
        }
    }

    // ---- 割り込み ---------------------------------------------------------

    pub fn hw_interrupt<B: Bus>(&mut self, bus: &mut B, nmi: bool) {
        self.waiting = false;
        bus.idle();
        bus.idle();
        if !self.e {
            self.push8(bus, self.pbr);
        }
        self.push16(bus, self.pc);
        let p = if self.e { self.p & !0x10 } else { self.p };
        self.push8(bus, p);
        self.p = (self.p | FLAG_I) & !FLAG_D;
        self.pbr = 0;
        let vec: u16 = match (nmi, self.e) {
            (true, false) => 0xFFEA,
            (true, true) => 0xFFFA,
            (false, false) => 0xFFEE,
            (false, true) => 0xFFFE,
        };
        self.pc = self.read16_bank0(bus, vec);
    }

    fn sw_interrupt<B: Bus>(&mut self, bus: &mut B, vec_native: u16, vec_emu: u16) {
        self.fetch(bus); // シグネチャバイト
        if !self.e {
            self.push8(bus, self.pbr);
        }
        self.push16(bus, self.pc);
        let p = if self.e { self.p | 0x10 } else { self.p };
        self.push8(bus, p);
        self.p = (self.p | FLAG_I) & !FLAG_D;
        self.pbr = 0;
        let vec = if self.e { vec_emu } else { vec_native };
        self.pc = self.read16_bank0(bus, vec);
    }

    // ---- 実行 -------------------------------------------------------------

    /// 1 命令 (または割り込み) を実行する。
    pub fn step<B: Bus>(&mut self, bus: &mut B) {
        if self.stopped {
            bus.idle();
            return;
        }
        if bus.nmi_pending() {
            self.hw_interrupt(bus, true);
            return;
        }
        if bus.irq_level() {
            if self.waiting {
                self.waiting = false; // I フラグが立っていても WAI は解除される
            }
            if !self.flag(FLAG_I) {
                self.hw_interrupt(bus, false);
                return;
            }
        }
        if self.waiting {
            bus.idle();
            return;
        }

        let op = self.fetch(bus);
        self.execute(bus, op);
    }

    fn execute<B: Bus>(&mut self, bus: &mut B, op: u8) {
        match op {
            // ---- ORA ----
            0x01 => { let a = self.am_ind_x(bus); let v = self.read_m(bus, a, false); self.ora(v); }
            0x03 => { let a = self.am_sr(bus); let v = self.read_m(bus, a, true); self.ora(v); }
            0x05 => { let a = self.am_dp(bus); let v = self.read_m(bus, a, true); self.ora(v); }
            0x07 => { let a = self.am_ind_long(bus); let v = self.read_m(bus, a, false); self.ora(v); }
            0x09 => { let v = self.imm_m(bus); self.ora(v); }
            0x0D => { let a = self.am_abs(bus); let v = self.read_m(bus, a, false); self.ora(v); }
            0x0F => { let a = self.am_long(bus); let v = self.read_m(bus, a, false); self.ora(v); }
            0x11 => { let a = self.am_ind_y(bus, false); let v = self.read_m(bus, a, false); self.ora(v); }
            0x12 => { let a = self.am_ind(bus); let v = self.read_m(bus, a, false); self.ora(v); }
            0x13 => { let a = self.am_sr_y(bus); let v = self.read_m(bus, a, false); self.ora(v); }
            0x15 => { let a = self.am_dp_idx(bus, self.x); let v = self.read_m(bus, a, true); self.ora(v); }
            0x17 => { let a = self.am_ind_long_y(bus); let v = self.read_m(bus, a, false); self.ora(v); }
            0x19 => { let a = self.am_abs_idx(bus, self.y, false); let v = self.read_m(bus, a, false); self.ora(v); }
            0x1D => { let a = self.am_abs_idx(bus, self.x, false); let v = self.read_m(bus, a, false); self.ora(v); }
            0x1F => { let a = self.am_long_x(bus); let v = self.read_m(bus, a, false); self.ora(v); }

            // ---- AND ----
            0x21 => { let a = self.am_ind_x(bus); let v = self.read_m(bus, a, false); self.and(v); }
            0x23 => { let a = self.am_sr(bus); let v = self.read_m(bus, a, true); self.and(v); }
            0x25 => { let a = self.am_dp(bus); let v = self.read_m(bus, a, true); self.and(v); }
            0x27 => { let a = self.am_ind_long(bus); let v = self.read_m(bus, a, false); self.and(v); }
            0x29 => { let v = self.imm_m(bus); self.and(v); }
            0x2D => { let a = self.am_abs(bus); let v = self.read_m(bus, a, false); self.and(v); }
            0x2F => { let a = self.am_long(bus); let v = self.read_m(bus, a, false); self.and(v); }
            0x31 => { let a = self.am_ind_y(bus, false); let v = self.read_m(bus, a, false); self.and(v); }
            0x32 => { let a = self.am_ind(bus); let v = self.read_m(bus, a, false); self.and(v); }
            0x33 => { let a = self.am_sr_y(bus); let v = self.read_m(bus, a, false); self.and(v); }
            0x35 => { let a = self.am_dp_idx(bus, self.x); let v = self.read_m(bus, a, true); self.and(v); }
            0x37 => { let a = self.am_ind_long_y(bus); let v = self.read_m(bus, a, false); self.and(v); }
            0x39 => { let a = self.am_abs_idx(bus, self.y, false); let v = self.read_m(bus, a, false); self.and(v); }
            0x3D => { let a = self.am_abs_idx(bus, self.x, false); let v = self.read_m(bus, a, false); self.and(v); }
            0x3F => { let a = self.am_long_x(bus); let v = self.read_m(bus, a, false); self.and(v); }

            // ---- EOR ----
            0x41 => { let a = self.am_ind_x(bus); let v = self.read_m(bus, a, false); self.eor(v); }
            0x43 => { let a = self.am_sr(bus); let v = self.read_m(bus, a, true); self.eor(v); }
            0x45 => { let a = self.am_dp(bus); let v = self.read_m(bus, a, true); self.eor(v); }
            0x47 => { let a = self.am_ind_long(bus); let v = self.read_m(bus, a, false); self.eor(v); }
            0x49 => { let v = self.imm_m(bus); self.eor(v); }
            0x4D => { let a = self.am_abs(bus); let v = self.read_m(bus, a, false); self.eor(v); }
            0x4F => { let a = self.am_long(bus); let v = self.read_m(bus, a, false); self.eor(v); }
            0x51 => { let a = self.am_ind_y(bus, false); let v = self.read_m(bus, a, false); self.eor(v); }
            0x52 => { let a = self.am_ind(bus); let v = self.read_m(bus, a, false); self.eor(v); }
            0x53 => { let a = self.am_sr_y(bus); let v = self.read_m(bus, a, false); self.eor(v); }
            0x55 => { let a = self.am_dp_idx(bus, self.x); let v = self.read_m(bus, a, true); self.eor(v); }
            0x57 => { let a = self.am_ind_long_y(bus); let v = self.read_m(bus, a, false); self.eor(v); }
            0x59 => { let a = self.am_abs_idx(bus, self.y, false); let v = self.read_m(bus, a, false); self.eor(v); }
            0x5D => { let a = self.am_abs_idx(bus, self.x, false); let v = self.read_m(bus, a, false); self.eor(v); }
            0x5F => { let a = self.am_long_x(bus); let v = self.read_m(bus, a, false); self.eor(v); }

            // ---- ADC ----
            0x61 => { let a = self.am_ind_x(bus); let v = self.read_m(bus, a, false); self.adc(v); }
            0x63 => { let a = self.am_sr(bus); let v = self.read_m(bus, a, true); self.adc(v); }
            0x65 => { let a = self.am_dp(bus); let v = self.read_m(bus, a, true); self.adc(v); }
            0x67 => { let a = self.am_ind_long(bus); let v = self.read_m(bus, a, false); self.adc(v); }
            0x69 => { let v = self.imm_m(bus); self.adc(v); }
            0x6D => { let a = self.am_abs(bus); let v = self.read_m(bus, a, false); self.adc(v); }
            0x6F => { let a = self.am_long(bus); let v = self.read_m(bus, a, false); self.adc(v); }
            0x71 => { let a = self.am_ind_y(bus, false); let v = self.read_m(bus, a, false); self.adc(v); }
            0x72 => { let a = self.am_ind(bus); let v = self.read_m(bus, a, false); self.adc(v); }
            0x73 => { let a = self.am_sr_y(bus); let v = self.read_m(bus, a, false); self.adc(v); }
            0x75 => { let a = self.am_dp_idx(bus, self.x); let v = self.read_m(bus, a, true); self.adc(v); }
            0x77 => { let a = self.am_ind_long_y(bus); let v = self.read_m(bus, a, false); self.adc(v); }
            0x79 => { let a = self.am_abs_idx(bus, self.y, false); let v = self.read_m(bus, a, false); self.adc(v); }
            0x7D => { let a = self.am_abs_idx(bus, self.x, false); let v = self.read_m(bus, a, false); self.adc(v); }
            0x7F => { let a = self.am_long_x(bus); let v = self.read_m(bus, a, false); self.adc(v); }

            // ---- SBC ----
            0xE1 => { let a = self.am_ind_x(bus); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xE3 => { let a = self.am_sr(bus); let v = self.read_m(bus, a, true); self.sbc(v); }
            0xE5 => { let a = self.am_dp(bus); let v = self.read_m(bus, a, true); self.sbc(v); }
            0xE7 => { let a = self.am_ind_long(bus); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xE9 => { let v = self.imm_m(bus); self.sbc(v); }
            0xED => { let a = self.am_abs(bus); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xEF => { let a = self.am_long(bus); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xF1 => { let a = self.am_ind_y(bus, false); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xF2 => { let a = self.am_ind(bus); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xF3 => { let a = self.am_sr_y(bus); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xF5 => { let a = self.am_dp_idx(bus, self.x); let v = self.read_m(bus, a, true); self.sbc(v); }
            0xF7 => { let a = self.am_ind_long_y(bus); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xF9 => { let a = self.am_abs_idx(bus, self.y, false); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xFD => { let a = self.am_abs_idx(bus, self.x, false); let v = self.read_m(bus, a, false); self.sbc(v); }
            0xFF => { let a = self.am_long_x(bus); let v = self.read_m(bus, a, false); self.sbc(v); }

            // ---- CMP ----
            0xC1 => { let a = self.am_ind_x(bus); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xC3 => { let a = self.am_sr(bus); let v = self.read_m(bus, a, true); self.compare(self.a, v, self.m8()); }
            0xC5 => { let a = self.am_dp(bus); let v = self.read_m(bus, a, true); self.compare(self.a, v, self.m8()); }
            0xC7 => { let a = self.am_ind_long(bus); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xC9 => { let v = self.imm_m(bus); self.compare(self.a, v, self.m8()); }
            0xCD => { let a = self.am_abs(bus); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xCF => { let a = self.am_long(bus); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xD1 => { let a = self.am_ind_y(bus, false); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xD2 => { let a = self.am_ind(bus); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xD3 => { let a = self.am_sr_y(bus); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xD5 => { let a = self.am_dp_idx(bus, self.x); let v = self.read_m(bus, a, true); self.compare(self.a, v, self.m8()); }
            0xD7 => { let a = self.am_ind_long_y(bus); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xD9 => { let a = self.am_abs_idx(bus, self.y, false); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xDD => { let a = self.am_abs_idx(bus, self.x, false); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }
            0xDF => { let a = self.am_long_x(bus); let v = self.read_m(bus, a, false); self.compare(self.a, v, self.m8()); }

            // ---- CPX / CPY ----
            0xE0 => { let v = self.imm_x(bus); self.compare(self.x, v, self.x8()); }
            0xE4 => { let a = self.am_dp(bus); let e = self.x8(); let v = self.read_w(bus, a, true, e); self.compare(self.x, v, e); }
            0xEC => { let a = self.am_abs(bus); let e = self.x8(); let v = self.read_w(bus, a, false, e); self.compare(self.x, v, e); }
            0xC0 => { let v = self.imm_x(bus); self.compare(self.y, v, self.x8()); }
            0xC4 => { let a = self.am_dp(bus); let e = self.x8(); let v = self.read_w(bus, a, true, e); self.compare(self.y, v, e); }
            0xCC => { let a = self.am_abs(bus); let e = self.x8(); let v = self.read_w(bus, a, false, e); self.compare(self.y, v, e); }

            // ---- LDA ----
            0xA1 => { let a = self.am_ind_x(bus); let v = self.read_m(bus, a, false); self.lda(v); }
            0xA3 => { let a = self.am_sr(bus); let v = self.read_m(bus, a, true); self.lda(v); }
            0xA5 => { let a = self.am_dp(bus); let v = self.read_m(bus, a, true); self.lda(v); }
            0xA7 => { let a = self.am_ind_long(bus); let v = self.read_m(bus, a, false); self.lda(v); }
            0xA9 => { let v = self.imm_m(bus); self.lda(v); }
            0xAD => { let a = self.am_abs(bus); let v = self.read_m(bus, a, false); self.lda(v); }
            0xAF => { let a = self.am_long(bus); let v = self.read_m(bus, a, false); self.lda(v); }
            0xB1 => { let a = self.am_ind_y(bus, false); let v = self.read_m(bus, a, false); self.lda(v); }
            0xB2 => { let a = self.am_ind(bus); let v = self.read_m(bus, a, false); self.lda(v); }
            0xB3 => { let a = self.am_sr_y(bus); let v = self.read_m(bus, a, false); self.lda(v); }
            0xB5 => { let a = self.am_dp_idx(bus, self.x); let v = self.read_m(bus, a, true); self.lda(v); }
            0xB7 => { let a = self.am_ind_long_y(bus); let v = self.read_m(bus, a, false); self.lda(v); }
            0xB9 => { let a = self.am_abs_idx(bus, self.y, false); let v = self.read_m(bus, a, false); self.lda(v); }
            0xBD => { let a = self.am_abs_idx(bus, self.x, false); let v = self.read_m(bus, a, false); self.lda(v); }
            0xBF => { let a = self.am_long_x(bus); let v = self.read_m(bus, a, false); self.lda(v); }

            // ---- LDX / LDY ----
            0xA2 => { let v = self.imm_x(bus); self.x = v; self.set_nz(v, self.x8()); }
            0xA6 => { let a = self.am_dp(bus); let e = self.x8(); let v = self.read_w(bus, a, true, e); self.x = v; self.set_nz(v, e); }
            0xB6 => { let a = self.am_dp_idx(bus, self.y); let e = self.x8(); let v = self.read_w(bus, a, true, e); self.x = v; self.set_nz(v, e); }
            0xAE => { let a = self.am_abs(bus); let e = self.x8(); let v = self.read_w(bus, a, false, e); self.x = v; self.set_nz(v, e); }
            0xBE => { let a = self.am_abs_idx(bus, self.y, false); let e = self.x8(); let v = self.read_w(bus, a, false, e); self.x = v; self.set_nz(v, e); }
            0xA0 => { let v = self.imm_x(bus); self.y = v; self.set_nz(v, self.x8()); }
            0xA4 => { let a = self.am_dp(bus); let e = self.x8(); let v = self.read_w(bus, a, true, e); self.y = v; self.set_nz(v, e); }
            0xB4 => { let a = self.am_dp_idx(bus, self.x); let e = self.x8(); let v = self.read_w(bus, a, true, e); self.y = v; self.set_nz(v, e); }
            0xAC => { let a = self.am_abs(bus); let e = self.x8(); let v = self.read_w(bus, a, false, e); self.y = v; self.set_nz(v, e); }
            0xBC => { let a = self.am_abs_idx(bus, self.x, false); let e = self.x8(); let v = self.read_w(bus, a, false, e); self.y = v; self.set_nz(v, e); }

            // ---- STA ----
            0x81 => { let a = self.am_ind_x(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x83 => { let a = self.am_sr(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, true, e); }
            0x85 => { let a = self.am_dp(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, true, e); }
            0x87 => { let a = self.am_ind_long(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x8D => { let a = self.am_abs(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x8F => { let a = self.am_long(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x91 => { let a = self.am_ind_y(bus, true); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x92 => { let a = self.am_ind(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x93 => { let a = self.am_sr_y(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x95 => { let a = self.am_dp_idx(bus, self.x); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, true, e); }
            0x97 => { let a = self.am_ind_long_y(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x99 => { let a = self.am_abs_idx(bus, self.y, true); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x9D => { let a = self.am_abs_idx(bus, self.x, true); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }
            0x9F => { let a = self.am_long_x(bus); let (v, e) = (self.a, self.m8()); self.write_w(bus, a, v, false, e); }

            // ---- STX / STY / STZ ----
            0x86 => { let a = self.am_dp(bus); let (v, e) = (self.x, self.x8()); self.write_w(bus, a, v, true, e); }
            0x96 => { let a = self.am_dp_idx(bus, self.y); let (v, e) = (self.x, self.x8()); self.write_w(bus, a, v, true, e); }
            0x8E => { let a = self.am_abs(bus); let (v, e) = (self.x, self.x8()); self.write_w(bus, a, v, false, e); }
            0x84 => { let a = self.am_dp(bus); let (v, e) = (self.y, self.x8()); self.write_w(bus, a, v, true, e); }
            0x94 => { let a = self.am_dp_idx(bus, self.x); let (v, e) = (self.y, self.x8()); self.write_w(bus, a, v, true, e); }
            0x8C => { let a = self.am_abs(bus); let (v, e) = (self.y, self.x8()); self.write_w(bus, a, v, false, e); }
            0x64 => { let a = self.am_dp(bus); let e = self.m8(); self.write_w(bus, a, 0, true, e); }
            0x74 => { let a = self.am_dp_idx(bus, self.x); let e = self.m8(); self.write_w(bus, a, 0, true, e); }
            0x9C => { let a = self.am_abs(bus); let e = self.m8(); self.write_w(bus, a, 0, false, e); }
            0x9E => { let a = self.am_abs_idx(bus, self.x, true); let e = self.m8(); self.write_w(bus, a, 0, false, e); }

            // ---- BIT / TSB / TRB ----
            0x24 => { let a = self.am_dp(bus); let v = self.read_m(bus, a, true); self.bit(v); }
            0x2C => { let a = self.am_abs(bus); let v = self.read_m(bus, a, false); self.bit(v); }
            0x34 => { let a = self.am_dp_idx(bus, self.x); let v = self.read_m(bus, a, true); self.bit(v); }
            0x3C => { let a = self.am_abs_idx(bus, self.x, false); let v = self.read_m(bus, a, false); self.bit(v); }
            0x89 => {
                // BIT #imm は Z のみ変化
                let v = self.imm_m(bus);
                let a = if self.m8() { self.a & 0xFF } else { self.a };
                self.set_flag(FLAG_Z, a & v == 0);
            }
            0x04 => { let a = self.am_dp(bus); self.rmw(bus, a, true, Self::tsb_val); }
            0x0C => { let a = self.am_abs(bus); self.rmw(bus, a, false, Self::tsb_val); }
            0x14 => { let a = self.am_dp(bus); self.rmw(bus, a, true, Self::trb_val); }
            0x1C => { let a = self.am_abs(bus); self.rmw(bus, a, false, Self::trb_val); }

            // ---- シフト (A) ----
            0x0A => { bus.idle(); let v = self.a; let r = self.asl_val(v); self.set_a_w(r); }
            0x4A => { bus.idle(); let v = self.a; let r = self.lsr_val(v); self.set_a_w(r); }
            0x2A => { bus.idle(); let v = self.a; let r = self.rol_val(v); self.set_a_w(r); }
            0x6A => { bus.idle(); let v = self.a; let r = self.ror_val(v); self.set_a_w(r); }
            0x1A => { bus.idle(); let v = self.a; let r = self.inc_val(v); self.set_a_w(r); }
            0x3A => { bus.idle(); let v = self.a; let r = self.dec_val(v); self.set_a_w(r); }

            // ---- シフト / INC / DEC (メモリ) ----
            0x06 => { let a = self.am_dp(bus); self.rmw(bus, a, true, Self::asl_val); }
            0x0E => { let a = self.am_abs(bus); self.rmw(bus, a, false, Self::asl_val); }
            0x16 => { let a = self.am_dp_idx(bus, self.x); self.rmw(bus, a, true, Self::asl_val); }
            0x1E => { let a = self.am_abs_idx(bus, self.x, true); self.rmw(bus, a, false, Self::asl_val); }
            0x26 => { let a = self.am_dp(bus); self.rmw(bus, a, true, Self::rol_val); }
            0x2E => { let a = self.am_abs(bus); self.rmw(bus, a, false, Self::rol_val); }
            0x36 => { let a = self.am_dp_idx(bus, self.x); self.rmw(bus, a, true, Self::rol_val); }
            0x3E => { let a = self.am_abs_idx(bus, self.x, true); self.rmw(bus, a, false, Self::rol_val); }
            0x46 => { let a = self.am_dp(bus); self.rmw(bus, a, true, Self::lsr_val); }
            0x4E => { let a = self.am_abs(bus); self.rmw(bus, a, false, Self::lsr_val); }
            0x56 => { let a = self.am_dp_idx(bus, self.x); self.rmw(bus, a, true, Self::lsr_val); }
            0x5E => { let a = self.am_abs_idx(bus, self.x, true); self.rmw(bus, a, false, Self::lsr_val); }
            0x66 => { let a = self.am_dp(bus); self.rmw(bus, a, true, Self::ror_val); }
            0x6E => { let a = self.am_abs(bus); self.rmw(bus, a, false, Self::ror_val); }
            0x76 => { let a = self.am_dp_idx(bus, self.x); self.rmw(bus, a, true, Self::ror_val); }
            0x7E => { let a = self.am_abs_idx(bus, self.x, true); self.rmw(bus, a, false, Self::ror_val); }
            0xE6 => { let a = self.am_dp(bus); self.rmw(bus, a, true, Self::inc_val); }
            0xEE => { let a = self.am_abs(bus); self.rmw(bus, a, false, Self::inc_val); }
            0xF6 => { let a = self.am_dp_idx(bus, self.x); self.rmw(bus, a, true, Self::inc_val); }
            0xFE => { let a = self.am_abs_idx(bus, self.x, true); self.rmw(bus, a, false, Self::inc_val); }
            0xC6 => { let a = self.am_dp(bus); self.rmw(bus, a, true, Self::dec_val); }
            0xCE => { let a = self.am_abs(bus); self.rmw(bus, a, false, Self::dec_val); }
            0xD6 => { let a = self.am_dp_idx(bus, self.x); self.rmw(bus, a, true, Self::dec_val); }
            0xDE => { let a = self.am_abs_idx(bus, self.x, true); self.rmw(bus, a, false, Self::dec_val); }

            // ---- INX/INY/DEX/DEY ----
            0xE8 => { bus.idle(); self.x = self.idx_add(self.x, 1); }
            0xC8 => { bus.idle(); self.y = self.idx_add(self.y, 1); }
            0xCA => { bus.idle(); self.x = self.idx_add(self.x, 0xFFFF); }
            0x88 => { bus.idle(); self.y = self.idx_add(self.y, 0xFFFF); }

            // ---- 分岐 ----
            0x10 => { let c = !self.flag(FLAG_N); self.branch(bus, c); }
            0x30 => { let c = self.flag(FLAG_N); self.branch(bus, c); }
            0x50 => { let c = !self.flag(FLAG_V); self.branch(bus, c); }
            0x70 => { let c = self.flag(FLAG_V); self.branch(bus, c); }
            0x90 => { let c = !self.flag(FLAG_C); self.branch(bus, c); }
            0xB0 => { let c = self.flag(FLAG_C); self.branch(bus, c); }
            0xD0 => { let c = !self.flag(FLAG_Z); self.branch(bus, c); }
            0xF0 => { let c = self.flag(FLAG_Z); self.branch(bus, c); }
            0x80 => { self.branch(bus, true); }
            0x82 => {
                let off = self.fetch16(bus);
                bus.idle();
                self.pc = self.pc.wrapping_add(off);
            }

            // ---- ジャンプ / コール ----
            0x4C => { self.pc = self.fetch16(bus); }
            0x5C => {
                let a = self.fetch24(bus);
                self.pbr = (a >> 16) as u8;
                self.pc = a as u16;
            }
            0x6C => {
                let ptr = self.fetch16(bus);
                self.pc = self.read16_bank0(bus, ptr);
            }
            0x7C => {
                let ptr = self.fetch16(bus);
                bus.idle();
                let addr = ((self.pbr as u32) << 16) | ptr.wrapping_add(self.x) as u32;
                self.pc = self.read16(bus, addr, true);
            }
            0xDC => {
                let ptr = self.fetch16(bus);
                let lo = bus.read(ptr as u32) as u16;
                let hi = bus.read(ptr.wrapping_add(1) as u32) as u16;
                let bank = bus.read(ptr.wrapping_add(2) as u32);
                self.pc = (hi << 8) | lo;
                self.pbr = bank;
            }
            0x20 => {
                let t = self.fetch16(bus);
                bus.idle();
                let ret = self.pc.wrapping_sub(1);
                self.push16(bus, ret);
                self.pc = t;
            }
            0xFC => {
                let ptr = self.fetch16(bus);
                let ret = self.pc.wrapping_sub(1);
                self.push16(bus, ret);
                bus.idle();
                let addr = ((self.pbr as u32) << 16) | ptr.wrapping_add(self.x) as u32;
                self.pc = self.read16(bus, addr, true);
            }
            0x22 => {
                let t = self.fetch16(bus);
                self.push8_n(bus, self.pbr);
                bus.idle();
                let bank = self.fetch(bus);
                let ret = self.pc.wrapping_sub(1);
                self.push16_n(bus, ret);
                self.stack_fixup();
                self.pbr = bank;
                self.pc = t;
            }
            0x60 => {
                bus.idle();
                bus.idle();
                self.pc = self.pull16(bus).wrapping_add(1);
                bus.idle();
            }
            0x6B => {
                bus.idle();
                bus.idle();
                self.pc = self.pull16_n(bus).wrapping_add(1);
                self.pbr = self.pull8_n(bus);
                self.stack_fixup();
            }
            0x40 => {
                bus.idle();
                bus.idle();
                let p = self.pull8(bus);
                self.set_p(p);
                self.pc = self.pull16(bus);
                if !self.e {
                    self.pbr = self.pull8(bus);
                }
            }

            // ---- 割り込み命令 ----
            0x00 => { self.sw_interrupt(bus, 0xFFE6, 0xFFFE); }
            0x02 => { self.sw_interrupt(bus, 0xFFE4, 0xFFF4); }

            // ---- スタック ----
            0x48 => { bus.idle(); let (v, e) = (self.a, self.m8()); self.push_w(bus, v, e); }
            0x68 => { bus.idle(); bus.idle(); let e = self.m8(); let v = self.pull_w(bus, e); self.lda(v); }
            0xDA => { bus.idle(); let (v, e) = (self.x, self.x8()); self.push_w(bus, v, e); }
            0xFA => { bus.idle(); bus.idle(); let e = self.x8(); let v = self.pull_w(bus, e); self.x = v; self.set_nz(v, e); }
            0x5A => { bus.idle(); let (v, e) = (self.y, self.x8()); self.push_w(bus, v, e); }
            0x7A => { bus.idle(); bus.idle(); let e = self.x8(); let v = self.pull_w(bus, e); self.y = v; self.set_nz(v, e); }
            0x08 => { bus.idle(); let p = self.p; self.push8(bus, p); }
            0x28 => { bus.idle(); bus.idle(); let p = self.pull8(bus); self.set_p(p); }
            0x8B => { bus.idle(); self.push8_n(bus, self.dbr); self.stack_fixup(); }
            0xAB => { bus.idle(); bus.idle(); let v = self.pull8_n(bus); self.stack_fixup(); self.dbr = v; self.set_nz8(v); }
            0x0B => { bus.idle(); let d = self.d; self.push16_n(bus, d); self.stack_fixup(); }
            0x2B => { bus.idle(); bus.idle(); let v = self.pull16_n(bus); self.stack_fixup(); self.d = v; self.set_nz16(v); }
            0x4B => { bus.idle(); self.push8_n(bus, self.pbr); self.stack_fixup(); }
            0xF4 => { let v = self.fetch16(bus); self.push16_n(bus, v); self.stack_fixup(); }
            0xD4 => {
                let p = self.am_dp(bus);
                let v = self.read16(bus, p, true);
                self.push16_n(bus, v);
                self.stack_fixup();
            }
            0x62 => {
                let off = self.fetch16(bus);
                bus.idle();
                let v = self.pc.wrapping_add(off);
                self.push16_n(bus, v);
                self.stack_fixup();
            }

            // ---- フラグ操作 ----
            0x18 => { bus.idle(); self.set_flag(FLAG_C, false); }
            0x38 => { bus.idle(); self.set_flag(FLAG_C, true); }
            0x58 => { bus.idle(); self.set_flag(FLAG_I, false); }
            0x78 => { bus.idle(); self.set_flag(FLAG_I, true); }
            0xD8 => { bus.idle(); self.set_flag(FLAG_D, false); }
            0xF8 => { bus.idle(); self.set_flag(FLAG_D, true); }
            0xB8 => { bus.idle(); self.set_flag(FLAG_V, false); }
            0xC2 => { let v = self.fetch(bus); bus.idle(); let p = self.p & !v; self.set_p(p); }
            0xE2 => { let v = self.fetch(bus); bus.idle(); let p = self.p | v; self.set_p(p); }

            // ---- 転送 ----
            0xAA => { bus.idle(); let v = self.width_x(self.a); self.x = v; self.set_nz(v, self.x8()); }
            0xA8 => { bus.idle(); let v = self.width_x(self.a); self.y = v; self.set_nz(v, self.x8()); }
            0x8A => { bus.idle(); let v = self.x; self.lda(v); }
            0x98 => { bus.idle(); let v = self.y; self.lda(v); }
            0xBA => { bus.idle(); let v = self.width_x(self.s); self.x = v; self.set_nz(v, self.x8()); }
            0x9A => {
                bus.idle();
                self.s = if self.e { 0x0100 | (self.x & 0xFF) } else { self.x };
            }
            0x9B => { bus.idle(); let v = self.width_x(self.x); self.y = v; self.set_nz(v, self.x8()); }
            0xBB => { bus.idle(); let v = self.width_x(self.y); self.x = v; self.set_nz(v, self.x8()); }
            0x1B => {
                bus.idle();
                self.s = if self.e { 0x0100 | (self.a & 0xFF) } else { self.a };
            }
            0x3B => { bus.idle(); self.a = self.s; self.set_nz16(self.a); }
            0x5B => { bus.idle(); self.d = self.a; self.set_nz16(self.d); }
            0x7B => { bus.idle(); self.a = self.d; self.set_nz16(self.a); }

            // ---- ブロック転送 ----
            0x54 | 0x44 => {
                // MVN (0x54) / MVP (0x44)
                let dst_bank = self.fetch(bus);
                let src_bank = self.fetch(bus);
                self.dbr = dst_bank;
                let xm = if self.x8() { self.x & 0xFF } else { self.x };
                let ym = if self.x8() { self.y & 0xFF } else { self.y };
                let v = bus.read(((src_bank as u32) << 16) | xm as u32);
                bus.write(((dst_bank as u32) << 16) | ym as u32, v);
                bus.idle();
                bus.idle();
                // MVN/MVP はフラグ不変
                let delta: u16 = if op == 0x54 { 1 } else { 0xFFFF };
                if self.x8() {
                    self.x = (self.x as u8).wrapping_add(delta as u8) as u16;
                    self.y = (self.y as u8).wrapping_add(delta as u8) as u16;
                } else {
                    self.x = self.x.wrapping_add(delta);
                    self.y = self.y.wrapping_add(delta);
                }
                self.a = self.a.wrapping_sub(1);
                if self.a != 0xFFFF {
                    self.pc = self.pc.wrapping_sub(3);
                }
            }

            // ---- その他 ----
            0xEA => { bus.idle(); }
            0x42 => { self.fetch(bus); } // WDM
            0xEB => {
                bus.idle();
                bus.idle();
                self.a = self.a.rotate_left(8);
                self.set_nz8(self.a as u8);
            }
            0xFB => {
                bus.idle();
                let c = self.flag(FLAG_C);
                self.set_flag(FLAG_C, self.e);
                self.e = c;
                if self.e {
                    self.p |= FLAG_M | FLAG_X;
                    self.x &= 0xFF;
                    self.y &= 0xFF;
                    self.s = 0x0100 | (self.s & 0xFF);
                }
            }
            0xDB => {
                self.stopped = true;
                bus.idle();
                bus.idle();
            }
            0xCB => {
                self.waiting = true;
                bus.idle();
                bus.idle();
            }
        }
    }

    /// A への幅つき書き戻し (シフト系 A 変種用)。フラグは既に設定済み。
    fn set_a_w(&mut self, v: u16) {
        if self.m8() {
            self.a = (self.a & 0xFF00) | (v & 0xFF);
        } else {
            self.a = v;
        }
    }

    /// インデックスレジスタの加減算 (NZ 設定つき、幅対応)
    fn idx_add(&mut self, reg: u16, delta: u16) -> u16 {
        let r = if self.x8() {
            (reg as u8).wrapping_add(delta as u8) as u16
        } else {
            reg.wrapping_add(delta)
        };
        self.set_nz(r, self.x8());
        r
    }

    /// X フラグ幅でマスク
    fn width_x(&self, v: u16) -> u16 {
        if self.x8() {
            v & 0xFF
        } else {
            v
        }
    }
}

// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    struct TestBus {
        mem: Vec<u8>,
    }

    impl TestBus {
        fn new() -> Self {
            TestBus {
                mem: vec![0; 0x100_0000],
            }
        }

        fn load(&mut self, addr: u32, bytes: &[u8]) {
            self.mem[addr as usize..addr as usize + bytes.len()].copy_from_slice(bytes);
        }
    }

    impl Bus for TestBus {
        fn read(&mut self, addr: u32) -> u8 {
            self.mem[(addr & 0xFF_FFFF) as usize]
        }
        fn write(&mut self, addr: u32, data: u8) {
            self.mem[(addr & 0xFF_FFFF) as usize] = data;
        }
        fn idle(&mut self) {}
    }

    fn setup(prog: &[u8]) -> (Cpu, TestBus) {
        let mut bus = TestBus::new();
        bus.load(0x8000, prog);
        bus.load(0xFFFC, &[0x00, 0x80]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        (cpu, bus)
    }

    fn run(cpu: &mut Cpu, bus: &mut TestBus, n: usize) {
        for _ in 0..n {
            cpu.step(bus);
        }
    }

    #[test]
    fn lda_sta_emulation() {
        // LDA #$42; STA $10; LDA $10
        let (mut cpu, mut bus) = setup(&[0xA9, 0x42, 0x85, 0x10, 0xA9, 0x00, 0xA5, 0x10]);
        run(&mut cpu, &mut bus, 4);
        assert_eq!(cpu.a & 0xFF, 0x42);
        assert_eq!(bus.mem[0x10], 0x42);
        assert!(!cpu.flag(FLAG_Z));
    }

    #[test]
    fn native_16bit_adc() {
        // CLC; XCE; REP #$30; LDA #$1234; CLC; ADC #$4321
        let (mut cpu, mut bus) = setup(&[
            0x18, 0xFB, 0xC2, 0x30, 0xA9, 0x34, 0x12, 0x18, 0x69, 0x21, 0x43,
        ]);
        run(&mut cpu, &mut bus, 6);
        assert!(!cpu.e);
        assert!(!cpu.m8());
        assert_eq!(cpu.a, 0x5555);
        assert!(!cpu.flag(FLAG_C));
        assert!(!cpu.flag(FLAG_V));
    }

    #[test]
    fn bcd_adc() {
        // SED; LDA #$19; CLC; ADC #$28 => $47
        let (mut cpu, mut bus) = setup(&[0xF8, 0xA9, 0x19, 0x18, 0x69, 0x28]);
        run(&mut cpu, &mut bus, 4);
        assert_eq!(cpu.a & 0xFF, 0x47);
        assert!(!cpu.flag(FLAG_C));
    }

    #[test]
    fn bcd_sbc() {
        // SED; SEC; LDA #$46; SBC #$12 => $34
        let (mut cpu, mut bus) = setup(&[0xF8, 0x38, 0xA9, 0x46, 0xE9, 0x12]);
        run(&mut cpu, &mut bus, 4);
        assert_eq!(cpu.a & 0xFF, 0x34);
        assert!(cpu.flag(FLAG_C));
    }

    #[test]
    fn jsr_rts() {
        // JSR $8010; BRK padding...; at $8010: LDA #$55; RTS
        let mut prog = vec![0x20, 0x10, 0x80, 0xA9, 0x99];
        prog.resize(0x10, 0xEA);
        prog.extend_from_slice(&[0xA9, 0x55, 0x60]);
        let (mut cpu, mut bus) = setup(&prog);
        run(&mut cpu, &mut bus, 3); // JSR, LDA #$55, RTS
        assert_eq!(cpu.pc, 0x8003);
        assert_eq!(cpu.a & 0xFF, 0x55);
        run(&mut cpu, &mut bus, 1); // LDA #$99
        assert_eq!(cpu.a & 0xFF, 0x99);
        assert_eq!(cpu.s, 0x01FF);
    }

    #[test]
    fn branch_loop() {
        // LDX #$05; loop: DEX; BNE loop
        let (mut cpu, mut bus) = setup(&[0xA2, 0x05, 0xCA, 0xD0, 0xFD]);
        run(&mut cpu, &mut bus, 1 + 5 * 2);
        assert_eq!(cpu.x, 0);
        assert!(cpu.flag(FLAG_Z));
        assert_eq!(cpu.pc, 0x8005);
    }

    #[test]
    fn block_move_mvn() {
        // 転送元 $7E1000 に 4 バイト置き、$7E2000 へ MVN
        // CLC; XCE; REP #$30; LDA #$0003; LDX #$1000; LDY #$2000; MVN $7E,$7E
        let (mut cpu, mut bus) = setup(&[
            0x18, 0xFB, 0xC2, 0x30, 0xA9, 0x03, 0x00, 0xA2, 0x00, 0x10, 0xA0, 0x00, 0x20, 0x54,
            0x7E, 0x7E,
        ]);
        bus.load(0x7E1000, &[0xDE, 0xAD, 0xBE, 0xEF]);
        run(&mut cpu, &mut bus, 6 + 4);
        assert_eq!(&bus.mem[0x7E2000..0x7E2004], &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(cpu.a, 0xFFFF);
        assert_eq!(cpu.x, 0x1004);
        assert_eq!(cpu.y, 0x2004);
        assert_eq!(cpu.dbr, 0x7E);
    }

    #[test]
    fn stack_16bit_push_pull() {
        // CLC; XCE; REP #$30; LDA #$ABCD; PHA; LDA #$0000; PLA
        let (mut cpu, mut bus) = setup(&[
            0x18, 0xFB, 0xC2, 0x30, 0xA9, 0xCD, 0xAB, 0x48, 0xA9, 0x00, 0x00, 0x68,
        ]);
        run(&mut cpu, &mut bus, 7);
        assert_eq!(cpu.a, 0xABCD);
        assert!(cpu.flag(FLAG_N));
    }

    #[test]
    fn interrupt_nmi_native() {
        struct NmiBus {
            inner: TestBus,
            nmi: bool,
        }
        impl Bus for NmiBus {
            fn read(&mut self, a: u32) -> u8 {
                self.inner.read(a)
            }
            fn write(&mut self, a: u32, v: u8) {
                self.inner.write(a, v)
            }
            fn idle(&mut self) {}
            fn nmi_pending(&mut self) -> bool {
                std::mem::take(&mut self.nmi)
            }
        }
        let mut bus = NmiBus {
            inner: TestBus::new(),
            nmi: false,
        };
        // メイン: CLC; XCE; NOP...  NMI ハンドラ ($9000): LDA #$77; RTI
        bus.inner.load(0x8000, &[0x18, 0xFB, 0xEA, 0xEA, 0xEA]);
        bus.inner.load(0x9000, &[0xA9, 0x77, 0x40]);
        bus.inner.load(0xFFFC, &[0x00, 0x80]);
        bus.inner.load(0xFFEA, &[0x00, 0x90]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus); // CLC
        cpu.step(&mut bus); // XCE -> native
        bus.nmi = true;
        cpu.step(&mut bus); // NMI 受理
        assert_eq!(cpu.pc, 0x9000);
        cpu.step(&mut bus); // LDA #$77 (M=8bit のまま)
        assert_eq!(cpu.a & 0xFF, 0x77);
        cpu.step(&mut bus); // RTI
        assert_eq!(cpu.pc, 0x8002);
        assert_eq!(cpu.pbr, 0x00);
    }
}
