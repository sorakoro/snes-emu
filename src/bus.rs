//! メインバス: メモリマップ、CPU I/O レジスタ、DMA、ジョイパッド、
//! マスターサイクル計上とスキャンラインタイミング。

use crate::apu::Apu;
use crate::cartridge::Cartridge;
use crate::cpu;
use crate::ppu::Ppu;

pub const CYCLES_PER_LINE: u64 = 1364;
pub const LINES_PER_FRAME: u16 = 262;
pub const VBLANK_START_LINE: u16 = 225;

#[derive(Default, Clone, Copy)]
struct DmaChannel {
    param: u8,
    b_addr: u8,
    a_addr: u16,
    a_bank: u8,
    size: u16, // DAS: HDMA では間接アドレス
    ind_bank: u8,
    hdma_table: u16, // HDMA の現在テーブルポインタ
    hdma_line: u8,   // ラインカウンタ (bit7 = リピート)
    hdma_done: bool,
    hdma_do_transfer: bool,
}

/// DMA 転送モードごとの B バスアドレスオフセットパターン
const DMA_PATTERNS: [&[u8]; 8] = [
    &[0],
    &[0, 1],
    &[0, 0],
    &[0, 0, 1, 1],
    &[0, 1, 2, 3],
    &[0, 1, 0, 1],
    &[0, 0],
    &[0, 0, 1, 1],
];

pub struct MainBus {
    pub cart: Cartridge,
    pub wram: Box<[u8; 0x20000]>,
    pub ppu: Ppu,

    pub cycles: u64,
    next_line_at: u64,
    pub scanline: u16,
    pub frame: u64,
    pub vblank: bool,

    // 割り込み
    pub nmitimen: u8,
    nmi_flag: bool,
    nmi_edge: bool,
    pub irq_flag: bool,
    pub htime: u16,
    pub vtime: u16,

    // その他 CPU I/O
    fastrom: bool,
    wrio: u8,
    mul_a: u8,
    rddiv: u16,
    rdmpy: u16,
    div_a: u16,
    wram_addr: u32,
    open_bus: u8,

    // ジョイパッド
    pub joy1: u16,
    pub joy2: u16,
    joy_strobe: bool,
    joy1_shift: u32,
    joy2_shift: u32,
    joy_auto: [u16; 4],

    pub apu: Apu,

    dma: [DmaChannel; 8],
    hdma_enable: u8,
}

impl MainBus {
    pub fn new(cart: Cartridge) -> Self {
        MainBus {
            cart,
            wram: Box::new([0; 0x20000]),
            ppu: Ppu::new(),
            cycles: 0,
            next_line_at: CYCLES_PER_LINE,
            scanline: 0,
            frame: 0,
            vblank: false,
            nmitimen: 0,
            nmi_flag: false,
            nmi_edge: false,
            irq_flag: false,
            htime: 0x1FF,
            vtime: 0x1FF,
            fastrom: false,
            wrio: 0xFF,
            mul_a: 0xFF,
            rddiv: 0,
            rdmpy: 0,
            div_a: 0,
            wram_addr: 0,
            open_bus: 0,
            joy1: 0,
            joy2: 0,
            joy_strobe: false,
            joy1_shift: 0,
            joy2_shift: 0,
            joy_auto: [0; 4],
            apu: Apu::new(),
            dma: [DmaChannel::default(); 8],
            hdma_enable: 0,
        }
    }

    // ---- タイミング --------------------------------------------------------

    fn add_cycles(&mut self, n: u64) {
        self.cycles += n;
        while self.cycles >= self.next_line_at {
            self.next_line_at += CYCLES_PER_LINE;
            self.advance_line();
        }
    }

    fn advance_line(&mut self) {
        // APU をライン単位で追従させ、サンプル生成を途切れさせない
        self.apu.run_to(self.cycles);
        self.scanline += 1;
        if self.scanline == LINES_PER_FRAME {
            self.scanline = 0;
            self.frame += 1;
        }
        self.ppu.scanline = self.scanline;

        if self.scanline == 0 {
            // 新フレーム開始
            self.vblank = false;
            self.nmi_flag = false;
            self.ppu.vblank = false;
            self.hdma_init();
        }
        if (1..=224).contains(&self.scanline) {
            self.hdma_run_line();
            self.ppu.render_scanline(self.scanline);
        }
        if self.scanline == VBLANK_START_LINE {
            self.vblank = true;
            self.ppu.vblank = true;
            self.ppu.frame += 1;
            self.ppu.on_vblank_start();
            self.nmi_flag = true;
            if self.nmitimen & 0x80 != 0 {
                self.nmi_edge = true;
            }
            if self.nmitimen & 0x01 != 0 {
                self.auto_joypad_read();
            }
        }

        // H/V IRQ (現状ライン粒度の近似。$4200 bit4 = H 有効, bit5 = V 有効)
        let h_en = self.nmitimen & 0x10 != 0;
        let v_en = self.nmitimen & 0x20 != 0;
        if (v_en && !h_en && self.scanline == self.vtime)
            || (v_en && h_en && self.scanline == self.vtime)
            || (h_en && !v_en)
        {
            self.irq_flag = true;
        }
    }

    fn auto_joypad_read(&mut self) {
        self.joy_auto[0] = self.joy1;
        self.joy_auto[1] = self.joy2;
        self.joy_auto[2] = 0;
        self.joy_auto[3] = 0;
        // 自動読み取りはコントローラのシフトレジスタを 16 クロック消費するので、
        // 以降の $4016/$4017 シリアル読みは 1 を返す (DQ6 等はこれで接続判定する)
        self.joy1_shift = 0xFFFF_FFFF;
        self.joy2_shift = 0xFFFF_FFFF;
    }

    // ---- メモリルーティング -------------------------------------------------

    fn read_mem(&mut self, addr: u32) -> u8 {
        let bank = (addr >> 16) as u8;
        let off = addr as u16;
        match bank {
            0x7E | 0x7F => self.wram[(addr - 0x7E_0000) as usize],
            0x00..=0x3F | 0x80..=0xBF => match off {
                0x0000..=0x1FFF => self.wram[off as usize],
                0x2100..=0x2133 => self.open_bus,
                0x2134..=0x213F => self.ppu.read(off),
                0x2140..=0x217F => {
                    self.apu.run_to(self.cycles);
                    self.apu.inner.out_ports[(off & 3) as usize]
                }
                0x2180 => {
                    let v = self.wram[self.wram_addr as usize];
                    self.wram_addr = (self.wram_addr + 1) & 0x1_FFFF;
                    v
                }
                0x4016 => self.joy_serial_read(0),
                0x4017 => self.joy_serial_read(1),
                0x4200..=0x421F => self.cpu_io_read(off),
                0x4300..=0x437F => self.dma_read(off),
                _ => self.cart.read(bank, off).unwrap_or(self.open_bus),
            },
            _ => self.cart.read(bank, off).unwrap_or(self.open_bus),
        }
    }

    fn write_mem(&mut self, addr: u32, v: u8) {
        let bank = (addr >> 16) as u8;
        let off = addr as u16;
        match bank {
            0x7E | 0x7F => self.wram[(addr - 0x7E_0000) as usize] = v,
            0x00..=0x3F | 0x80..=0xBF => match off {
                0x0000..=0x1FFF => self.wram[off as usize] = v,
                0x2100..=0x2133 => self.ppu.write(off, v),
                0x2140..=0x217F => {
                    self.apu.run_to(self.cycles);
                    self.apu.inner.in_ports[(off & 3) as usize] = v;
                }
                0x2180 => {
                    self.wram[self.wram_addr as usize] = v;
                    self.wram_addr = (self.wram_addr + 1) & 0x1_FFFF;
                }
                0x2181 => self.wram_addr = (self.wram_addr & 0x1_FF00) | v as u32,
                0x2182 => self.wram_addr = (self.wram_addr & 0x1_00FF) | ((v as u32) << 8),
                0x2183 => self.wram_addr = (self.wram_addr & 0x0_FFFF) | (((v & 1) as u32) << 16),
                0x4016 => {
                    let strobe = v & 1 != 0;
                    if self.joy_strobe && !strobe {
                        self.joy1_shift = ((self.joy1 as u32) << 16) | 0xFFFF;
                        self.joy2_shift = ((self.joy2 as u32) << 16) | 0xFFFF;
                    }
                    self.joy_strobe = strobe;
                }
                0x4200..=0x421F => self.cpu_io_write(off, v),
                0x4300..=0x437F => self.dma_write(off, v),
                _ => self.cart.write(bank, off, v),
            },
            _ => self.cart.write(bank, off, v),
        }
    }

    fn joy_serial_read(&mut self, pad: usize) -> u8 {
        if self.joy_strobe {
            self.joy1_shift = ((self.joy1 as u32) << 16) | 0xFFFF;
            self.joy2_shift = ((self.joy2 as u32) << 16) | 0xFFFF;
        }
        let shift = if pad == 0 {
            &mut self.joy1_shift
        } else {
            &mut self.joy2_shift
        };
        let bit = (*shift >> 31) as u8;
        *shift = (*shift << 1) | 1; // 16 ビット読み切った後は 1 が返る
        bit
    }

    // ---- CPU I/O ($4200-$421F) ---------------------------------------------

    fn cpu_io_read(&mut self, off: u16) -> u8 {
        match off {
            0x4210 => {
                let v = (if self.nmi_flag { 0x80 } else { 0 }) | 0x02;
                self.nmi_flag = false;
                v
            }
            0x4211 => {
                let v = if self.irq_flag { 0x80 } else { 0 };
                self.irq_flag = false;
                v
            }
            0x4212 => {
                let mut v = 0u8;
                if self.vblank {
                    v |= 0x80;
                }
                // H-Blank 近似: ライン終端 268 マスターサイクル
                let in_line = self.cycles + CYCLES_PER_LINE - self.next_line_at;
                if in_line >= 1096 {
                    v |= 0x40;
                }
                v
            }
            0x4213 => self.wrio,
            0x4214 => self.rddiv as u8,
            0x4215 => (self.rddiv >> 8) as u8,
            0x4216 => self.rdmpy as u8,
            0x4217 => (self.rdmpy >> 8) as u8,
            0x4218..=0x421F => {
                let i = ((off - 0x4218) / 2) as usize;
                let w = self.joy_auto[i];
                if off & 1 == 0 {
                    w as u8
                } else {
                    (w >> 8) as u8
                }
            }
            _ => self.open_bus,
        }
    }

    fn cpu_io_write(&mut self, off: u16, v: u8) {
        match off {
            0x4200 => {
                let rising = v & 0x80 != 0 && self.nmitimen & 0x80 == 0;
                self.nmitimen = v;
                if rising && self.nmi_flag {
                    self.nmi_edge = true;
                }
            }
            0x4201 => self.wrio = v,
            0x4202 => self.mul_a = v,
            0x4203 => {
                self.rdmpy = self.mul_a as u16 * v as u16;
            }
            0x4204 => self.div_a = (self.div_a & 0xFF00) | v as u16,
            0x4205 => self.div_a = (self.div_a & 0x00FF) | ((v as u16) << 8),
            0x4206 => {
                if v == 0 {
                    self.rddiv = 0xFFFF;
                    self.rdmpy = self.div_a;
                } else {
                    self.rddiv = self.div_a / v as u16;
                    self.rdmpy = self.div_a % v as u16;
                }
            }
            0x4207 => self.htime = (self.htime & 0x100) | v as u16,
            0x4208 => self.htime = (self.htime & 0x0FF) | (((v & 1) as u16) << 8),
            0x4209 => self.vtime = (self.vtime & 0x100) | v as u16,
            0x420A => self.vtime = (self.vtime & 0x0FF) | (((v & 1) as u16) << 8),
            0x420B => self.run_mdma(v),
            0x420C => self.hdma_enable = v,
            0x420D => self.fastrom = v & 1 != 0,
            _ => {}
        }
    }

    // ---- DMA ----------------------------------------------------------------

    fn dma_read(&mut self, off: u16) -> u8 {
        let ch = &self.dma[((off >> 4) & 7) as usize];
        match off & 0xF {
            0x0 => ch.param,
            0x1 => ch.b_addr,
            0x2 => ch.a_addr as u8,
            0x3 => (ch.a_addr >> 8) as u8,
            0x4 => ch.a_bank,
            0x5 => ch.size as u8,
            0x6 => (ch.size >> 8) as u8,
            0x7 => ch.ind_bank,
            0x8 => ch.hdma_table as u8,
            0x9 => (ch.hdma_table >> 8) as u8,
            0xA => ch.hdma_line,
            _ => self.open_bus,
        }
    }

    fn dma_write(&mut self, off: u16, v: u8) {
        let ch = &mut self.dma[((off >> 4) & 7) as usize];
        match off & 0xF {
            0x0 => ch.param = v,
            0x1 => ch.b_addr = v,
            0x2 => ch.a_addr = (ch.a_addr & 0xFF00) | v as u16,
            0x3 => ch.a_addr = (ch.a_addr & 0x00FF) | ((v as u16) << 8),
            0x4 => ch.a_bank = v,
            0x5 => ch.size = (ch.size & 0xFF00) | v as u16,
            0x6 => ch.size = (ch.size & 0x00FF) | ((v as u16) << 8),
            0x7 => ch.ind_bank = v,
            0x8 => ch.hdma_table = (ch.hdma_table & 0xFF00) | v as u16,
            0x9 => ch.hdma_table = (ch.hdma_table & 0x00FF) | ((v as u16) << 8),
            0xA => ch.hdma_line = v,
            _ => {}
        }
    }

    // ---- HDMA -----------------------------------------------------------

    fn hdma_init(&mut self) {
        for i in 0..8 {
            if self.hdma_enable & (1 << i) == 0 {
                self.dma[i].hdma_done = true;
                continue;
            }
            self.dma[i].hdma_table = self.dma[i].a_addr;
            self.dma[i].hdma_done = false;
            self.hdma_load_entry(i);
        }
    }

    /// テーブルから次のエントリ (ラインカウンタ + 間接アドレス) を読む
    fn hdma_load_entry(&mut self, i: usize) {
        let bank = self.dma[i].a_bank;
        let addr = self.dma[i].hdma_table;
        let v = self.read_mem(((bank as u32) << 16) | addr as u32);
        self.dma[i].hdma_table = addr.wrapping_add(1);
        if v == 0 {
            self.dma[i].hdma_done = true;
            self.dma[i].hdma_do_transfer = false;
            return;
        }
        self.dma[i].hdma_line = v;
        if self.dma[i].param & 0x40 != 0 {
            // 間接モード: 2 バイトの実データアドレスを読む
            let t = self.dma[i].hdma_table;
            let lo = self.read_mem(((bank as u32) << 16) | t as u32) as u16;
            let hi = self.read_mem(((bank as u32) << 16) | t.wrapping_add(1) as u32) as u16;
            self.dma[i].size = (hi << 8) | lo;
            self.dma[i].hdma_table = t.wrapping_add(2);
        }
        self.dma[i].hdma_do_transfer = true;
    }

    fn hdma_run_line(&mut self) {
        for i in 0..8 {
            if self.hdma_enable & (1 << i) == 0 || self.dma[i].hdma_done {
                continue;
            }
            if self.dma[i].hdma_do_transfer {
                let ch = self.dma[i];
                let pattern = DMA_PATTERNS[(ch.param & 7) as usize];
                let indirect = ch.param & 0x40 != 0;
                for (k, &boff) in pattern.iter().enumerate() {
                    let src = if indirect {
                        let a = ((ch.ind_bank as u32) << 16)
                            | ch.size.wrapping_add(k as u16) as u32;
                        a
                    } else {
                        ((ch.a_bank as u32) << 16)
                            | ch.hdma_table.wrapping_add(k as u16) as u32
                    };
                    let v = self.read_mem(src);
                    self.write_mem(0x2100 | ch.b_addr.wrapping_add(boff) as u32, v);
                    self.cycles += 8;
                }
                let n = pattern.len() as u16;
                if indirect {
                    self.dma[i].size = self.dma[i].size.wrapping_add(n);
                } else {
                    self.dma[i].hdma_table = self.dma[i].hdma_table.wrapping_add(n);
                }
            }
            self.dma[i].hdma_line = self.dma[i].hdma_line.wrapping_sub(1);
            self.dma[i].hdma_do_transfer = self.dma[i].hdma_line & 0x80 != 0;
            if self.dma[i].hdma_line & 0x7F == 0 {
                self.hdma_load_entry(i);
            }
        }
    }

    fn run_mdma(&mut self, mask: u8) {
        self.add_cycles(8);
        for i in 0..8 {
            if mask & (1 << i) == 0 {
                continue;
            }
            self.add_cycles(8);
            let ch = self.dma[i];
            let pattern = DMA_PATTERNS[(ch.param & 7) as usize];
            let b_to_a = ch.param & 0x80 != 0;
            let fixed = ch.param & 0x08 != 0;
            let dec = ch.param & 0x10 != 0;

            let mut count: u32 = if ch.size == 0 { 0x1_0000 } else { ch.size as u32 };
            let mut a_addr = ch.a_addr;
            'transfer: loop {
                for &boff in pattern {
                    let a = ((ch.a_bank as u32) << 16) | a_addr as u32;
                    let b = 0x2100 | ch.b_addr.wrapping_add(boff) as u32;
                    if b_to_a {
                        let v = self.read_mem(b);
                        self.write_mem(a, v);
                    } else {
                        let v = self.read_mem(a);
                        self.write_mem(b, v);
                    }
                    if !fixed {
                        a_addr = if dec {
                            a_addr.wrapping_sub(1)
                        } else {
                            a_addr.wrapping_add(1)
                        };
                    }
                    self.add_cycles(8);
                    count -= 1;
                    if count == 0 {
                        break 'transfer;
                    }
                }
            }
            self.dma[i].a_addr = a_addr;
            self.dma[i].size = 0;
        }
    }

    // ---- アクセス速度 (マスターサイクル) --------------------------------------

    fn access_speed(&self, addr: u32) -> u64 {
        let bank = (addr >> 16) as u8;
        let off = addr as u16;
        match bank {
            0x00..=0x3F => Self::system_area_speed(off, 8),
            0x40..=0x7F => 8,
            0x80..=0xBF => {
                Self::system_area_speed(off, if self.fastrom { 6 } else { 8 })
            }
            0xC0..=0xFF => {
                if self.fastrom {
                    6
                } else {
                    8
                }
            }
        }
    }

    fn system_area_speed(off: u16, rom_speed: u64) -> u64 {
        match off {
            0x0000..=0x1FFF => 8,
            0x2000..=0x3FFF => 6,
            0x4000..=0x41FF => 12,
            0x4200..=0x5FFF => 6,
            0x6000..=0x7FFF => 8,
            _ => rom_speed,
        }
    }
}

impl cpu::Bus for MainBus {
    fn read(&mut self, addr: u32) -> u8 {
        let addr = addr & 0xFF_FFFF;
        self.add_cycles(self.access_speed(addr));
        let v = self.read_mem(addr);
        self.open_bus = v;
        v
    }

    fn write(&mut self, addr: u32, data: u8) {
        let addr = addr & 0xFF_FFFF;
        self.add_cycles(self.access_speed(addr));
        self.open_bus = data;
        self.write_mem(addr, data);
    }

    fn idle(&mut self) {
        self.add_cycles(6);
    }

    fn nmi_pending(&mut self) -> bool {
        std::mem::take(&mut self.nmi_edge)
    }

    fn irq_level(&self) -> bool {
        self.irq_flag
    }
}
