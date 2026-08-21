//! PPU。現段階ではレジスタ / VRAM / CGRAM / OAM のポート動作と
//! フレームタイミングを実装し、描画はバックドロップ色のみ (フェーズ2 で本実装)。

pub const SCREEN_W: usize = 256;
pub const SCREEN_H: usize = 224;

pub struct Ppu {
    pub vram: [u8; 0x10000],
    pub cgram: [u16; 256],
    pub oam: [u8; 544],

    // $2100 INIDISP
    pub forced_blank: bool,
    pub brightness: u8,

    // BG 設定 (フェーズ2 で使用)
    pub bg_mode: u8,
    pub bg3_priority: bool,
    pub tile_size_bits: u8,
    pub obsel: u8,
    pub bg_sc: [u8; 4],     // $2107-210A
    pub bg_chr: [u8; 2],    // $210B-210C
    pub bg_hofs: [u16; 4],
    pub bg_vofs: [u16; 4],
    pub tm: u8,
    pub ts: u8,
    pub mosaic: u8,
    pub cgwsel: u8,
    pub cgadsub: u8,
    pub fixed_color: u16,
    pub setini: u8,
    pub w12sel: u8,
    pub w34sel: u8,
    pub wobjsel: u8,
    pub wh: [u8; 4],
    pub wbglog: u8,
    pub wobjlog: u8,
    pub tmw: u8,
    pub tsw: u8,

    // Mode 7
    pub m7sel: u8,
    pub m7: [i16; 4], // A,B,C,D
    pub m7x: i16,
    pub m7y: i16,
    pub m7_hofs: i16,
    pub m7_vofs: i16,
    m7_latch: u8,

    // VRAM ポート
    vmain: u8,
    vmaddr: u16,
    vram_prefetch: u16,

    // CGRAM ポート
    cgadd: u8,
    cg_latch: Option<u8>,
    cg_read_latch: Option<u8>,

    // OAM ポート
    oamaddr_reload: u16,
    oamaddr: u16,
    oam_latch: u8,

    // スクロールレジスタの書き込みラッチ
    bgofs_latch: u8,
    bghofs_latch: u8,

    // カウンタラッチ
    latched_h: u16,
    latched_v: u16,
    counter_latched: bool,
    ophct_flip: bool,
    opvct_flip: bool,

    // タイミング
    pub scanline: u16,
    pub frame: u64,
    pub vblank: bool,

    pub framebuffer: Vec<u32>,
}

impl Default for Ppu {
    fn default() -> Self {
        Self::new()
    }
}

impl Ppu {
    pub fn new() -> Self {
        Ppu {
            vram: [0; 0x10000],
            cgram: [0; 256],
            oam: [0; 544],
            forced_blank: true,
            brightness: 0,
            bg_mode: 0,
            bg3_priority: false,
            tile_size_bits: 0,
            obsel: 0,
            bg_sc: [0; 4],
            bg_chr: [0; 2],
            bg_hofs: [0; 4],
            bg_vofs: [0; 4],
            tm: 0,
            ts: 0,
            mosaic: 0,
            cgwsel: 0,
            cgadsub: 0,
            fixed_color: 0,
            setini: 0,
            w12sel: 0,
            w34sel: 0,
            wobjsel: 0,
            wh: [0; 4],
            wbglog: 0,
            wobjlog: 0,
            tmw: 0,
            tsw: 0,
            m7sel: 0,
            m7: [0; 4],
            m7x: 0,
            m7y: 0,
            m7_hofs: 0,
            m7_vofs: 0,
            m7_latch: 0,
            vmain: 0,
            vmaddr: 0,
            vram_prefetch: 0,
            cgadd: 0,
            cg_latch: None,
            cg_read_latch: None,
            oamaddr_reload: 0,
            oamaddr: 0,
            oam_latch: 0,
            bgofs_latch: 0,
            bghofs_latch: 0,
            latched_h: 0,
            latched_v: 0,
            counter_latched: false,
            ophct_flip: false,
            opvct_flip: false,
            scanline: 0,
            frame: 0,
            vblank: false,
            framebuffer: vec![0; SCREEN_W * SCREEN_H],
        }
    }

    // ---- レジスタ書き込み ($2100-$2133) ----------------------------------

    pub fn write(&mut self, addr: u16, v: u8) {
        match addr {
            0x2100 => {
                self.forced_blank = v & 0x80 != 0;
                self.brightness = v & 0x0F;
            }
            0x2101 => self.obsel = v,
            0x2102 => {
                self.oamaddr_reload = (self.oamaddr_reload & 0x0200) | ((v as u16) << 1);
                self.oamaddr = self.oamaddr_reload;
            }
            0x2103 => {
                self.oamaddr_reload =
                    (self.oamaddr_reload & 0x01FE) | (((v & 1) as u16) << 9);
                self.oamaddr = self.oamaddr_reload;
            }
            0x2104 => self.oam_write(v),
            0x2105 => {
                self.bg_mode = v & 7;
                self.bg3_priority = v & 8 != 0;
                self.tile_size_bits = v >> 4;
            }
            0x2106 => self.mosaic = v,
            0x2107..=0x210A => self.bg_sc[(addr - 0x2107) as usize] = v,
            0x210B => self.bg_chr[0] = v,
            0x210C => self.bg_chr[1] = v,
            0x210D => {
                // BG1HOFS は Mode7 の M7HOFS も兼ねる
                self.m7_hofs = Self::m7_ext13(((v as u16) << 8) | self.m7_latch as u16);
                self.m7_latch = v;
                self.write_bg_hofs(0, v);
            }
            0x210E => {
                self.m7_vofs = Self::m7_ext13(((v as u16) << 8) | self.m7_latch as u16);
                self.m7_latch = v;
                self.write_bg_vofs(0, v);
            }
            0x210F => self.write_bg_hofs(1, v),
            0x2110 => self.write_bg_vofs(1, v),
            0x2111 => self.write_bg_hofs(2, v),
            0x2112 => self.write_bg_vofs(2, v),
            0x2113 => self.write_bg_hofs(3, v),
            0x2114 => self.write_bg_vofs(3, v),
            0x2115 => self.vmain = v,
            0x2116 => {
                self.vmaddr = (self.vmaddr & 0xFF00) | v as u16;
                self.vram_prefetch = self.vram_read_word(self.vmaddr);
            }
            0x2117 => {
                self.vmaddr = (self.vmaddr & 0x00FF) | ((v as u16) << 8);
                self.vram_prefetch = self.vram_read_word(self.vmaddr);
            }
            0x2118 => {
                let a = self.vram_translate() as usize * 2;
                self.vram[a & 0xFFFF] = v;
                if self.vmain & 0x80 == 0 {
                    self.vmaddr = self.vmaddr.wrapping_add(self.vram_step());
                }
            }
            0x2119 => {
                let a = self.vram_translate() as usize * 2 + 1;
                self.vram[a & 0xFFFF] = v;
                if self.vmain & 0x80 != 0 {
                    self.vmaddr = self.vmaddr.wrapping_add(self.vram_step());
                }
            }
            0x211A => self.m7sel = v,
            0x211B..=0x211E => {
                let i = (addr - 0x211B) as usize;
                self.m7[i] = (((v as u16) << 8) | self.m7_latch as u16) as i16;
                self.m7_latch = v;
            }
            0x211F => {
                self.m7x = Self::m7_ext13(((v as u16) << 8) | self.m7_latch as u16);
                self.m7_latch = v;
            }
            0x2120 => {
                self.m7y = Self::m7_ext13(((v as u16) << 8) | self.m7_latch as u16);
                self.m7_latch = v;
            }
            0x2121 => {
                self.cgadd = v;
                self.cg_latch = None;
                self.cg_read_latch = None;
            }
            0x2122 => {
                if let Some(lo) = self.cg_latch.take() {
                    self.cgram[self.cgadd as usize] = ((v as u16 & 0x7F) << 8) | lo as u16;
                    self.cgadd = self.cgadd.wrapping_add(1);
                } else {
                    self.cg_latch = Some(v);
                }
            }
            0x2123 => self.w12sel = v,
            0x2124 => self.w34sel = v,
            0x2125 => self.wobjsel = v,
            0x2126..=0x2129 => self.wh[(addr - 0x2126) as usize] = v,
            0x212A => self.wbglog = v,
            0x212B => self.wobjlog = v,
            0x212C => self.tm = v,
            0x212D => self.ts = v,
            0x212E => self.tmw = v,
            0x212F => self.tsw = v,
            0x2130 => self.cgwsel = v,
            0x2131 => self.cgadsub = v,
            0x2132 => {
                let c = (v & 0x1F) as u16;
                if v & 0x20 != 0 {
                    self.fixed_color = (self.fixed_color & !0x001F) | c;
                }
                if v & 0x40 != 0 {
                    self.fixed_color = (self.fixed_color & !0x03E0) | (c << 5);
                }
                if v & 0x80 != 0 {
                    self.fixed_color = (self.fixed_color & !0x7C00) | (c << 10);
                }
            }
            0x2133 => self.setini = v,
            _ => {}
        }
    }

    // ---- レジスタ読み出し ($2134-$213F) -----------------------------------

    pub fn read(&mut self, addr: u16) -> u8 {
        match addr {
            // Mode7 乗算結果 (M7A × M7B 上位バイト)
            0x2134..=0x2136 => {
                let r = (self.m7[0] as i32) * ((self.m7[1] >> 8) as i8 as i32);
                (r >> (8 * (addr - 0x2134))) as u8
            }
            0x2137 => {
                // SLHV: H/V カウンタラッチ
                self.latched_h = 0; // ドット単位の H は未実装
                self.latched_v = self.scanline;
                self.counter_latched = true;
                0
            }
            0x2138 => {
                let a = (self.oamaddr as usize).min(543);
                let v = if a < 0x200 {
                    self.oam[a]
                } else {
                    self.oam[0x200 + (a & 0x1F)]
                };
                self.oamaddr = (self.oamaddr + 1) & 0x3FF;
                v
            }
            0x2139 => {
                let v = self.vram_prefetch as u8;
                if self.vmain & 0x80 == 0 {
                    self.vram_prefetch = self.vram_read_word(self.vmaddr);
                    self.vmaddr = self.vmaddr.wrapping_add(self.vram_step());
                }
                v
            }
            0x213A => {
                let v = (self.vram_prefetch >> 8) as u8;
                if self.vmain & 0x80 != 0 {
                    self.vram_prefetch = self.vram_read_word(self.vmaddr);
                    self.vmaddr = self.vmaddr.wrapping_add(self.vram_step());
                }
                v
            }
            0x213B => {
                let word = self.cgram[self.cgadd as usize];
                if let Some(_) = self.cg_read_latch.take() {
                    self.cgadd = self.cgadd.wrapping_add(1);
                    (word >> 8) as u8
                } else {
                    self.cg_read_latch = Some(0);
                    word as u8
                }
            }
            0x213C => {
                self.ophct_flip = !self.ophct_flip;
                if self.ophct_flip {
                    self.latched_h as u8
                } else {
                    (self.latched_h >> 8) as u8
                }
            }
            0x213D => {
                self.opvct_flip = !self.opvct_flip;
                if self.opvct_flip {
                    self.latched_v as u8
                } else {
                    (self.latched_v >> 8) as u8
                }
            }
            0x213E => 0x01, // STAT77: PPU1 バージョン
            0x213F => {
                let v = 0x02 | if self.counter_latched { 0x40 } else { 0 };
                self.counter_latched = false;
                self.ophct_flip = false;
                self.opvct_flip = false;
                v
            }
            _ => 0,
        }
    }

    // ---- 内部ヘルパ --------------------------------------------------------

    fn write_bg_hofs(&mut self, bg: usize, v: u8) {
        self.bg_hofs[bg] = ((v as u16) << 8)
            | (self.bghofs_latch as u16 & !7)
            | ((self.bg_hofs[bg] >> 8) & 7);
        self.bghofs_latch = v;
        self.bgofs_latch = v;
    }

    fn write_bg_vofs(&mut self, bg: usize, v: u8) {
        self.bg_vofs[bg] = ((v as u16) << 8) | self.bgofs_latch as u16;
        self.bgofs_latch = v;
    }

    /// Mode7 の 13bit 符号拡張
    fn m7_ext13(v: u16) -> i16 {
        ((v << 3) as i16) >> 3
    }

    fn vram_step(&self) -> u16 {
        match self.vmain & 3 {
            0 => 1,
            1 => 32,
            _ => 128,
        }
    }

    /// VRAM アドレスリマップ (VMAIN ビット 2-3)
    fn vram_translate(&self) -> u16 {
        let a = self.vmaddr;
        match (self.vmain >> 2) & 3 {
            0 => a,
            1 => (a & 0xFF00) | ((a & 0x001F) << 3) | ((a >> 5) & 7),
            2 => (a & 0xFE00) | ((a & 0x003F) << 3) | ((a >> 6) & 7),
            _ => (a & 0xFC00) | ((a & 0x007F) << 3) | ((a >> 7) & 7),
        }
    }

    fn vram_read_word(&self, addr: u16) -> u16 {
        let a = (addr as usize * 2) & 0xFFFF;
        u16::from_le_bytes([self.vram[a], self.vram[a | 1]])
    }

    fn oam_write(&mut self, v: u8) {
        let a = self.oamaddr as usize;
        if a < 0x200 {
            if a & 1 == 0 {
                self.oam_latch = v;
            } else {
                self.oam[a - 1] = self.oam_latch;
                self.oam[a] = v;
            }
        } else {
            self.oam[0x200 + (a & 0x1F)] = v;
        }
        self.oamaddr = (self.oamaddr + 1) & 0x3FF;
    }

    // ---- 描画 (本体は ppu_render.rs) ----------------------------------------

    pub fn render_scanline(&mut self, line: u16) {
        self.render_scanline_full(line);
    }

    pub fn rgb555_to_argb(c: u16, brightness: u8) -> u32 {
        let scale = |v: u16| -> u32 {
            let v8 = ((v & 0x1F) as u32 * 255 / 31) * (brightness as u32) / 15;
            v8
        };
        let r = scale(c);
        let g = scale(c >> 5);
        let b = scale(c >> 10);
        0xFF00_0000 | (r << 16) | (g << 8) | b
    }

    /// V-Blank 開始時に OAM アドレスをリロード (実機挙動)
    pub fn on_vblank_start(&mut self) {
        if !self.forced_blank {
            self.oamaddr = self.oamaddr_reload;
        }
    }
}
