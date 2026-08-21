//! PPU スキャンラインレンダラ。
//!
//! ラインバッファ方式: 各 BG レイヤと OBJ を一旦ラインバッファに描き、
//! モード別の優先順位テーブルに従ってメイン/サブスクリーンを合成し、
//! ウィンドウとカラー演算を適用する。

use crate::ppu::{Ppu, SCREEN_W};

/// 優先順位テーブルの要素 (上から順に走査)
#[derive(Clone, Copy)]
enum Layer {
    Bg(usize, bool), // (BG 番号 0-3, 高優先か)
    Obj(u8),         // OBJ 優先度 0-3
}

use Layer::*;

const ORDER_MODE0: &[Layer] = &[
    Obj(3), Bg(0, true), Bg(1, true), Obj(2), Bg(0, false), Bg(1, false),
    Obj(1), Bg(2, true), Bg(3, true), Obj(0), Bg(2, false), Bg(3, false),
];
const ORDER_MODE1: &[Layer] = &[
    Obj(3), Bg(0, true), Bg(1, true), Obj(2), Bg(0, false), Bg(1, false),
    Obj(1), Bg(2, true), Obj(0), Bg(2, false),
];
const ORDER_MODE1_BG3P: &[Layer] = &[
    Bg(2, true), Obj(3), Bg(0, true), Bg(1, true), Obj(2), Bg(0, false),
    Bg(1, false), Obj(1), Obj(0), Bg(2, false),
];
const ORDER_MODE23456: &[Layer] = &[
    Obj(3), Bg(0, true), Obj(2), Bg(1, true), Obj(1), Bg(0, false),
    Obj(0), Bg(1, false),
];
const ORDER_MODE7: &[Layer] = &[Obj(3), Obj(2), Obj(1), Bg(0, false), Obj(0)];

/// モード別の BG ビット深度
fn mode_bpp(mode: u8) -> &'static [u8] {
    match mode {
        0 => &[2, 2, 2, 2],
        1 => &[4, 4, 2],
        2 => &[4, 4],
        3 => &[8, 4],
        4 => &[8, 2],
        5 => &[4, 2],
        6 => &[4],
        _ => &[],
    }
}

#[derive(Clone, Copy, Default)]
struct BgPixel {
    color: u8, // CGRAM インデックス (0 = 透明)
    prio: bool,
}

#[derive(Clone, Copy, Default)]
struct ObjPixel {
    color: u8, // 0 = 透明
    prio: u8,
    math_ok: bool, // パレット 4-7 のみカラー演算対象
}

const LAYER_OBJ: usize = 4;
const LAYER_BACKDROP: usize = 5;

impl Ppu {
    pub fn render_scanline_full(&mut self, line: u16) {
        if !(1..=crate::ppu::SCREEN_H as u16).contains(&line) {
            return;
        }
        let row = (line - 1) as usize;
        if self.forced_blank {
            self.framebuffer[row * SCREEN_W..(row + 1) * SCREEN_W].fill(0xFF00_0000);
            return;
        }

        let y = line - 1; // 0 始まりの表示行

        // --- レイヤ描画 ---
        let mut bg_buf = [[BgPixel::default(); SCREEN_W]; 4];
        let mut obj_buf = [ObjPixel::default(); SCREEN_W];

        if self.bg_mode == 7 {
            self.render_mode7_line(y, &mut bg_buf[0]);
        } else {
            let bpps = mode_bpp(self.bg_mode);
            for (bg, &bpp) in bpps.iter().enumerate() {
                if (self.tm | self.ts) & (1 << bg) != 0 {
                    self.render_bg_line(bg, bpp, y, &mut bg_buf[bg]);
                }
            }
        }
        if (self.tm | self.ts) & 0x10 != 0 {
            self.render_obj_line(y, &mut obj_buf);
        }

        // --- ウィンドウマスク (true = ウィンドウ内) ---
        let masks: [[bool; SCREEN_W]; 6] = [
            self.window_mask(self.w12sel, 0, self.wbglog & 3),
            self.window_mask(self.w12sel, 4, (self.wbglog >> 2) & 3),
            self.window_mask(self.w34sel, 0, (self.wbglog >> 4) & 3),
            self.window_mask(self.w34sel, 4, (self.wbglog >> 6) & 3),
            self.window_mask(self.wobjsel, 0, self.wobjlog & 3),
            self.window_mask(self.wobjsel, 4, (self.wobjlog >> 2) & 3), // カラーウィンドウ
        ];

        let order: &[Layer] = match self.bg_mode {
            0 => ORDER_MODE0,
            1 => {
                if self.bg3_priority {
                    ORDER_MODE1_BG3P
                } else {
                    ORDER_MODE1
                }
            }
            7 => ORDER_MODE7,
            _ => ORDER_MODE23456,
        };

        let sub_enabled = self.cgwsel & 0x02 != 0;
        let brightness = self.brightness;

        for x in 0..SCREEN_W {
            // メイン/サブスクリーンの有効レイヤ (ウィンドウで打ち消し)
            let main_en = |layer: usize| -> bool {
                let bit = 1u8 << layer.min(4);
                self.tm & bit != 0 && !(self.tmw & bit != 0 && masks[layer][x])
            };
            let sub_en = |layer: usize| -> bool {
                let bit = 1u8 << layer.min(4);
                self.ts & bit != 0 && !(self.tsw & bit != 0 && masks[layer][x])
            };

            let pick = |en: &dyn Fn(usize) -> bool| -> (u16, usize) {
                for &l in order {
                    match l {
                        Bg(bg, hi) => {
                            let p = bg_buf[bg][x];
                            if p.color != 0 && p.prio == hi && en(bg) {
                                return (self.cgram[p.color as usize], bg);
                            }
                        }
                        Obj(prio) => {
                            let p = obj_buf[x];
                            if p.color != 0 && p.prio == prio && en(LAYER_OBJ) {
                                // OBJ のカラー演算可否をレイヤ番号に埋め込む
                                let l = if p.math_ok { LAYER_OBJ } else { 6 };
                                return (self.cgram[p.color as usize], l);
                            }
                        }
                    }
                }
                (self.cgram[0], LAYER_BACKDROP)
            };

            let (mut main_color, main_layer) = pick(&main_en);

            let cw_inside = masks[5][x];
            let clip_black = Self::window_region_test(self.cgwsel >> 6, cw_inside);
            let prevent_math = Self::window_region_test((self.cgwsel >> 4) & 3, cw_inside);

            if clip_black {
                main_color = 0;
            }

            // カラー演算
            let math_layer_ok = match main_layer {
                0..=3 => self.cgadsub & (1 << main_layer) != 0,
                LAYER_OBJ => self.cgadsub & 0x10 != 0,
                LAYER_BACKDROP => self.cgadsub & 0x20 != 0,
                _ => false, // OBJ パレット 0-3
            };
            if math_layer_ok && !prevent_math {
                let (addend, half_allowed) = if sub_enabled {
                    let (sc, sl) = pick(&sub_en);
                    if sl == LAYER_BACKDROP {
                        (self.fixed_color, false)
                    } else {
                        (sc, !clip_black)
                    }
                } else {
                    (self.fixed_color, !clip_black)
                };
                let half = self.cgadsub & 0x40 != 0 && half_allowed;
                let subtract = self.cgadsub & 0x80 != 0;
                main_color = Self::color_math(main_color, addend, subtract, half);
            }

            self.framebuffer[row * SCREEN_W + x] = Self::rgb555_to_argb(main_color, brightness);
        }
    }

    /// cgwsel の 2bit 領域指定: 0=なし 1=ウィンドウ外 2=ウィンドウ内 3=常時
    fn window_region_test(mode: u8, inside: bool) -> bool {
        match mode & 3 {
            0 => false,
            1 => !inside,
            2 => inside,
            _ => true,
        }
    }

    fn color_math(a: u16, b: u16, subtract: bool, half: bool) -> u16 {
        let mut out = 0u16;
        for shift in [0u16, 5, 10] {
            let ca = (a >> shift) & 0x1F;
            let cb = (b >> shift) & 0x1F;
            let mut c: i32 = if subtract {
                ca as i32 - cb as i32
            } else {
                ca as i32 + cb as i32
            };
            if half {
                c >>= 1;
            }
            out |= (c.clamp(0, 31) as u16) << shift;
        }
        out
    }

    /// ウィンドウ 1/2 の合成マスクを作る。sel の bit(base)=W1 反転, bit(base+1)=W1 有効,
    /// bit(base+2)=W2 反転, bit(base+3)=W2 有効。logic: 0=OR 1=AND 2=XOR 3=XNOR
    fn window_mask(&self, sel: u8, base: u8, logic: u8) -> [bool; SCREEN_W] {
        let w1_inv = sel & (1 << base) != 0;
        let w1_en = sel & (1 << (base + 1)) != 0;
        let w2_inv = sel & (1 << (base + 2)) != 0;
        let w2_en = sel & (1 << (base + 3)) != 0;
        let mut mask = [false; SCREEN_W];
        if !w1_en && !w2_en {
            return mask;
        }
        for (x, m) in mask.iter_mut().enumerate() {
            let x = x as u8;
            let in1 = (self.wh[0]..=self.wh[1]).contains(&x) != w1_inv;
            let in2 = (self.wh[2]..=self.wh[3]).contains(&x) != w2_inv;
            *m = match (w1_en, w2_en) {
                (true, false) => in1,
                (false, true) => in2,
                (true, true) => match logic {
                    0 => in1 | in2,
                    1 => in1 & in2,
                    2 => in1 ^ in2,
                    _ => !(in1 ^ in2),
                },
                (false, false) => unreachable!(),
            };
        }
        mask
    }

    // ---- BG タイル描画 -----------------------------------------------------

    fn render_bg_line(&self, bg: usize, bpp: u8, y: u16, buf: &mut [BgPixel; SCREEN_W]) {
        // モザイク: 対象 BG は縦方向を量子化
        let y = if self.mosaic & (1 << bg) != 0 {
            let size = (self.mosaic >> 4) as u16 + 1;
            y - y % size
        } else {
            y
        };
        let mosaic_h = if self.mosaic & (1 << bg) != 0 {
            (self.mosaic >> 4) as usize + 1
        } else {
            1
        };

        let tile16 = self.tile_size_bits & (1 << bg) != 0;
        let (tw, th): (u16, u16) = if tile16 { (16, 16) } else { (8, 8) };
        let hofs = self.bg_hofs[bg] & 0x3FF;
        let vofs = self.bg_vofs[bg] & 0x3FF;
        let map_base = ((self.bg_sc[bg] as usize & 0xFC) << 8) & 0x7FFF; // ワード単位
        let map_size = self.bg_sc[bg] & 3;
        let chr_base = ((self.bg_chr[bg / 2] as usize >> ((bg % 2) * 4)) & 0x0F) << 12; // ワード
        let bytes_per_tile = 8 * bpp as usize;
        let pal_base: u8 = if self.bg_mode == 0 { (bg as u8) * 32 } else { 0 };

        let py = y.wrapping_add(vofs);
        for x in 0..SCREEN_W {
            if mosaic_h > 1 && x % mosaic_h != 0 {
                buf[x] = buf[x - x % mosaic_h];
                continue;
            }
            let px = (x as u16).wrapping_add(hofs);
            let (tx, ty) = (px / tw, py / th);

            // タイルマップエントリ取得 (64 タイル幅/高さのクアドラント対応)
            let quad = match map_size {
                0 => 0,
                1 => ((tx >> 5) & 1) as usize * 0x400,
                2 => ((ty >> 5) & 1) as usize * 0x400,
                _ => ((tx >> 5) & 1) as usize * 0x400 + ((ty >> 5) & 1) as usize * 0x800,
            };
            let entry_addr = (map_base + quad + ((ty as usize & 31) << 5) + (tx as usize & 31))
                & 0x7FFF;
            let entry = u16::from_le_bytes([
                self.vram[entry_addr * 2],
                self.vram[entry_addr * 2 + 1],
            ]);

            let mut ch = (entry & 0x3FF) as usize;
            let pal = ((entry >> 10) & 7) as u8;
            let prio = entry & 0x2000 != 0;
            let hflip = entry & 0x4000 != 0;
            let vflip = entry & 0x8000 != 0;

            let mut fx = (px % tw) as usize;
            let mut fy = (py % th) as usize;
            if hflip {
                fx = tw as usize - 1 - fx;
            }
            if vflip {
                fy = th as usize - 1 - fy;
            }
            if tile16 {
                if fx >= 8 {
                    ch += 1;
                    fx -= 8;
                }
                if fy >= 8 {
                    ch += 16;
                    fy -= 8;
                }
            }

            let addr = (chr_base * 2 + (ch & 0x3FF) * bytes_per_tile + fy * 2) & 0xFFFF;
            let idx = self.tile_pixel(addr, bpp, fx);

            buf[x] = if idx == 0 {
                BgPixel::default()
            } else {
                let color = match bpp {
                    2 => pal_base + pal * 4 + idx,
                    4 => pal * 16 + idx,
                    _ => idx,
                };
                BgPixel { color, prio }
            };
        }
    }

    /// タイル 1 ピクセル取り出し。addr = プレーン 0/1 行先頭 (バイト)、fx = タイル内 x (0-7)
    fn tile_pixel(&self, addr: usize, bpp: u8, fx: usize) -> u8 {
        let bit = 7 - fx;
        let mut idx = ((self.vram[addr & 0xFFFF] >> bit) & 1)
            | (((self.vram[(addr + 1) & 0xFFFF] >> bit) & 1) << 1);
        if bpp >= 4 {
            idx |= (((self.vram[(addr + 16) & 0xFFFF] >> bit) & 1) << 2)
                | (((self.vram[(addr + 17) & 0xFFFF] >> bit) & 1) << 3);
        }
        if bpp == 8 {
            idx |= (((self.vram[(addr + 32) & 0xFFFF] >> bit) & 1) << 4)
                | (((self.vram[(addr + 33) & 0xFFFF] >> bit) & 1) << 5)
                | (((self.vram[(addr + 48) & 0xFFFF] >> bit) & 1) << 6)
                | (((self.vram[(addr + 49) & 0xFFFF] >> bit) & 1) << 7);
        }
        idx
    }

    // ---- Mode 7 -------------------------------------------------------------

    fn render_mode7_line(&self, y: u16, buf: &mut [BgPixel; SCREEN_W]) {
        let clip13 = |v: i32| -> i32 {
            if v & 0x2000 != 0 {
                v | !0x3FF
            } else {
                v & 0x3FF
            }
        };
        let a = self.m7[0] as i32;
        let b = self.m7[1] as i32;
        let c = self.m7[2] as i32;
        let d = self.m7[3] as i32;
        let cx = self.m7x as i32;
        let cy = self.m7y as i32;
        let h = clip13((self.m7_hofs as i32 - cx) & 0x3FFF);
        let v = clip13((self.m7_vofs as i32 - cy) & 0x3FFF);

        let yy = if self.m7sel & 2 != 0 {
            255 - y as i32
        } else {
            y as i32
        };

        let px = ((a * h) & !63) + ((b * yy) & !63) + ((b * v) & !63) + (cx << 8);
        let py = ((c * h) & !63) + ((d * yy) & !63) + ((d * v) & !63) + (cy << 8);

        let over = self.m7sel >> 6; // 0/1: ラップ, 2: 透明, 3: タイル 0 埋め

        for x in 0..SCREEN_W {
            let xx = if self.m7sel & 1 != 0 {
                255 - x as i32
            } else {
                x as i32
            };
            let tx = (px + a * xx) >> 8;
            let ty = (py + c * xx) >> 8;

            let out_of_map = !(0..1024).contains(&tx) || !(0..1024).contains(&ty);
            let (mtx, mty, force_tile0) = match (out_of_map, over) {
                (false, _) | (true, 0) | (true, 1) => (tx & 0x3FF, ty & 0x3FF, false),
                (true, 2) => {
                    buf[x] = BgPixel::default();
                    continue;
                }
                _ => (tx & 0x3FF, ty & 0x3FF, true),
            };

            let tile = if force_tile0 {
                0
            } else {
                let map_addr = ((mty as usize >> 3) * 128 + (mtx as usize >> 3)) & 0x7FFF;
                self.vram[map_addr * 2] as usize
            };
            let chr_addr = tile * 128 + ((mty as usize & 7) * 8 + (mtx as usize & 7)) * 2 + 1;
            let idx = self.vram[chr_addr & 0xFFFF];
            buf[x] = if idx == 0 {
                BgPixel::default()
            } else {
                BgPixel {
                    color: idx,
                    prio: false,
                }
            };
        }
    }

    // ---- OBJ (スプライト) -----------------------------------------------------

    fn render_obj_line(&self, y: u16, buf: &mut [ObjPixel; SCREEN_W]) {
        let (small, large) = Self::obj_sizes(self.obsel >> 5);
        let base = ((self.obsel as usize & 7) << 14) & 0xFFFF;
        let gap = (((self.obsel as usize >> 3) & 3) + 1) << 13;

        // 番号の小さいスプライトほど手前。後勝ちで上書きするため降順に描く。
        for i in (0..128).rev() {
            let sx_low = self.oam[i * 4];
            let sy = self.oam[i * 4 + 1];
            let tile = self.oam[i * 4 + 2] as usize;
            let attr = self.oam[i * 4 + 3];
            let hi = self.oam[0x200 + i / 4] >> ((i % 4) * 2);
            let x_high = hi & 1 != 0;
            let big = hi & 2 != 0;
            let (w, h) = if big { large } else { small };

            let dy = (y as u8).wrapping_sub(sy) as usize;
            if dy >= h {
                continue;
            }

            let sx = if x_high {
                sx_low as i32 - 256
            } else {
                sx_low as i32
            };
            if sx <= -(w as i32) {
                continue;
            }

            let vflip = attr & 0x80 != 0;
            let hflip = attr & 0x40 != 0;
            let prio = (attr >> 4) & 3;
            let pal = (attr >> 1) & 7;
            let nametable = attr & 1 != 0;

            let row = if vflip { h - 1 - dy } else { dy };
            let tile_base = base + if nametable { gap } else { 0 };

            for cx in 0..w {
                let px = sx + cx as i32;
                if !(0..SCREEN_W as i32).contains(&px) {
                    continue;
                }
                let col = if hflip { w - 1 - cx } else { cx };
                // 16x16 グリッド内でタイル番号をニブル単位にラップ
                let tx = ((tile & 0x0F) + col / 8) & 0x0F;
                let ty = ((tile >> 4) + row / 8) & 0x0F;
                let t = (ty << 4) | tx;
                let addr = (tile_base + t * 32 + (row & 7) * 2) & 0xFFFF;
                let idx = self.tile_pixel(addr, 4, col & 7);
                if idx != 0 {
                    buf[px as usize] = ObjPixel {
                        color: 128 + pal * 16 + idx,
                        prio,
                        math_ok: pal >= 4,
                    };
                }
            }
        }
    }

    fn obj_sizes(mode: u8) -> ((usize, usize), (usize, usize)) {
        match mode {
            0 => ((8, 8), (16, 16)),
            1 => ((8, 8), (32, 32)),
            2 => ((8, 8), (64, 64)),
            3 => ((16, 16), (32, 32)),
            4 => ((16, 16), (64, 64)),
            5 => ((32, 32), (64, 64)),
            6 => ((16, 32), (32, 64)),
            _ => ((16, 32), (32, 32)),
        }
    }
}
