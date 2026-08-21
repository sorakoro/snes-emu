//! S-DSP: 8 ボイスの BRR デコード、ガウシアン補間、ADSR/GAIN エンベロープ、
//! ピッチ変調、ノイズ、エコー (FIR)。32kHz でステレオサンプルを生成する。
//!
//! 演算は実機準拠 (ares/bsnes の観測結果に基づく): サンプルは 2 倍スケールで
//! 12 サンプルリングに格納し、各加算段で 16bit クランプする。

/// エンベロープなどのレート → 周期 (サンプル数)。0 は「変化なし」。
const RATE_PERIODS: [u32; 32] = [
    0, 2048, 1536, 1280, 1024, 768, 640, 512, 384, 320, 256, 192, 160, 128, 96, 80, 64, 48, 40,
    32, 24, 20, 16, 12, 10, 8, 6, 5, 4, 3, 2, 1,
];

#[inline]
fn clamp16(v: i32) -> i32 {
    v.clamp(-0x8000, 0x7FFF)
}

#[derive(Clone, Copy, PartialEq)]
enum EnvMode {
    Attack,
    Decay,
    Sustain,
    Release,
}

#[derive(Clone, Copy)]
struct Voice {
    playing: bool,
    brr_addr: u16,
    brr_offset: u8, // ブロック内バイト位置 (1-8)
    brr_header: u8,
    /// デコード済みサンプル (2 倍スケール、12 エントリリング)
    buf: [i32; 12],
    buf_offset: usize, // 次のデコード書き込み位置 = 最古サンプル位置
    gauss_offset: u32, // bit4-11: ガウス位相, bit12-14: サンプルオフセット
    env: i32,          // 0-0x7FF
    env_mode: EnvMode,
    env_timer: u32,
}

impl Default for Voice {
    fn default() -> Self {
        Voice {
            playing: false,
            brr_addr: 0,
            brr_offset: 1,
            brr_header: 0,
            buf: [0; 12],
            buf_offset: 0,
            gauss_offset: 0,
            env: 0,
            env_mode: EnvMode::Release,
            env_timer: 0,
        }
    }
}

pub struct Dsp {
    pub regs: [u8; 128],
    gauss: [i32; 512],
    voices: [Voice; 8],
    noise: i32,
    noise_timer: u32,
    echo_offset: u16,
    echo_length: u16,
    fir_l: [i32; 8],
    fir_r: [i32; 8],
    fir_pos: usize,
    kon_latch: u8,
    kof_latch: u8,
}

impl Default for Dsp {
    fn default() -> Self {
        Self::new()
    }
}

/// ガウシアン補間テーブルを実機同等の式から生成する
fn build_gauss_table() -> [i32; 512] {
    use std::f64::consts::PI;
    let mut raw = [0f64; 512];
    for n in 0..512usize {
        let k = 0.5 + n as f64;
        let s = (PI * k * 1.280 / 1024.0).sin();
        let t = ((PI * k * 2.000 / 1023.0).cos() - 1.0) * 0.50;
        let u = ((PI * k * 4.000 / 1023.0).cos() - 1.0) * 0.08;
        raw[511 - n] = s * (t + u + 1.0) / k;
    }
    let mut g = [0i32; 512];
    for phase in 0..128usize {
        let sum = raw[phase] + raw[phase + 256] + raw[511 - phase] + raw[255 - phase];
        let scale = 2048.0 / sum;
        g[phase] = (raw[phase] * scale + 0.5) as i32;
        g[phase + 256] = (raw[phase + 256] * scale + 0.5) as i32;
        g[511 - phase] = (raw[511 - phase] * scale + 0.5) as i32;
        g[255 - phase] = (raw[255 - phase] * scale + 0.5) as i32;
    }
    g
}

impl Dsp {
    pub fn new() -> Self {
        let mut regs = [0u8; 128];
        regs[0x6C] = 0xE0; // FLG: リセット + ミュート + エコー無効
        Dsp {
            regs,
            gauss: build_gauss_table(),
            voices: [Voice::default(); 8],
            noise: 0x4000,
            noise_timer: 0,
            echo_offset: 0,
            echo_length: 4,
            fir_l: [0; 8],
            fir_r: [0; 8],
            fir_pos: 0,
            kon_latch: 0,
            kof_latch: 0,
        }
    }

    pub fn read(&self, addr: u8) -> u8 {
        self.regs[(addr & 0x7F) as usize]
    }

    pub fn write(&mut self, addr: u8, v: u8) {
        let addr = (addr & 0x7F) as usize;
        match addr {
            0x4C => self.kon_latch |= v, // KON
            0x5C => self.kof_latch |= v, // KOF
            0x7C => {
                // ENDX への書き込みはクリア
                self.regs[0x7C] = 0;
                return;
            }
            _ => {}
        }
        self.regs[addr] = v;
    }

    #[inline]
    fn vreg(&self, voice: usize, off: usize) -> u8 {
        self.regs[(voice << 4) | off]
    }

    /// 1 サンプル (32kHz) 生成
    pub fn sample(&mut self, ram: &mut [u8; 0x10000]) -> (i16, i16) {
        let flg = self.regs[0x6C];

        // キーオン / キーオフ
        let kon = std::mem::take(&mut self.kon_latch);
        let kof = std::mem::take(&mut self.kof_latch);
        for v in 0..8 {
            let bit = 1 << v;
            if kon & bit != 0 {
                self.key_on(v, ram);
            }
            if (kof & bit != 0 || flg & 0x80 != 0) && self.voices[v].playing {
                self.voices[v].env_mode = EnvMode::Release;
            }
        }

        // ノイズ生成 (15bit LFSR)
        let noise_rate = (flg & 0x1F) as usize;
        if RATE_PERIODS[noise_rate] != 0 {
            self.noise_timer += 1;
            if self.noise_timer >= RATE_PERIODS[noise_rate] {
                self.noise_timer = 0;
                let fb = (self.noise ^ (self.noise >> 1)) & 1;
                self.noise = ((self.noise >> 1) & 0x3FFF) | (fb << 14);
            }
        }

        let pmon = self.regs[0x2D];
        let non = self.regs[0x3D];
        let eon = self.regs[0x4D];

        let mut main_l = 0i32;
        let mut main_r = 0i32;
        let mut echo_l = 0i32;
        let mut echo_r = 0i32;
        let mut prev_out = 0i32; // ピッチ変調用の前ボイス出力

        for v in 0..8 {
            if !self.voices[v].playing {
                prev_out = 0;
                continue;
            }

            // ピッチ (+ 変調)
            let mut pitch =
                (((self.vreg(v, 3) as u32 & 0x3F) << 8) | self.vreg(v, 2) as u32) as i32;
            if v > 0 && pmon & (1 << v) != 0 {
                pitch += (prev_out >> 5) * pitch >> 10;
                pitch = pitch.clamp(0, 0x3FFF);
            }

            // 必要ならデコード (gauss_offset が 1 グループ分進んだら 4 サンプル補充)
            while self.voices[v].gauss_offset >= 0x4000 && self.voices[v].playing {
                self.voices[v].gauss_offset -= 0x4000;
                self.decode_group(v, ram);
            }
            if !self.voices[v].playing {
                prev_out = 0;
                continue;
            }

            // 波形取得
            let raw = if non & (1 << v) != 0 {
                (((self.noise << 1) & 0xFFFF) as u16) as i16 as i32
            } else {
                self.gaussian(v)
            };

            // エンベロープ更新
            self.update_envelope(v);
            let vc = &mut self.voices[v];
            if !vc.playing {
                prev_out = 0;
                continue;
            }
            let output = (raw * vc.env >> 11) & !1;
            prev_out = output;

            // ENVX / OUTX
            self.regs[(v << 4) | 8] = ((self.voices[v].env >> 4) & 0x7F) as u8;
            self.regs[(v << 4) | 9] = (output >> 8) as u8;

            let vol_l = self.vreg(v, 0) as i8 as i32;
            let vol_r = self.vreg(v, 1) as i8 as i32;
            main_l = clamp16(main_l + (output * vol_l >> 7));
            main_r = clamp16(main_r + (output * vol_r >> 7));
            if eon & (1 << v) != 0 {
                echo_l = clamp16(echo_l + (output * vol_l >> 7));
                echo_r = clamp16(echo_r + (output * vol_r >> 7));
            }

            // 再生位置を進める (15bit 上限)
            let vc = &mut self.voices[v];
            vc.gauss_offset = (vc.gauss_offset + pitch as u32).min(0x7FFF);
        }

        // ---- エコー ----
        let esa = self.regs[0x6D] as u16;
        let echo_base = (esa << 8).wrapping_add(self.echo_offset);

        // バッファ読み → 履歴 (>>1 で 15bit)
        let rd = |ram: &[u8; 0x10000], a: u16| -> i32 {
            i16::from_le_bytes([ram[a as usize], ram[a.wrapping_add(1) as usize]]) as i32 >> 1
        };
        self.fir_pos = (self.fir_pos + 1) & 7;
        self.fir_l[self.fir_pos] = rd(ram, echo_base);
        self.fir_r[self.fir_pos] = rd(ram, echo_base.wrapping_add(2));

        // FIR: タップ 0-6 を 16bit ラップで積算し、タップ 7 を加えてクランプ
        let mut fir_out_l = 0i32;
        let mut fir_out_r = 0i32;
        for t in 0..7 {
            let coef = self.regs[0x0F | (t << 4)] as i8 as i32;
            let s = (self.fir_pos + t + 1) & 7;
            fir_out_l += self.fir_l[s] * coef >> 6;
            fir_out_r += self.fir_r[s] * coef >> 6;
        }
        fir_out_l = (fir_out_l as i16) as i32;
        fir_out_r = (fir_out_r as i16) as i32;
        let coef7 = self.regs[0x7F] as i8 as i32;
        fir_out_l = clamp16(fir_out_l + ((self.fir_l[self.fir_pos] * coef7 >> 6) as i16 as i32)) & !1;
        fir_out_r = clamp16(fir_out_r + ((self.fir_r[self.fir_pos] * coef7 >> 6) as i16 as i32)) & !1;

        // エコーバッファ書き込み (FLG bit5 = 書き込み禁止)
        if flg & 0x20 == 0 {
            let efb = self.regs[0x0D] as i8 as i32;
            let wl = (clamp16(echo_l + ((fir_out_l * efb >> 7) as i16 as i32)) & !1) as i16;
            let wr = (clamp16(echo_r + ((fir_out_r * efb >> 7) as i16 as i32)) & !1) as i16;
            let a = echo_base;
            ram[a as usize] = wl as u8;
            ram[a.wrapping_add(1) as usize] = (wl >> 8) as u8;
            ram[a.wrapping_add(2) as usize] = wr as u8;
            ram[a.wrapping_add(3) as usize] = (wr >> 8) as u8;
        }

        // オフセット更新 (先頭に戻るとき EDL を再ロード)
        if self.echo_offset == 0 {
            let edl = self.regs[0x7D] & 0x0F;
            self.echo_length = if edl == 0 { 4 } else { edl as u16 * 2048 };
        }
        self.echo_offset += 4;
        if self.echo_offset >= self.echo_length {
            self.echo_offset = 0;
        }

        // ---- 最終ミックス ----
        if flg & 0x40 != 0 {
            return (0, 0); // ミュート
        }
        let mvol_l = self.regs[0x0C] as i8 as i32;
        let mvol_r = self.regs[0x1C] as i8 as i32;
        let evol_l = self.regs[0x2C] as i8 as i32;
        let evol_r = self.regs[0x3C] as i8 as i32;
        let out_l = clamp16(((main_l * mvol_l >> 7) as i16 as i32) + ((fir_out_l * evol_l >> 7) as i16 as i32));
        let out_r = clamp16(((main_r * mvol_r >> 7) as i16 as i32) + ((fir_out_r * evol_r >> 7) as i16 as i32));
        (out_l as i16, out_r as i16)
    }

    /// ガウシアン補間 (4 タップ)
    fn gaussian(&self, v: usize) -> i32 {
        let vc = &self.voices[v];
        let go = vc.gauss_offset & 0x3FFF;
        let o = ((go >> 4) & 0xFF) as usize;
        let g = &self.gauss;
        let mut idx = (vc.buf_offset + (go >> 12) as usize) % 12;
        let mut out = g[255 - o] * vc.buf[idx] >> 11;
        idx = (idx + 1) % 12;
        out += g[511 - o] * vc.buf[idx] >> 11;
        idx = (idx + 1) % 12;
        out += g[256 + o] * vc.buf[idx] >> 11;
        idx = (idx + 1) % 12;
        out = (out as i16) as i32; // 16bit ラップ (実機挙動)
        out += g[o] * vc.buf[idx] >> 11;
        clamp16(out) & !1
    }

    fn dir_entry(&self, ram: &[u8; 0x10000], srcn: u8) -> (u16, u16) {
        let dir = self.regs[0x5D] as usize * 0x100 + srcn as usize * 4;
        let start = u16::from_le_bytes([ram[dir & 0xFFFF], ram[(dir + 1) & 0xFFFF]]);
        let lp = u16::from_le_bytes([ram[(dir + 2) & 0xFFFF], ram[(dir + 3) & 0xFFFF]]);
        (start, lp)
    }

    fn key_on(&mut self, v: usize, ram: &[u8; 0x10000]) {
        let srcn = self.vreg(v, 4);
        let (start, _) = self.dir_entry(ram, srcn);
        {
            let vc = &mut self.voices[v];
            vc.playing = true;
            vc.brr_addr = start;
            vc.brr_offset = 1;
            vc.brr_header = ram[start as usize];
            vc.buf = [0; 12];
            vc.buf_offset = 0;
            vc.gauss_offset = 0;
            vc.env = 0;
            vc.env_mode = EnvMode::Attack;
            vc.env_timer = 0;
        }
        self.regs[0x7C] &= !(1 << v); // ENDX クリア
        // 3 グループ (12 サンプル) 先行デコードして補間タップを埋める
        for _ in 0..3 {
            self.decode_group(v, ram);
        }
    }

    /// BRR 1 グループ (2 バイト = 4 サンプル) をデコードしてリングへ
    fn decode_group(&mut self, v: usize, ram: &[u8; 0x10000]) {
        let vc = &mut self.voices[v];
        if !vc.playing {
            return;
        }
        let header = vc.brr_header;
        let scale = (header >> 4) as i32;
        let filter = (header >> 2) & 3;

        let b0 = ram[vc.brr_addr.wrapping_add(vc.brr_offset as u16) as usize] as i32;
        let b1 = ram[vc.brr_addr.wrapping_add(vc.brr_offset as u16 + 1) as usize] as i32;
        let mut nybbles: i32 = (b0 << 8) | b1;

        for _ in 0..4 {
            // 上位ニブルを符号拡張して取り出す
            let mut s = ((nybbles as i16) >> 12) as i32;
            nybbles <<= 4;

            if scale <= 12 {
                s = (s << scale) >> 1;
            } else {
                s &= !0x7FF;
            }

            // IIR フィルタ (p1 は 2 倍スケール、p2 は 1 倍スケールで参照)
            let p1 = vc.buf[(vc.buf_offset + 11) % 12];
            let p2 = vc.buf[(vc.buf_offset + 10) % 12] >> 1;
            match filter {
                0 => {}
                1 => {
                    s += p1 >> 1;
                    s += (-p1) >> 5;
                }
                2 => {
                    s += p1;
                    s -= p2;
                    s += p2 >> 4;
                    s += (p1 * -3) >> 6;
                }
                _ => {
                    s += p1;
                    s -= p2;
                    s += (p1 * -13) >> 7;
                    s += (p2 * 3) >> 4;
                }
            }

            s = clamp16(s);
            s = (((s << 1) & 0xFFFF) as u16) as i16 as i32; // 2 倍スケールで格納 (ラップ)
            vc.buf[vc.buf_offset] = s;
            vc.buf_offset = (vc.buf_offset + 1) % 12;
        }
        vc.brr_offset += 2;

        if vc.brr_offset > 8 {
            // ブロック終端
            if header & 1 != 0 {
                self.regs[0x7C] |= 1 << v; // ENDX
                if header & 2 != 0 {
                    let srcn = self.vreg(v, 4);
                    let (_, lp) = self.dir_entry(ram, srcn);
                    let vc = &mut self.voices[v];
                    vc.brr_addr = lp;
                } else {
                    let vc = &mut self.voices[v];
                    vc.playing = false;
                    vc.env = 0;
                    return;
                }
            } else {
                let vc = &mut self.voices[v];
                vc.brr_addr = vc.brr_addr.wrapping_add(9);
            }
            let vc = &mut self.voices[v];
            vc.brr_offset = 1;
            vc.brr_header = ram[vc.brr_addr as usize];
        }
    }

    fn update_envelope(&mut self, v: usize) {
        let adsr1 = self.vreg(v, 5);
        let adsr2 = self.vreg(v, 6);
        let gain = self.vreg(v, 7);
        let vc = &mut self.voices[v];

        if vc.env_mode == EnvMode::Release {
            vc.env -= 8;
            if vc.env <= 0 {
                vc.env = 0;
                vc.playing = false;
            }
            return;
        }

        if adsr1 & 0x80 != 0 {
            // ADSR モード
            match vc.env_mode {
                EnvMode::Attack => {
                    let rate = ((adsr1 & 0x0F) << 1 | 1) as usize;
                    if rate == 31 {
                        vc.env += 1024;
                    } else if Self::rate_tick(vc, rate) {
                        vc.env += 32;
                    }
                    if vc.env >= 0x7E0 {
                        vc.env = vc.env.min(0x7FF);
                        vc.env_mode = EnvMode::Decay;
                        vc.env_timer = 0;
                    }
                }
                EnvMode::Decay => {
                    let rate = (((adsr1 >> 4) & 7) << 1 | 0x10) as usize;
                    if Self::rate_tick(vc, rate) {
                        vc.env -= ((vc.env - 1) >> 8) + 1;
                    }
                    let sustain = ((adsr2 as i32 >> 5) + 1) << 8;
                    if vc.env <= sustain {
                        vc.env_mode = EnvMode::Sustain;
                        vc.env_timer = 0;
                    }
                }
                EnvMode::Sustain => {
                    let rate = (adsr2 & 0x1F) as usize;
                    if rate != 0 && Self::rate_tick(vc, rate) {
                        vc.env -= ((vc.env - 1) >> 8) + 1;
                    }
                }
                EnvMode::Release => unreachable!(),
            }
        } else {
            // GAIN モード
            if gain & 0x80 == 0 {
                vc.env = (gain as i32 & 0x7F) << 4;
            } else {
                let rate = (gain & 0x1F) as usize;
                if rate != 0 && Self::rate_tick(vc, rate) {
                    match (gain >> 5) & 3 {
                        0 => vc.env -= 32,                      // 線形減少
                        1 => vc.env -= ((vc.env - 1) >> 8) + 1, // 指数減少
                        2 => vc.env += 32,                      // 線形増加
                        _ => {
                            // ベントライン増加
                            vc.env += if vc.env < 0x600 { 32 } else { 8 };
                        }
                    }
                }
            }
        }
        vc.env = vc.env.clamp(0, 0x7FF);
    }

    /// レートに応じた周期タイマ。true なら 1 ステップ実行
    fn rate_tick(vc: &mut Voice, rate: usize) -> bool {
        let period = RATE_PERIODS[rate & 31];
        if period == 0 {
            return false;
        }
        vc.env_timer += 1;
        if vc.env_timer >= period {
            vc.env_timer = 0;
            true
        } else {
            false
        }
    }
}
