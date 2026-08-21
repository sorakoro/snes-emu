//! カートリッジ (ROM イメージ) の読み込みと LoROM/HiROM マッピング。

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapMode {
    LoRom,
    HiRom,
}

pub struct Cartridge {
    pub rom: Vec<u8>,
    pub sram: Vec<u8>,
    pub map: MapMode,
    pub title: String,
}

impl Cartridge {
    pub fn new(mut data: Vec<u8>) -> Result<Cartridge, String> {
        if data.len() < 0x8000 {
            return Err("ROM が小さすぎます".into());
        }
        // 512 バイトのコピーツールヘッダを除去
        if data.len() % 0x400 == 0x200 {
            data.drain(..0x200);
        }

        let lo_score = Self::score_header(&data, 0x7FC0);
        let hi_score = Self::score_header(&data, 0xFFC0);
        let map = if hi_score > lo_score {
            MapMode::HiRom
        } else {
            MapMode::LoRom
        };
        let header = match map {
            MapMode::LoRom => 0x7FC0,
            MapMode::HiRom => 0xFFC0,
        };

        let title = data[header..header + 21]
            .iter()
            .map(|&b| if (0x20..0x7F).contains(&b) { b as char } else { ' ' })
            .collect::<String>()
            .trim()
            .to_string();

        let sram_size_code = data.get(header + 0x18).copied().unwrap_or(0);
        let sram_len = if sram_size_code == 0 || sram_size_code > 0x0C {
            0
        } else {
            0x400usize << sram_size_code
        };

        Ok(Cartridge {
            rom: data,
            sram: vec![0; sram_len],
            map,
            title,
        })
    }

    /// ヘッダ位置の妥当性をスコアリングして LoROM/HiROM を判定する。
    fn score_header(data: &[u8], base: usize) -> i32 {
        if data.len() < base + 0x40 {
            return i32::MIN;
        }
        let mut score = 0;
        let mode = data[base + 0x15];
        let checksum = u16::from_le_bytes([data[base + 0x1C], data[base + 0x1D]]);
        let complement = u16::from_le_bytes([data[base + 0x1E], data[base + 0x1F]]);
        if checksum ^ complement == 0xFFFF {
            score += 8;
        }
        // リセットベクタは $8000 以上を指すはず
        let reset = u16::from_le_bytes([data[base + 0x3C], data[base + 0x3D]]);
        if reset >= 0x8000 {
            score += 4;
        } else {
            score -= 8;
        }
        // マップモードバイトとヘッダ位置の一致
        match mode & 0x0F {
            0x00 | 0x02 | 0x03 if base == 0x7FC0 => score += 4,
            0x01 | 0x05 | 0x0A if base == 0xFFC0 => score += 4,
            _ => {}
        }
        // タイトルが概ね ASCII か
        if data[base..base + 21]
            .iter()
            .all(|&b| b == 0 || (0x20..0x7F).contains(&b))
        {
            score += 2;
        }
        score
    }

    /// CPU アドレス (bank:offset) → ROM/SRAM 読み出し。マップ外は None (オープンバス)。
    pub fn read(&self, bank: u8, off: u16) -> Option<u8> {
        match self.map {
            MapMode::LoRom => self.read_lorom(bank, off),
            MapMode::HiRom => self.read_hirom(bank, off),
        }
    }

    pub fn write(&mut self, bank: u8, off: u16, v: u8) {
        match self.map {
            MapMode::LoRom => {
                let b = bank & 0x7F;
                if (0x70..=0x7D).contains(&b) && off < 0x8000 && !self.sram.is_empty() {
                    let idx = (((b - 0x70) as usize) << 15 | off as usize) % self.sram.len();
                    self.sram[idx] = v;
                }
            }
            MapMode::HiRom => {
                let b = bank & 0x7F;
                if (0x20..=0x3F).contains(&b) && (0x6000..0x8000).contains(&off) && !self.sram.is_empty() {
                    let idx = (((b - 0x20) as usize) << 13 | (off - 0x6000) as usize) % self.sram.len();
                    self.sram[idx] = v;
                }
            }
        }
    }

    fn read_lorom(&self, bank: u8, off: u16) -> Option<u8> {
        let b = (bank & 0x7F) as usize;
        if (0x70..=0x7D).contains(&(b as u8)) && off < 0x8000 && !self.sram.is_empty() {
            let idx = ((b - 0x70) << 15 | off as usize) % self.sram.len();
            return Some(self.sram[idx]);
        }
        // 上位半分 (および bank 0x40 以降は下位半分もミラー) が ROM
        if off >= 0x8000 || b >= 0x40 {
            let idx = (b << 15) | (off & 0x7FFF) as usize;
            return Some(self.rom[idx % self.rom.len()]);
        }
        None
    }

    fn read_hirom(&self, bank: u8, off: u16) -> Option<u8> {
        let b = (bank & 0x7F) as usize;
        if (0x20..=0x3F).contains(&(b as u8)) && (0x6000..0x8000).contains(&off) && !self.sram.is_empty() {
            let idx = ((b - 0x20) << 13 | (off - 0x6000) as usize) % self.sram.len();
            return Some(self.sram[idx]);
        }
        if b >= 0x40 || off >= 0x8000 {
            let idx = ((b & 0x3F) << 16) | off as usize;
            return Some(self.rom[idx % self.rom.len()]);
        }
        None
    }
}
