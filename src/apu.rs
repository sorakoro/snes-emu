//! APU: SPC700 + 64KB ARAM + タイマ + CPU 通信ポート + S-DSP。
//!
//! メイン CPU のマスターサイクルに対して `run_to` で追いつく方式。
//! SPC700 は 1.024MHz、マスタークロックは 21.477MHz。

use crate::dsp::Dsp;
use crate::spc700::{Spc700, SpcBus};

/// IPL ブート ROM (64 バイト)
const IPL_ROM: [u8; 64] = [
    0xCD, 0xEF, 0xBD, 0xE8, 0x00, 0xC6, 0x1D, 0xD0, 0xFC, 0x8F, 0xAA, 0xF4, 0x8F, 0xBB, 0xF5,
    0x78, 0xCC, 0xF4, 0xD0, 0xFB, 0x2F, 0x19, 0xEB, 0xF4, 0xD0, 0xFC, 0x7E, 0xF4, 0xD0, 0x0B,
    0xE4, 0xF5, 0xCB, 0xF4, 0xD7, 0x00, 0xFC, 0xD0, 0xF3, 0xAB, 0x01, 0x10, 0xEF, 0x7E, 0xF4,
    0x10, 0xEB, 0xBA, 0xF6, 0xDA, 0x00, 0xBA, 0xF4, 0xC4, 0xF4, 0xDD, 0x5D, 0xD0, 0xDB, 0x1F,
    0x00, 0x00, 0xC0, 0xFF,
];

#[derive(Default, Clone, Copy)]
struct Timer {
    enabled: bool,
    target: u8,
    divider: u8,
    counter: u8, // 4bit
    prescaler: u32,
}

impl Timer {
    fn tick(&mut self, cycles: u32, period: u32) {
        self.prescaler += cycles;
        while self.prescaler >= period {
            self.prescaler -= period;
            if self.enabled {
                self.divider = self.divider.wrapping_add(1);
                if self.divider == self.target {
                    self.divider = 0;
                    self.counter = (self.counter + 1) & 0x0F;
                }
            }
        }
    }
}

pub struct ApuInner {
    pub ram: Box<[u8; 0x10000]>,
    pub dsp: Dsp,
    dsp_addr: u8,
    /// メイン CPU → SPC ($2140-43 write / $F4-F7 read)
    pub in_ports: [u8; 4],
    /// SPC → メイン CPU ($F4-F7 write / $2140-43 read)
    pub out_ports: [u8; 4],
    aux: [u8; 2],
    timers: [Timer; 3],
    ipl_enabled: bool,
    dsp_cycle_acc: u32,
    /// ステレオ i16 インターリーブのサンプルバッファ (32kHz)
    pub samples: Vec<i16>,
}

impl SpcBus for ApuInner {
    fn read(&mut self, addr: u16) -> u8 {
        match addr {
            0x00F0..=0x00FF => match addr {
                0x00F2 => self.dsp_addr,
                0x00F3 => self.dsp.read(self.dsp_addr & 0x7F),
                0x00F4..=0x00F7 => self.in_ports[(addr - 0x00F4) as usize],
                0x00F8 | 0x00F9 => self.aux[(addr - 0x00F8) as usize],
                0x00FD..=0x00FF => {
                    let t = &mut self.timers[(addr - 0x00FD) as usize];
                    let v = t.counter;
                    t.counter = 0;
                    v
                }
                _ => 0,
            },
            0xFFC0..=0xFFFF if self.ipl_enabled => IPL_ROM[(addr - 0xFFC0) as usize],
            _ => self.ram[addr as usize],
        }
    }

    fn write(&mut self, addr: u16, v: u8) {
        // I/O 領域以外は常に RAM (IPL 領域下の RAM にも書ける)
        if !(0x00F0..=0x00FF).contains(&addr) {
            self.ram[addr as usize] = v;
            return;
        }
        match addr {
            0x00F1 => {
                for i in 0..3 {
                    let en = v & (1 << i) != 0;
                    if en && !self.timers[i].enabled {
                        self.timers[i].divider = 0;
                        self.timers[i].counter = 0;
                    }
                    self.timers[i].enabled = en;
                }
                if v & 0x10 != 0 {
                    self.in_ports[0] = 0;
                    self.in_ports[1] = 0;
                }
                if v & 0x20 != 0 {
                    self.in_ports[2] = 0;
                    self.in_ports[3] = 0;
                }
                self.ipl_enabled = v & 0x80 != 0;
            }
            0x00F2 => self.dsp_addr = v,
            0x00F3 => {
                if self.dsp_addr < 0x80 {
                    self.dsp.write(self.dsp_addr, v);
                }
            }
            0x00F4..=0x00F7 => self.out_ports[(addr - 0x00F4) as usize] = v,
            0x00F8 | 0x00F9 => self.aux[(addr - 0x00F8) as usize] = v,
            0x00FA..=0x00FC => self.timers[(addr - 0x00FA) as usize].target = v,
            _ => {}
        }
    }
}

impl ApuInner {
    /// SPC サイクル経過に伴うタイマ / DSP の駆動
    fn tick(&mut self, cycles: u32) {
        self.timers[0].tick(cycles, 128); // 8kHz
        self.timers[1].tick(cycles, 128);
        self.timers[2].tick(cycles, 16); // 64kHz
        self.dsp_cycle_acc += cycles;
        while self.dsp_cycle_acc >= 32 {
            self.dsp_cycle_acc -= 32;
            let (l, r) = self.dsp.sample(&mut self.ram);
            self.samples.push(l);
            self.samples.push(r);
        }
    }
}

pub struct Apu {
    pub cpu: Spc700,
    pub inner: ApuInner,
    spc_cycles: u64,
}

impl Default for Apu {
    fn default() -> Self {
        Self::new()
    }
}

impl Apu {
    pub fn new() -> Self {
        let mut inner = ApuInner {
            ram: Box::new([0; 0x10000]),
            dsp: Dsp::new(),
            dsp_addr: 0,
            in_ports: [0; 4],
            out_ports: [0; 4],
            aux: [0; 2],
            timers: [Timer::default(); 3],
            ipl_enabled: true,
            dsp_cycle_acc: 0,
            samples: Vec::new(),
        };
        let mut cpu = Spc700::new();
        cpu.reset(&mut inner);
        Apu {
            cpu,
            inner,
            spc_cycles: 0,
        }
    }

    /// マスターサイクル時刻 `master` まで SPC700 を実行する
    pub fn run_to(&mut self, master: u64) {
        let target = (master as u128 * 1_024_000 / 21_477_272) as u64;
        while self.spc_cycles < target {
            let c = if self.cpu.stopped {
                2
            } else {
                self.cpu.step(&mut self.inner)
            };
            self.spc_cycles += c as u64;
            self.inner.tick(c);
        }
    }

    pub fn take_samples(&mut self) -> Vec<i16> {
        std::mem::take(&mut self.inner.samples)
    }
}
