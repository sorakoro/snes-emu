//! CPU とバスを束ねるエミュレータ本体。

use crate::bus::MainBus;
use crate::cartridge::Cartridge;
use crate::cpu::Cpu;

// ジョイパッドのビット割り当て ($4218/$4219 の 16bit 表現)
pub const JOY_B: u16 = 0x8000;
pub const JOY_Y: u16 = 0x4000;
pub const JOY_SELECT: u16 = 0x2000;
pub const JOY_START: u16 = 0x1000;
pub const JOY_UP: u16 = 0x0800;
pub const JOY_DOWN: u16 = 0x0400;
pub const JOY_LEFT: u16 = 0x0200;
pub const JOY_RIGHT: u16 = 0x0100;
pub const JOY_A: u16 = 0x0080;
pub const JOY_X: u16 = 0x0040;
pub const JOY_L: u16 = 0x0020;
pub const JOY_R: u16 = 0x0010;

pub struct Snes {
    pub cpu: Cpu,
    pub bus: MainBus,
}

impl Snes {
    pub fn new(rom: Vec<u8>) -> Result<Snes, String> {
        let cart = Cartridge::new(rom)?;
        let mut bus = MainBus::new(cart);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        Ok(Snes { cpu, bus })
    }

    /// 1 フレーム分実行する。
    pub fn run_frame(&mut self, joy1: u16) {
        self.bus.joy1 = joy1;
        let target = self.bus.frame + 1;
        while self.bus.frame < target {
            self.cpu.step(&mut self.bus);
        }
    }

    pub fn framebuffer(&self) -> &[u32] {
        &self.bus.ppu.framebuffer
    }
}
