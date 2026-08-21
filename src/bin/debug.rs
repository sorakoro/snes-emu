//! デバッグ用: N フレーム実行後の PPU/APU 内部状態を表示し、
//! BG レイヤ別のスクリーンショットを出力する。
//! 使い方: debug <ROM> <フレーム数> <出力プレフィックス>

use snes_emu::ppu::{SCREEN_H, SCREEN_W};
use snes_emu::snes::Snes;
use std::io::Write;

fn dump_ppm(path: &str, fb: &[u32]) {
    let mut out = std::fs::File::create(path).unwrap();
    writeln!(out, "P6\n{SCREEN_W} {SCREEN_H}\n255").unwrap();
    for &px in fb {
        out.write_all(&[(px >> 16) as u8, (px >> 8) as u8, px as u8])
            .unwrap();
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let rom = std::fs::read(&args[1]).expect("ROM 読み込み失敗");
    let frames: u32 = args[2].parse().unwrap();
    let prefix = &args[3];
    let mut snes = Snes::new(rom).expect("ROM 解析失敗");
    // 第 4 引数: "フレーム[:マスク16進]" のカンマ区切り (各 20 フレーム押下)
    let presses: Vec<(u32, u16)> = args
        .get(4)
        .map(|s| {
            s.split(',')
                .filter_map(|x| {
                    let mut it = x.split(':');
                    let frame: u32 = it.next()?.parse().ok()?;
                    let mask = it
                        .next()
                        .and_then(|m| u16::from_str_radix(m, 16).ok())
                        .unwrap_or(0x1000);
                    Some((frame, mask))
                })
                .collect()
        })
        .unwrap_or_default();
    for f in 0..frames {
        let mut joy = 0u16;
        for &(p, mask) in &presses {
            if (p..p + 20).contains(&f) {
                joy |= mask;
            }
        }
        snes.run_frame(joy);
    }

    let ppu = &snes.bus.ppu;
    println!("--- PPU ---");
    println!(
        "mode={} bg3prio={} tilesize={:04b} tm={:02X} ts={:02X} forced_blank={} brightness={}",
        ppu.bg_mode, ppu.bg3_priority, ppu.tile_size_bits, ppu.tm, ppu.ts, ppu.forced_blank,
        ppu.brightness
    );
    println!(
        "obsel={:02X} cgwsel={:02X} cgadsub={:02X} setini={:02X} mosaic={:02X}",
        ppu.obsel, ppu.cgwsel, ppu.cgadsub, ppu.setini, ppu.mosaic
    );
    for bg in 0..4 {
        println!(
            "BG{}: sc={:02X} chr={:X} hofs={:03X} vofs={:03X}",
            bg + 1,
            ppu.bg_sc[bg],
            (ppu.bg_chr[bg / 2] >> ((bg % 2) * 4)) & 0xF,
            ppu.bg_hofs[bg] & 0x3FF,
            ppu.bg_vofs[bg] & 0x3FF
        );
    }
    println!(
        "windows: w12sel={:02X} w34sel={:02X} wobjsel={:02X} wh={:?} tmw={:02X} tsw={:02X}",
        ppu.w12sel, ppu.w34sel, ppu.wobjsel, ppu.wh, ppu.tmw, ppu.tsw
    );

    println!("--- CPU / 割り込み ---");
    println!(
        "nmitimen={:02X} htime={:03X} vtime={:03X} irq_flag={} cpu: pbr:pc={:02X}:{:04X} p={:02X} e={} waiting={}",
        snes.bus.nmitimen, snes.bus.htime, snes.bus.vtime, snes.bus.irq_flag,
        snes.cpu.pbr, snes.cpu.pc, snes.cpu.p, snes.cpu.e, snes.cpu.waiting
    );
    // PC サンプリング: 2000 命令実行して頻出アドレスを表示
    let mut hist: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
    for _ in 0..2000 {
        snes.cpu.step(&mut snes.bus);
        let key = ((snes.cpu.pbr as u32) << 16) | snes.cpu.pc as u32;
        *hist.entry(key).or_insert(0) += 1;
    }
    let mut top: Vec<_> = hist.into_iter().collect();
    top.sort_by(|a, b| b.1.cmp(&a.1));
    print!("PC 頻出: ");
    for (addr, n) in top.iter().take(8) {
        print!("{:06X}({}) ", addr, n);
    }
    println!();

    println!("--- APU ---");
    let apu = &snes.bus.apu;
    println!(
        "spc pc={:04X} stopped={} ports in={:02X?} out={:02X?}",
        apu.cpu.pc, apu.cpu.stopped, apu.inner.in_ports, apu.inner.out_ports
    );
    let dsp = &apu.inner.dsp;
    println!(
        "dsp: MVOL L/R={:02X}/{:02X} EVOL={:02X}/{:02X} FLG={:02X} KON(reg)={:02X} NON={:02X} EON={:02X} DIR={:02X}",
        dsp.regs[0x0C], dsp.regs[0x1C], dsp.regs[0x2C], dsp.regs[0x3C], dsp.regs[0x6C],
        dsp.regs[0x4C], dsp.regs[0x3D], dsp.regs[0x4D], dsp.regs[0x5D]
    );
    for v in 0..8 {
        println!(
            "  v{}: vol={:02X}/{:02X} pitch={:02X}{:02X} srcn={:02X} adsr={:02X}{:02X} gain={:02X} envx={:02X}",
            v,
            dsp.regs[v << 4],
            dsp.regs[(v << 4) | 1],
            dsp.regs[(v << 4) | 3],
            dsp.regs[(v << 4) | 2],
            dsp.regs[(v << 4) | 4],
            dsp.regs[(v << 4) | 5],
            dsp.regs[(v << 4) | 6],
            dsp.regs[(v << 4) | 7],
            dsp.regs[(v << 4) | 8],
        );
    }

    // レイヤ別ダンプ
    let saved_tm = snes.bus.ppu.tm;
    for (name, mask) in [
        ("bg1", 0x01u8),
        ("bg2", 0x02),
        ("bg3", 0x04),
        ("bg4", 0x08),
        ("obj", 0x10),
        ("all", saved_tm),
    ] {
        snes.bus.ppu.tm = mask;
        snes.bus.ppu.ts = 0;
        for line in 1..=SCREEN_H as u16 {
            snes.bus.ppu.render_scanline(line);
        }
        dump_ppm(&format!("{prefix}_{name}.ppm"), &snes.bus.ppu.framebuffer);
    }
    println!("レイヤ別ダンプ: {prefix}_{{bg1..4,obj,all}}.ppm");
}
