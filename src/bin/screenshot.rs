//! ヘッドレス実行して指定フレーム後の画面を PPM で保存する検証ツール。
//! 使い方: screenshot <ROM> <フレーム数> <出力.ppm> [出力.wav]

use snes_emu::ppu::{SCREEN_H, SCREEN_W};
use snes_emu::snes::Snes;
use std::io::Write;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 4 {
        eprintln!("使い方: screenshot <ROM> <フレーム数> <出力.ppm> [出力.wav]");
        std::process::exit(1);
    }
    let rom = std::fs::read(&args[1]).expect("ROM 読み込み失敗");
    let frames: u32 = args[2].parse().expect("フレーム数が不正");
    let mut snes = Snes::new(rom).expect("ROM 解析失敗");
    eprintln!(
        "タイトル: {} マッピング: {:?}",
        snes.bus.cart.title, snes.bus.cart.map
    );
    // 第 5 引数: ボタン押下スケジュール (カンマ区切り)。
    // 各要素は "フレーム" (Start を 20 フレーム) か "フレーム:マスク16進" (同じく 20 フレーム)
    let presses: Vec<(u32, u16)> = args
        .get(5)
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
    let mut audio: Vec<i16> = Vec::new();
    for f in 0..frames {
        let mut joy = 0u16;
        for &(p, mask) in &presses {
            if (p..p + 20).contains(&f) {
                joy |= mask;
            }
        }
        snes.run_frame(joy);
        audio.extend(snes.bus.apu.take_samples());
    }
    if let Some(wav_path) = args.get(4) {
        write_wav(wav_path, &audio);
        let peak = audio.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0);
        eprintln!("音声: {} サンプル, ピーク振幅 {}", audio.len() / 2, peak);
    }
    let mut out = std::fs::File::create(&args[3]).expect("出力ファイル作成失敗");
    writeln!(out, "P6\n{SCREEN_W} {SCREEN_H}\n255").unwrap();
    for &px in snes.framebuffer() {
        out.write_all(&[(px >> 16) as u8, (px >> 8) as u8, px as u8])
            .unwrap();
    }
    eprintln!("保存: {}", args[3]);
}

fn write_wav(path: &str, samples: &[i16]) {
    let mut f = std::fs::File::create(path).expect("WAV 作成失敗");
    let data_len = samples.len() as u32 * 2;
    let sample_rate: u32 = 32000;
    let byte_rate = sample_rate * 4;
    f.write_all(b"RIFF").unwrap();
    f.write_all(&(36 + data_len).to_le_bytes()).unwrap();
    f.write_all(b"WAVEfmt ").unwrap();
    f.write_all(&16u32.to_le_bytes()).unwrap();
    f.write_all(&1u16.to_le_bytes()).unwrap(); // PCM
    f.write_all(&2u16.to_le_bytes()).unwrap(); // ステレオ
    f.write_all(&sample_rate.to_le_bytes()).unwrap();
    f.write_all(&byte_rate.to_le_bytes()).unwrap();
    f.write_all(&4u16.to_le_bytes()).unwrap();
    f.write_all(&16u16.to_le_bytes()).unwrap();
    f.write_all(b"data").unwrap();
    f.write_all(&data_len.to_le_bytes()).unwrap();
    for &s in samples {
        f.write_all(&s.to_le_bytes()).unwrap();
    }
}
