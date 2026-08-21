use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use minifb::{Key, Scale, Window, WindowOptions};
use snes_emu::ppu::{SCREEN_H, SCREEN_W};
use snes_emu::snes::{self, Snes};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// 32kHz ステレオのエミュレータ出力をデバイスレートへ線形補間しながら流す
struct AudioOut {
    queue: Arc<Mutex<VecDeque<(i16, i16)>>>,
    _stream: cpal::Stream,
}

impl AudioOut {
    fn new() -> Option<AudioOut> {
        let host = cpal::default_host();
        let device = host.default_output_device()?;
        let config = device.default_output_config().ok()?;
        let sample_format = config.sample_format();
        let cfg: cpal::StreamConfig = config.into();
        let queue: Arc<Mutex<VecDeque<(i16, i16)>>> = Arc::new(Mutex::new(VecDeque::new()));

        // デバイスのサンプル形式ごとにストリームを構築 (Linux/ALSA は i16 のことがある)
        let stream = match sample_format {
            cpal::SampleFormat::F32 => Self::make_stream::<f32>(&device, &cfg, queue.clone()),
            cpal::SampleFormat::I16 => Self::make_stream::<i16>(&device, &cfg, queue.clone()),
            cpal::SampleFormat::U16 => Self::make_stream::<u16>(&device, &cfg, queue.clone()),
            _ => None,
        }?;
        stream.play().ok()?;
        Some(AudioOut {
            queue,
            _stream: stream,
        })
    }

    fn make_stream<T>(
        device: &cpal::Device,
        cfg: &cpal::StreamConfig,
        q: Arc<Mutex<VecDeque<(i16, i16)>>>,
    ) -> Option<cpal::Stream>
    where
        T: cpal::SizedSample + cpal::FromSample<f32>,
    {
        let out_rate = cfg.sample_rate.0 as f64;
        let channels = cfg.channels as usize;
        let step = 32000.0 / out_rate;
        let mut acc = 0.0f64;
        let mut cur = (0i16, 0i16);
        let mut next = (0i16, 0i16);
        device
            .build_output_stream(
                cfg,
                move |data: &mut [T], _| {
                    let mut q = q.lock().unwrap();
                    for frame in data.chunks_mut(channels) {
                        acc += step;
                        while acc >= 1.0 {
                            acc -= 1.0;
                            cur = next;
                            next = q.pop_front().unwrap_or(cur);
                        }
                        let t = acc as f32;
                        let l = (cur.0 as f32 + (next.0 - cur.0) as f32 * t) / 32768.0;
                        let r = (cur.1 as f32 + (next.1 - cur.1) as f32 * t) / 32768.0;
                        frame[0] = T::from_sample(l);
                        if channels > 1 {
                            frame[1] = T::from_sample(r);
                        }
                    }
                },
                |e| eprintln!("音声エラー: {e}"),
                None,
            )
            .ok()
    }

    fn push(&self, samples: &[i16]) {
        let mut q = self.queue.lock().unwrap();
        for pair in samples.chunks_exact(2) {
            q.push_back((pair[0], pair[1]));
        }
    }

    fn queued(&self) -> usize {
        self.queue.lock().unwrap().len()
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let Some(rom_path) = args.get(1) else {
        eprintln!("使い方: snes-emu <ROM ファイル (.sfc/.smc)>");
        std::process::exit(1);
    };

    let rom = match std::fs::read(rom_path) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("ROM を読み込めません ({rom_path}): {e}");
            std::process::exit(1);
        }
    };

    let mut snes = match Snes::new(rom) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("ROM の解析に失敗: {e}");
            std::process::exit(1);
        }
    };
    println!(
        "タイトル: {}  マッピング: {:?}",
        snes.bus.cart.title, snes.bus.cart.map
    );

    // SRAM (セーブデータ) の読み込み: <ROM名>.srm
    let srm_path = std::path::Path::new(rom_path).with_extension("srm");
    if !snes.bus.cart.sram.is_empty() {
        if let Ok(data) = std::fs::read(&srm_path) {
            let n = data.len().min(snes.bus.cart.sram.len());
            snes.bus.cart.sram[..n].copy_from_slice(&data[..n]);
            println!("セーブデータ読み込み: {}", srm_path.display());
        }
    }

    let mut window = Window::new(
        &format!("snes-emu - {}", snes.bus.cart.title),
        SCREEN_W,
        SCREEN_H,
        WindowOptions {
            scale: Scale::X2,
            ..WindowOptions::default()
        },
    )
    .expect("ウィンドウの作成に失敗");
    window.set_target_fps(60);

    let audio = AudioOut::new();
    if audio.is_none() {
        eprintln!("音声デバイスが見つかりません (無音で続行)");
    }

    // 音声駆動ペーシング: オーディオキューの残量が 100ms を下回らないように
    // エミュレーションを進める (バッファ枯渇によるノイズを防ぐ)
    const AUDIO_TARGET: usize = 3200; // ステレオペア数 (32kHz × 100ms)

    let save_sram = |snes: &Snes| {
        if !snes.bus.cart.sram.is_empty() {
            if let Err(e) = std::fs::write(&srm_path, &snes.bus.cart.sram) {
                eprintln!("セーブデータ書き込み失敗: {e}");
            }
        }
    };

    let mut frame_count: u64 = 0;
    while window.is_open() && !window.is_key_down(Key::Escape) {
        let joy = read_joypad(&window);
        match &audio {
            Some(a) => {
                let mut safety = 0;
                while a.queued() < AUDIO_TARGET && safety < 16 {
                    snes.run_frame(joy);
                    a.push(&snes.bus.apu.take_samples());
                    safety += 1;
                }
            }
            None => {
                snes.run_frame(joy);
                snes.bus.apu.take_samples();
            }
        }
        window
            .update_with_buffer(snes.framebuffer(), SCREEN_W, SCREEN_H)
            .expect("画面更新に失敗");

        // 10 秒ごとに SRAM を自動保存
        frame_count += 1;
        if frame_count % 600 == 0 {
            save_sram(&snes);
        }
    }
    save_sram(&snes);
}

/// キー割り当て:
///   十字キー: カーソルキー / B: Z / A: X / Y: A / X: S
///   L: Q / R: W / Start: Enter / Select: 右 Shift
fn read_joypad(window: &Window) -> u16 {
    let mut joy = 0u16;
    let map: &[(Key, u16)] = &[
        (Key::Up, snes::JOY_UP),
        (Key::Down, snes::JOY_DOWN),
        (Key::Left, snes::JOY_LEFT),
        (Key::Right, snes::JOY_RIGHT),
        (Key::Z, snes::JOY_B),
        (Key::X, snes::JOY_A),
        (Key::A, snes::JOY_Y),
        (Key::S, snes::JOY_X),
        (Key::Q, snes::JOY_L),
        (Key::W, snes::JOY_R),
        (Key::Enter, snes::JOY_START),
        (Key::RightShift, snes::JOY_SELECT),
    ];
    for &(key, bit) in map {
        if window.is_key_down(key) {
            joy |= bit;
        }
    }
    joy
}
