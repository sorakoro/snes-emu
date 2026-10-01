use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use minifb::{Key, Scale, ScaleMode, Window, WindowOptions};
use snes_emu::ppu::{SCREEN_H, SCREEN_W};
use snes_emu::snes::{self, Snes};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

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

    // SRAM (セーブデータ) は OS のデータディレクトリ配下に <ROM名>.srm として保存する。
    // 旧版は ROM の隣に置いていたため、読み込みはそちらにもフォールバックする。
    let legacy_srm_path = std::path::Path::new(rom_path).with_extension("srm");
    let srm_path = match dirs::data_dir() {
        Some(dir) => {
            let dir = dir.join("snes-emu");
            if let Err(e) = std::fs::create_dir_all(&dir) {
                eprintln!(
                    "セーブ用ディレクトリを作成できません ({}): {e}",
                    dir.display()
                );
            }
            dir.join(legacy_srm_path.file_name().unwrap())
        }
        None => legacy_srm_path.clone(),
    };
    if !snes.bus.cart.sram.is_empty() {
        let loaded = [&srm_path, &legacy_srm_path]
            .into_iter()
            .find_map(|p| std::fs::read(p).ok().map(|d| (p, d)));
        if let Some((path, data)) = loaded {
            let n = data.len().min(snes.bus.cart.sram.len());
            snes.bus.cart.sram[..n].copy_from_slice(&data[..n]);
            println!("セーブデータ読み込み: {}", path.display());
        }
        println!("セーブ先: {}", srm_path.display());
    }

    let mut window = Window::new(
        &format!("snes-emu - {}", snes.bus.cart.title),
        SCREEN_W,
        SCREEN_H,
        WindowOptions {
            scale: Scale::X2,
            resize: true,
            scale_mode: ScaleMode::AspectRatioStretch,
            ..WindowOptions::default()
        },
    )
    .expect("ウィンドウの作成に失敗");
    window.set_target_fps(60);

    // 音声駆動ペーシング: オーディオキューの残量が 100ms を下回らないように
    // エミュレーションを進める (バッファ枯渇によるノイズを防ぐ)
    const AUDIO_TARGET: usize = 3200; // ステレオペア数 (32kHz × 100ms)

    // エミュレーションは専用スレッドで回す。macOS では Spaces のスワイプ切り替え中に
    // メインスレッド (Cocoa イベントループ) が数百 ms ブロックされるため、
    // メインループ駆動だとオーディオキューが枯渇して音が途切れる。
    let shared_fb = Arc::new(Mutex::new(vec![0u32; SCREEN_W * SCREEN_H]));
    let shared_joy = Arc::new(AtomicU16::new(0));
    let running = Arc::new(AtomicBool::new(true));

    let emu_thread = {
        let shared_fb = shared_fb.clone();
        let shared_joy = shared_joy.clone();
        let running = running.clone();
        thread::spawn(move || {
            // cpal::Stream は Send でないため、ストリームもこのスレッド内で作る
            let audio = AudioOut::new();
            if audio.is_none() {
                eprintln!("音声デバイスが見つかりません (無音で続行)");
            }

            let save_sram = |snes: &Snes| {
                if !snes.bus.cart.sram.is_empty() {
                    if let Err(e) = std::fs::write(&srm_path, &snes.bus.cart.sram) {
                        eprintln!("セーブデータ書き込み失敗: {e}");
                    }
                }
            };

            let mut frame_count: u64 = 0;
            let mut next_frame = Instant::now();
            while running.load(Ordering::Relaxed) {
                let joy = shared_joy.load(Ordering::Relaxed);
                match &audio {
                    Some(a) => {
                        if a.queued() >= AUDIO_TARGET {
                            thread::sleep(Duration::from_millis(2));
                            continue;
                        }
                        snes.run_frame(joy);
                        a.push(&snes.bus.apu.take_samples());
                    }
                    None => {
                        // 音声なしのときは 60fps 相当の時間駆動でペーシングする
                        let now = Instant::now();
                        if next_frame > now {
                            thread::sleep(next_frame - now);
                        } else {
                            next_frame = now;
                        }
                        next_frame += Duration::from_nanos(16_666_667);
                        snes.run_frame(joy);
                        snes.bus.apu.take_samples();
                    }
                }
                shared_fb.lock().unwrap().copy_from_slice(snes.framebuffer());

                // 10 秒ごとに SRAM を自動保存
                frame_count += 1;
                if frame_count % 600 == 0 {
                    save_sram(&snes);
                }
            }
            save_sram(&snes);
        })
    };

    // メインスレッドは入力の読み取りと画面表示のみ
    let mut display = vec![0u32; SCREEN_W * SCREEN_H];
    while window.is_open() && !window.is_key_down(Key::Escape) {
        shared_joy.store(read_joypad(&window), Ordering::Relaxed);
        display.copy_from_slice(&shared_fb.lock().unwrap());
        window
            .update_with_buffer(&display, SCREEN_W, SCREEN_H)
            .expect("画面更新に失敗");
    }
    running.store(false, Ordering::Relaxed);
    emu_thread.join().expect("エミュレーションスレッドが異常終了");
}

/// キー割り当て (SNES パッドの菱形配置を IJKL に対応させている):
///   十字キー: WASD / カーソルキー
///   B: K (下) / A: L (右) / Y: J (左) / X: I (上)  (Z/X も B/A として使用可)
///   L: U / R: O / Start: Enter / Select: 右 Shift
fn read_joypad(window: &Window) -> u16 {
    let mut joy = 0u16;
    let map: &[(Key, u16)] = &[
        (Key::Up, snes::JOY_UP),
        (Key::Down, snes::JOY_DOWN),
        (Key::Left, snes::JOY_LEFT),
        (Key::Right, snes::JOY_RIGHT),
        (Key::W, snes::JOY_UP),
        (Key::S, snes::JOY_DOWN),
        (Key::A, snes::JOY_LEFT),
        (Key::D, snes::JOY_RIGHT),
        (Key::K, snes::JOY_B),
        (Key::L, snes::JOY_A),
        (Key::J, snes::JOY_Y),
        (Key::I, snes::JOY_X),
        (Key::Z, snes::JOY_B), // 旧割り当ての互換
        (Key::X, snes::JOY_A),
        (Key::U, snes::JOY_L),
        (Key::O, snes::JOY_R),
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
