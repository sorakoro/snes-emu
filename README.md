# snes-emu

Rust 製スーパーファミコン (SNES) エミュレータ。Windows / macOS / Linux 対応。

## 使い方

```sh
cargo run --release -- <ROM ファイル (.sfc/.smc)>
```

セーブデータ (SRAM) は ROM と同じ場所に `<ROM名>.srm` として自動保存されます。

## ビルド方法 (OS 別)

前提: [Rust](https://rustup.rs/) をインストールしておく。

### macOS

```sh
cargo build --release
```

### Windows

[Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
(C++ ワークロード) を入れた上で:

```powershell
cargo build --release
```

生成物: `target\release\snes-emu.exe`

### Linux (Debian / Ubuntu)

```sh
sudo apt-get install -y libasound2-dev \
  libx11-dev libxcursor-dev libxi-dev \
  libxkbcommon-dev libwayland-dev libgl1-mesa-dev
cargo build --release
```

(Fedora 系: `alsa-lib-devel libX11-devel libXcursor-devel libXi-devel
libxkbcommon-devel wayland-devel mesa-libGL-devel`)

### CI (GitHub Actions)

`.github/workflows/build.yml` により、GitHub に push すると
Linux / macOS (Intel・Apple Silicon) / Windows の4種のバイナリが
自動ビルドされます。`v*` タグを push すると GitHub Release に添付されます。

## キー割り当て

| SNES | キー | SNES | キー |
|------|------|------|------|
| 十字キー | WASD / カーソルキー | B (下) | K (または Z) |
| A (右) | L (または X) | Y (左) | J |
| X (上) | I | L / R | U / O |
| Start | Enter | Select | 右 Shift |

ボタン4つ (I/J/K/L) は SNES パッドの菱形配置と同じ並びです。

ESC で終了。ウィンドウは自由にリサイズ可能 (アスペクト比維持)。
フルスクリーンは macOS では緑ボタン、Windows/Linux では最大化で。

## 実装状況

- [x] **フェーズ1: CPU (65C816)** — 全 256 オペコード、8/16bit モード、
      エミュレーションモード、BCD 演算、割り込み。
      [SingleStepTests/65816](https://github.com/SingleStepTests/65816) の
      全 512 万テストベクタをパス。
- [x] カートリッジ (LoROM / HiROM 自動判定、SRAM)
- [x] メモリバス (WRAM、マスターサイクル計上、FastROM)
- [x] 汎用 DMA / HDMA、乗除算レジスタ、ジョイパッド (手動 + 自動読み取り)
- [x] **フェーズ2: PPU 描画** — BG Mode 0-7 (2/4/8BPP、16px タイル、64 タイル
      マップ)、スプライト、ウィンドウ、カラー演算 (加減算/半輝度)、
      Mode 7 行列変換、モザイク。テスト ROM で目視検証済み。
- [x] **フェーズ3: APU** — SPC700 (全 256 オペコード、
      [SingleStepTests/spc700](https://github.com/SingleStepTests/spc700) の
      全 25.6 万ベクタをパス) + IPL ROM + タイマ + S-DSP
      (BRR、ADSR/GAIN、エコー、ノイズ、ピッチ変調)。cpal で音声出力。
- [ ] フェーズ4: 精度向上 — H/V IRQ のドット精度、ガウシアン補間、
      オフセットパータイル (Mode 2/4/6)、疑似ハイレゾ (Mode 5/6 の 512px)、
      直接カラー、オープンバス詳細、スプライトのライン上限 (32 個/34 タイル)

## テスト

```sh
cargo test                     # ユニットテスト
# CPU テストベクタ (要ダウンロード)
SINGLESTEP_DIR=/path/to/65816/v1 cargo test --release --test singlestep
SPC_SINGLESTEP_DIR=/path/to/spc700/v1 cargo test --release --test spc_singlestep
```

検証用ヘッドレスツール (画面 PPM + 音声 WAV を出力):

```sh
cargo run --release --bin screenshot -- <ROM> <フレーム数> out.ppm [out.wav]
```

## 構成

```
src/
  cpu.rs         65C816 コア (Bus トレイトで駆動、サイクルはバス側で計上)
  bus.rs         メモリマップ / CPU I/O / DMA / HDMA / タイミング
  cartridge.rs   ROM ヘッダ解析とマッピング
  ppu.rs         PPU レジスタ + VRAM/CGRAM/OAM ポート
  ppu_render.rs  スキャンラインレンダラ (BG/OBJ/ウィンドウ/カラー演算/Mode7)
  spc700.rs      SPC700 コア
  apu.rs         APU 統合 (ARAM / IPL ROM / タイマ / ポート)
  dsp.rs         S-DSP (BRR / エンベロープ / エコー / ノイズ)
  snes.rs        統合 (フレーム実行)
  main.rs        minifb + cpal フロントエンド
```
