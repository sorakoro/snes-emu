//! SingleStepTests/65816 のテストベクタによる CPU 検証。
//!
//! 環境変数 SINGLESTEP_DIR に .json ファイル群のディレクトリを指定して実行する:
//!   SINGLESTEP_DIR=/path/to/v1 cargo test --release --test singlestep -- --nocapture
//! 未設定ならスキップ。

use serde::Deserialize;
use snes_emu::cpu::{Bus, Cpu};
use std::collections::HashMap;

#[derive(Deserialize)]
struct TestCase {
    name: String,
    initial: State,
    #[serde(rename = "final")]
    final_: State,
}

#[derive(Deserialize)]
struct State {
    pc: u16,
    s: u16,
    p: u8,
    a: u16,
    x: u16,
    y: u16,
    dbr: u8,
    d: u16,
    pbr: u8,
    e: u8,
    ram: Vec<(u32, u8)>,
}

struct MapBus {
    mem: HashMap<u32, u8>,
}

impl Bus for MapBus {
    fn read(&mut self, addr: u32) -> u8 {
        *self.mem.get(&(addr & 0xFF_FFFF)).unwrap_or(&0)
    }
    fn write(&mut self, addr: u32, v: u8) {
        self.mem.insert(addr & 0xFF_FFFF, v);
    }
    fn idle(&mut self) {}
}

fn run_case(case: &TestCase) -> Result<(), String> {
    let mut cpu = Cpu::new();
    let i = &case.initial;
    cpu.pc = i.pc;
    cpu.s = i.s;
    cpu.p = i.p;
    cpu.a = i.a;
    cpu.x = i.x;
    cpu.y = i.y;
    cpu.dbr = i.dbr;
    cpu.d = i.d;
    cpu.pbr = i.pbr;
    cpu.e = i.e != 0;
    if cpu.e {
        // E モードでは S の上位バイトはハードウェア的に 0x01 固定
        cpu.s = 0x0100 | (cpu.s & 0xFF);
    }

    let mut bus = MapBus {
        mem: i.ram.iter().map(|&(a, v)| (a, v)).collect(),
    };
    let opcode = bus.read(((i.pbr as u32) << 16) | i.pc as u32);
    if opcode == 0x44 || opcode == 0x54 {
        // MVN/MVP: テストベクタは 100 サイクル (= 14 反復) で打ち切られている。
        // 打ち切り時の期待 PC は次反復のオペランドフェッチ途中 (+2)。
        let mut completed = false;
        for _ in 0..14 {
            cpu.step(&mut bus);
            if cpu.a == 0xFFFF {
                completed = true;
                break;
            }
        }
        if !completed {
            cpu.pc = i.pc.wrapping_add(2);
        }
    } else {
        cpu.step(&mut bus);
    }

    let f = &case.final_;
    let mut errs = Vec::new();
    let mut chk = |what: &str, got: u32, want: u32| {
        if got != want {
            errs.push(format!("{what}: got {got:06X} want {want:06X}"));
        }
    };
    chk("pc", cpu.pc as u32, f.pc as u32);
    chk("s", cpu.s as u32, f.s as u32);
    chk("p", cpu.p as u32, f.p as u32);
    chk("a", cpu.a as u32, f.a as u32);
    chk("x", cpu.x as u32, f.x as u32);
    chk("y", cpu.y as u32, f.y as u32);
    chk("dbr", cpu.dbr as u32, f.dbr as u32);
    chk("d", cpu.d as u32, f.d as u32);
    chk("pbr", cpu.pbr as u32, f.pbr as u32);
    chk("e", cpu.e as u32, (f.e != 0) as u32);
    for &(addr, want) in &f.ram {
        let got = *bus.mem.get(&addr).unwrap_or(&0);
        if got != want {
            errs.push(format!("ram[{addr:06X}]: got {got:02X} want {want:02X}"));
        }
    }
    if errs.is_empty() {
        Ok(())
    } else {
        Err(errs.join(", "))
    }
}

#[test]
fn singlestep_vectors() {
    let Ok(dir) = std::env::var("SINGLESTEP_DIR") else {
        eprintln!("SINGLESTEP_DIR 未設定のためスキップ");
        return;
    };
    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .expect("SINGLESTEP_DIR を開けない")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    files.sort();
    assert!(!files.is_empty(), "テストファイルが見つからない");

    let mut total_fail = 0usize;
    let mut total = 0usize;
    for path in &files {
        let data = std::fs::read_to_string(path).unwrap();
        let cases: Vec<TestCase> = serde_json::from_str(&data).unwrap();
        let mut fails = 0;
        let mut shown = 0;
        for case in &cases {
            total += 1;
            if let Err(e) = run_case(case) {
                fails += 1;
                if shown < 3 {
                    eprintln!("  FAIL {}: {}", case.name, e);
                    shown += 1;
                }
            }
        }
        total_fail += fails;
        let name = path.file_name().unwrap().to_string_lossy();
        if fails > 0 {
            eprintln!("{name}: {fails}/{} 失敗", cases.len());
        } else {
            eprintln!("{name}: 全 {} パス", cases.len());
        }
    }
    assert_eq!(total_fail, 0, "{total_fail}/{total} ケース失敗");
}
