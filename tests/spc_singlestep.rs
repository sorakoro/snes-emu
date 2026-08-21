//! SingleStepTests/spc700 のテストベクタによる SPC700 検証。
//! SPC_SINGLESTEP_DIR に v1/*.json のディレクトリを指定して実行。未設定ならスキップ。

use serde::Deserialize;
use snes_emu::spc700::{Spc700, SpcBus};

#[derive(Deserialize)]
struct TestCase {
    name: String,
    initial: State,
    #[serde(rename = "final")]
    final_: State,
    cycles: Vec<serde_json::Value>,
}

#[derive(Deserialize)]
struct State {
    pc: u16,
    a: u8,
    x: u8,
    y: u8,
    sp: u8,
    psw: u8,
    ram: Vec<(u16, u8)>,
}

struct FlatBus {
    mem: Vec<u8>,
}

impl SpcBus for FlatBus {
    fn read(&mut self, addr: u16) -> u8 {
        self.mem[addr as usize]
    }
    fn write(&mut self, addr: u16, v: u8) {
        self.mem[addr as usize] = v;
    }
}

fn run_case(case: &TestCase) -> Result<(), String> {
    let mut cpu = Spc700::new();
    let i = &case.initial;
    cpu.pc = i.pc;
    cpu.a = i.a;
    cpu.x = i.x;
    cpu.y = i.y;
    cpu.sp = i.sp;
    cpu.psw = i.psw;

    let mut bus = FlatBus {
        mem: vec![0; 0x10000],
    };
    for &(a, v) in &i.ram {
        bus.mem[a as usize] = v;
    }
    let cycles = cpu.step(&mut bus);

    let f = &case.final_;
    let mut errs = Vec::new();
    let mut chk = |what: &str, got: u32, want: u32| {
        if got != want {
            errs.push(format!("{what}: got {got:04X} want {want:04X}"));
        }
    };
    chk("pc", cpu.pc as u32, f.pc as u32);
    chk("a", cpu.a as u32, f.a as u32);
    chk("x", cpu.x as u32, f.x as u32);
    chk("y", cpu.y as u32, f.y as u32);
    chk("sp", cpu.sp as u32, f.sp as u32);
    chk("psw", cpu.psw as u32, f.psw as u32);
    chk("cycles", cycles, case.cycles.len() as u32);
    for &(addr, want) in &f.ram {
        let got = bus.mem[addr as usize];
        if got != want {
            errs.push(format!("ram[{addr:04X}]: got {got:02X} want {want:02X}"));
        }
    }
    if errs.is_empty() {
        Ok(())
    } else {
        Err(errs.join(", "))
    }
}

#[test]
fn spc700_vectors() {
    let Ok(dir) = std::env::var("SPC_SINGLESTEP_DIR") else {
        eprintln!("SPC_SINGLESTEP_DIR 未設定のためスキップ");
        return;
    };
    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .expect("ディレクトリを開けない")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    files.sort();
    assert!(!files.is_empty());

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
        }
    }
    assert_eq!(total_fail, 0, "{total_fail}/{total} ケース失敗");
}
