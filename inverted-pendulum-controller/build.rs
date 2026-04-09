use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let codegen_dir = manifest_dir.join("codegen");
    let matlab_dir = manifest_dir.parent().unwrap().join("MATLAB");

    // MATLAB設計ファイル変更時に再ビルド
    println!("cargo:rerun-if-changed={}",
        matlab_dir.join("control").join("codegen").display());
    println!("cargo:rerun-if-changed={}",
        matlab_dir.join("control").join("design").display());
    println!("cargo:rerun-if-changed={}",
        matlab_dir.join("plant").display());
    println!("cargo:rerun-if-changed={}",
        matlab_dir.join("build.m").display());

    // codegen/ が存在しないか空の場合のみ再生成
    let needs_codegen = !codegen_dir.exists()
        || std::fs::read_dir(&codegen_dir)
            .map(|mut d| d.next().is_none())
            .unwrap_or(true);

    if needs_codegen {
        println!("cargo:warning=Running MATLAB code generation...");
        let status = Command::new("matlab")
            .args(["-batch", "cd MATLAB; build"])
            .current_dir(manifest_dir.parent().unwrap())
            .status()
            .expect("Failed to launch MATLAB. Is it installed and on PATH?");

        if !status.success() {
            panic!("MATLAB code generation failed with exit code: {:?}", status.code());
        }
    }

    // 生成されたCソースをコンパイル（interface/ディレクトリはMEX用なので除外）
    let c_files: Vec<PathBuf> = glob::glob(
        codegen_dir.join("**").join("*.c").to_str().unwrap()
    )
    .expect("Failed to glob codegen directory")
    .filter_map(|e| e.ok())
    .filter(|p| !p.components().any(|c| c.as_os_str() == "interface"))
    .collect();

    if c_files.is_empty() {
        panic!("No C source files found in {:?}. Code generation may have failed.", codegen_dir);
    }

    let mut build = cc::Build::new();
    for f in &c_files {
        build.file(f);
    }

    // ヘッダーインクルードパス（MATLAB Coderは複数サブディレクトリに生成することがある）
    build.include(&codegen_dir);
    for entry in std::fs::read_dir(&codegen_dir).unwrap() {
        let entry = entry.unwrap();
        if entry.file_type().unwrap().is_dir() {
            build.include(entry.path());
        }
    }

    // GCC/Clangターゲット（ARM組み込み）の場合のみfreestandingフラグ追加
    let target = env::var("TARGET").unwrap_or_default();
    if !target.contains("msvc") {
        build.flag("-ffreestanding");
        build.flag("-fno-builtin");
    }

    build
        .warnings(false)
        .compile("controller");
}
