# MATLAB Coder → Rust FFI 制御器コード生成設計

## 概要

MATLABで設計した制御器をEmbedded CoderでCコード生成し、Rustクレートとして提供する。シミュレーション（MEX）と実機（Cライブラリ → Rust FFI）で同一の生成コードを使用し、MATLAB設計とSTM32実機の数値一致を保証する。

## 要件

- MATLAB Embedded Coder (R2025b) でCコードを生成
- 対象: LQR, PID, MPC, MRAC, State Observer の全制御アルゴリズム
- 生成範囲: `ProcessedState` → `force` の制御アルゴリズムのみ（前処理・電流制御ループはRustのまま）
- `build.rs` から MATLAB を呼び出しフル自動ビルド
- 独立クレート `inverted-pendulum-controller/` としてSTM32・ESP32等から再利用可能
- `codegen/` は `.gitignore`（MATLABがある環境前提）

## ディレクトリ構成

```
InvertedPendulumRe/
├── MATLAB/
│   ├── main.m                          # シミュレーション・解析パイプライン
│   ├── build.m                         # コード生成パイプライン（新規）
│   ├── plant/                          # 既存（params, linearize）
│   ├── control/
│   │   ├── design/                     # 既存（design_lqr.m 等）
│   │   ├── export/                     # 既存 → 将来廃止
│   │   └── codegen/                    # 新規: MATLAB Coder関連
│   │       ├── codegen_controller.m    # コード生成メインスクリプト
│   │       ├── codegen_config.m        # Embedded Coder設定（共通）
│   │       └── entry_points/           # Coder用エントリ関数
│   │           ├── lqr_step.m
│   │           ├── pid_step.m
│   │           ├── mpc_step.m
│   │           ├── mrac_step.m
│   │           └── observer_step.m
│   └── sim/
│       ├── simulate_firmware.m         # MEX経由で制御器呼び出しに変更
│       └── mex/                        # MEX生成物出力先（.gitignore）
│
├── inverted-pendulum-controller/       # 新規クレート
│   ├── Cargo.toml
│   ├── build.rs                        # MATLAB呼び出し + ccコンパイル
│   ├── codegen/                        # MATLAB Coder出力先（.gitignore）
│   ├── src/
│   │   ├── lib.rs                      # pub API: safe Rustラッパー
│   │   └── ffi.rs                      # extern "C" 宣言
│   └── .gitignore                      # codegen/ を除外
│
├── inverted-pendulum-stm32/            # 既存（controller crateに依存追加）
├── inverted-pendulum-protocol/         # 既存（変更なし）
└── inverted-pendulum-esp32/            # 既存（将来controller crate利用可能）
```

## MATLAB側: コード生成

### build.m

コード生成専用エントリポイント。`main.m` から分離し独立実行可能。

```matlab
% build.m
addpath(genpath('.'));

% 1. プラント・設計
p = params();
[A, B, C, D] = linearize_system(p);

% 2. 全制御器を設計
K_lqr = design_lqr(A, B);
pid_gains = design_pid(A, B);
mpc_params = design_mpc(A, B);
mrac_params = design_mrac(A, B);
obs_params = design_observer(A, B, C);

% 3. コード生成（MEX + Cライブラリ）
codegen_controller(K_lqr, pid_gains, mpc_params, mrac_params, obs_params);
```

### エントリポイント関数

各制御器は統一的なインターフェースを持つ:

| 関数 | 入力 | 出力 | 内部状態 |
|------|------|------|----------|
| `lqr_step(state)` | `single(4x1)` | `single` force | なし |
| `pid_step(state)` | `single(4x1)` | `single` force | persistent（積分器） |
| `mpc_step(state)` | `single(4x1)` | `single` force | persistent（ADMM warm start） |
| `mrac_step(state)` | `single(4x1)` | `single` force | persistent（適応パラメータ） |
| `observer_step(position, theta, force)` | `single` x3 | `single(4x1)` x_hat | persistent（推定状態） |

設計パラメータは `codegen_controller.m` が事前計算し、`coder.Constant` またはグローバル変数としてコード生成時に定数埋め込みする。

### codegen_config.m

Embedded Coder共通設定:
- ハードウェアターゲット: ARM Cortex-M
- 数値型: `single`（f32）
- 動的メモリ割り当て: 無効
- 出力ターゲット:
  - `coder.config('lib')` → `../../inverted-pendulum-controller/codegen/`
  - `coder.config('mex')` → `../sim/mex/`

### シミュレーション統合

`simulate_firmware.m` はコード生成されたMEX関数を呼び出す:

```matlab
force = lqr_step_mex(single(state));
```

これによりシミュレーションと実機で同一の制御コードが実行される。

## Rustクレート: inverted-pendulum-controller

### Cargo.toml

```toml
[package]
name = "inverted-pendulum-controller"
version = "0.1.0"
edition = "2021"

[build-dependencies]
cc = "1"
glob = "0.3"
```

### build.rs

1. MATLAB設計ファイルの変更を検知（`cargo:rerun-if-changed`）
2. `codegen/` が古い or 存在しない場合、MATLABをヘッドレス起動（`matlab -batch "cd ../MATLAB; build"`）
3. `cc::Build` で生成Cソースを `thumbv7em-none-eabi` 向けにクロスコンパイル
4. `arm-none-eabi-gcc` が必要

```rust
// build.rs（概要）
fn main() {
    let codegen_dir = PathBuf::from("codegen");

    if should_regenerate(&codegen_dir) {
        // matlab -batch "cd ../MATLAB; build"
        run_matlab_codegen();
    }

    cc::Build::new()
        .files(glob("codegen/*.c"))
        .flag("-ffreestanding")
        .compile("controller");
}
```

### ffi.rs

```rust
extern "C" {
    pub fn lqr_step_init();
    pub fn lqr_step(state: *const f32) -> f32;

    pub fn pid_step_init();
    pub fn pid_step(state: *const f32) -> f32;

    pub fn mpc_step_init();
    pub fn mpc_step(state: *const f32) -> f32;

    pub fn mrac_step_init();
    pub fn mrac_step(state: *const f32) -> f32;

    pub fn observer_step_init();
    pub fn observer_step(position: f32, theta: f32, force: f32, x_hat: *mut f32);
}
```

### lib.rs

```rust
#![no_std]

mod ffi;

pub fn init_lqr() { unsafe { ffi::lqr_step_init(); } }
pub fn init_pid() { unsafe { ffi::pid_step_init(); } }
pub fn init_mpc() { unsafe { ffi::mpc_step_init(); } }
pub fn init_mrac() { unsafe { ffi::mrac_step_init(); } }
pub fn init_observer() { unsafe { ffi::observer_step_init(); } }

pub fn lqr_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::lqr_step(state.as_ptr()) }
}

pub fn pid_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::pid_step(state.as_ptr()) }
}

pub fn mpc_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::mpc_step(state.as_ptr()) }
}

pub fn mrac_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::mrac_step(state.as_ptr()) }
}

pub fn observer_update(position: f32, theta: f32, force: f32) -> [f32; 4] {
    let mut x_hat = [0.0f32; 4];
    unsafe { ffi::observer_step(position, theta, force, x_hat.as_mut_ptr()); }
    x_hat
}
```

## STM32側の変更

### 依存追加

```toml
# inverted-pendulum-stm32/Cargo.toml
[dependencies]
inverted-pendulum-controller = { path = "../inverted-pendulum-controller" }
```

### controller/mod.rs の変更

```rust
use inverted_pendulum_controller as ctrl;

pub struct ControlSystem {
    mode: ControlMode,
    current_pid_left: Pid,
    current_pid_right: Pid,
    current_lpf_left: Filter,
    current_lpf_right: Filter,
}

impl ControlSystem {
    pub fn set_mode(&mut self, mode: ControlMode) {
        match mode {
            ControlMode::Lqr => ctrl::init_lqr(),
            ControlMode::Pid => ctrl::init_pid(),
            ControlMode::Mpc => ctrl::init_mpc(),
            ControlMode::Mrac => ctrl::init_mrac(),
            _ => {}
        }
        ctrl::init_observer();
        self.mode = mode;
    }

    pub fn update_balance(&mut self, state: &ProcessedState) -> f32 {
        let x_hat = ctrl::observer_update(state.x, state.theta, state.last_force);

        match self.mode {
            ControlMode::Lqr => ctrl::lqr_compute(&x_hat),
            ControlMode::Pid => ctrl::pid_compute(&x_hat),
            ControlMode::Mpc => ctrl::mpc_compute(&x_hat),
            ControlMode::Mrac => ctrl::mrac_compute(&x_hat),
            _ => 0.0,
        }
    }

    // update_current は既存のRust PIDをそのまま使用
}
```

### 削除するファイル

- `controller/lqr.rs` + `lqr_constants.rs`
- `controller/pid_balance.rs` + `pid_balance_constants.rs`
- `controller/mpc.rs` + `mpc_constants.rs`
- `controller/mrac.rs` + `mrac_constants.rs`
- `controller/observer.rs` + `observer_constants.rs`

### 残すファイル

- `controller/mod.rs` — 統合ロジック（簡素化）
- `controller/pid.rs` — 汎用PID（電流制御ループ用）
- `controller/filter.rs` — ローパスフィルタ（電流フィルタリング用）
- `tasks/control.rs` — タスクスケジューリング（変更最小限）

### tasks/mpc.rs について

現在MPC計算を非同期タスクで50Hzに間引いているが、Embedded Coderの最適化Cコードであれば1kHzループ内で直接実行できる可能性がある。実測で判断し、不要であれば `tasks/mpc.rs` も削除。

## ビルド要件

- MATLAB R2025b + MATLAB Coder + Embedded Coder
- Rust nightly（build-std）
- `arm-none-eabi-gcc`（Cクロスコンパイル用）
- `probe-rs`（フラッシュ書き込み用）
