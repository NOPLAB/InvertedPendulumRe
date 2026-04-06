# MATLAB コードリファクタリング設計

## 概要

MATLABシミュレーションコードの整理・構造改善を行う。デッドコードの削除、FirmwareExperiment.m（742行）の機能別分割、Rustエクスポートユーティリティの統合を実施する。外部から見た動作（シミュレーション結果、プロット出力）は一切変更しない。

## 変更前の構造

```
MATLAB/
├── main.m                              (150行)
├── control/
│   ├── design_lqr.m                    (10行)
│   ├── design_pid.m                    (14行)
│   ├── design_mrac.m                   (61行)
│   ├── design_observer.m              (33行)
│   ├── design_mpc.m                    (84行)
│   ├── export_lqr_rust.m              (18行)
│   ├── export_pid_rust.m              (20行)
│   ├── export_mrac_rust.m             (66行)
│   └── export_observer_rust.m         (40行)
├── plant/
│   ├── params.m                        (23行)
│   ├── plant_model.m                   (33行)
│   ├── linearize_system.m             (48行)
│   └── linearize_mode4_voltage_system.m (23行)
├── sim/
│   ├── FirmwareExperiment.m            (742行)
│   ├── simulate_firmware.m            (20行)
│   ├── simulate_nonlinear.m           (160行)
│   ├── mpc_controller.m              (54行)
│   ├── animate_pendulum.m            (57行)
│   └── compare_controllers.m         (61行)
└── simulink/
    ├── build_simulink_model.m         (251行)
    └── run_simulink_model.m           (65行)
```

## 変更後の構造

```
MATLAB/
├── main.m                              (微修正のみ)
├── control/
│   ├── design_lqr.m                    (既存)
│   ├── design_pid.m                    (既存)
│   ├── design_mrac.m                   (既存)
│   ├── design_observer.m              (既存)
│   ├── design_mpc.m                    (既存)
│   ├── export_lqr_rust.m              (ヘルパー削除、export_utils呼び出しに変更)
│   ├── export_pid_rust.m              (同上)
│   ├── export_mrac_rust.m             (同上)
│   ├── export_observer_rust.m         (同上)
│   ├── export_utils.m                  (新規: 共通Rustエクスポートヘルパー)
│   ├── step_pid.m                      (新規: PIDステップ処理)
│   ├── step_lqr.m                      (新規: LQRステップ処理)
│   ├── step_mrac.m                     (新規: MRACステップ処理)
│   └── step_mpc.m                      (新規: MPCステップ処理)
├── plant/
│   ├── params.m                        (既存)
│   ├── plant_model.m                   (既存)
│   ├── linearize_system.m             (既存)
│   └── linearize_voltage_input.m      (リネーム: mode4→voltage_input)
├── sim/
│   ├── FirmwareExperiment.m            (コアのみに縮小、約250行)
│   ├── sim_utils.m                     (新規: LPF、構造体マージ等)
│   ├── sim_options.m                   (新規: デフォルトオプション生成)
│   ├── simulate_firmware.m            (既存)
│   └── mpc_controller.m              (既存)
└── simulink/
    ├── build_simulink_model.m         (対象外)
    └── run_simulink_model.m           (対象外)
```

## 変更内容の詳細

### 1. ファイル削除（デッドコード除去）

| ファイル | 行数 | 削除理由 |
|---------|------|---------|
| `sim/simulate_nonlinear.m` | 160 | FirmwareExperimentで完全に置き換え済み。main.mから未使用 |
| `sim/animate_pendulum.m` | 57 | 未使用のアニメーションユーティリティ |
| `sim/compare_controllers.m` | 61 | main.mの比較ロジックで置き換え済み |

### 2. リネーム

- `plant/linearize_mode4_voltage_system.m` → `plant/linearize_voltage_input.m`
  - 関数名を `linearize_voltage_input` に変更
  - ファイル内コメントも更新
  - 呼び出し元（`control/design_mrac.m`、`sim/FirmwareExperiment.m`）を修正

### 3. FirmwareExperiment.m の分割

現在の742行を以下の構成に分割する。

#### `sim/FirmwareExperiment.m`（コア、約250行）
- シミュレーションループ（RK4積分、5kHz電流ループ、1kHzバランスループ）
- `run()` メソッド
- モード別に `step_*.m` を呼び出すディスパッチ処理

#### `control/step_pid.m`（約40行）
- `balance_pid_step` 相当のロジック
- 入力: 状態ベクトル、ゲイン、フィルタ状態
- 出力: 制御入力（力）、更新されたフィルタ状態

#### `control/step_lqr.m`（約20行）
- LQR状態フィードバック `u = -K * x_hat`

#### `control/step_mrac.m`（約80行）
- 適応則の更新、ソフトゾーン処理、電圧指令計算

#### `control/step_mpc.m`（約30行）
- `mpc_controller.m` の呼び出しラッパー

#### `sim/sim_utils.m`（約80行）
- `init_lpf` / `lpf_update` — ローパスフィルタの初期化・更新
- `merge_structs` — 構造体マージユーティリティ
- `normalize_mode` — モード文字列の正規化

#### `sim/sim_options.m`（約100行）
- `default_options` — デフォルトオプション生成
- オブザーバ行列は `design_observer.m` を呼び出して動的に計算（ハードコード除去）

### 4. Rustエクスポートユーティリティ統合

#### 新規: `control/export_utils.m`（約40行）
- `print_rust_scalar(fid, name, value)` — スカラー値出力
- `print_rust_vector(fid, name, vec)` — ベクトル出力
- `print_rust_matrix(fid, name, mat)` — 行列出力

#### 既存4ファイルの修正
- 各ファイル内のローカルヘルパー関数を削除
- `export_utils` の関数を呼び出す形に変更
- エクスポート対象パラメータの選定ロジック（各ファイル固有）はそのまま維持

### 5. main.m への影響

- `linearize_mode4_voltage_system` → `linearize_voltage_input` の呼び出し修正（該当箇所がある場合）
- パス管理: `addpath(genpath(...))` を既に使用しているため、新ファイルは自動認識される（変更不要）
- 削除ファイルの呼び出しは既にないため除去不要

## 対象外

- `simulink/` 配下のファイル（`build_simulink_model.m`、`run_simulink_model.m`）は今回の対象外
- 制御設計ファイル（`design_*.m`）の内部ロジックは変更しない

## 制約

- **外部動作不変**: main.mの実行結果（シミュレーション結果、プロット出力）は一切変わらない
- **MATLABパス**: `addpath(genpath(...))` により新規ファイルは自動認識される
