# inverted-pendulum-esp32 設計仕様

## 概要

ESP32-C3-WROOM-02上のEmbassy Rustファームウェア。KiCad基板上でSTM32F303K8とUART接続し、BLEおよびWiFi無線インターフェースを提供する通信モジュール。

制御ロジックはSTM32側が担当し、ESP32-C3はセンサデータの無線配信とリモートコマンドの中継を行う。

## アーキテクチャ

```
[STM32F303K8] ──UART(COBS+Binary)──> [ESP32-C3] ──BLE GATT──> [スマホアプリ]
                                          │
                                          └──WiFi WebSocket──> [ブラウザ/PC]
```

### クレート構成

```
InvertedPendulumRe/
├── inverted-pendulum-protocol/    # 共有プロトコルクレート (no_std lib)
│   ├── Cargo.toml
│   └── src/
│       └── lib.rs
├── inverted-pendulum-stm32/       # 既存STM32ファームウェア (protocol依存追加)
│   └── Cargo.toml
└── inverted-pendulum-esp32/       # 新規ESP32-C3ファームウェア
    ├── Cargo.toml
    ├── build.rs
    ├── rust-toolchain.toml
    └── src/
        ├── main.rs
        ├── ble.rs
        ├── wifi.rs
        ├── uart.rs
        └── led.rs
```

## inverted-pendulum-protocol クレート

`no_std`ライブラリ。STM32/ESP32両方から依存する共有プロトコル定義。

### データ型

```rust
/// センサデータフレーム (STM32 → ESP32)
pub struct SensorData {
    pub theta: f32,        // 振子角度 [rad]
    pub position: f32,     // カート位置 [m]
    pub velocity: f32,     // カート速度 [m/s]
    pub theta_dot: f32,    // 角速度 [rad/s]
    pub current_l: f32,    // 左モーター電流 [A]
    pub current_r: f32,    // 右モーター電流 [A]
    pub control_mode: u8,  // 制御モード (0-4)
    pub running: u8,       // 動作中フラグ
}
// シリアライズサイズ: 26 bytes + CRC8 1 byte = 27 bytes
// COBSエンコード後最大: 29 bytes

/// コマンドフレーム (ESP32 → STM32)
pub struct Command {
    pub command_type: CommandType,
    pub payload: [u8; 2],
}
// シリアライズサイズ: 3 bytes + CRC8 1 byte = 4 bytes
// COBSエンコード後最大: 6 bytes

pub enum CommandType {
    Start = 0x01,
    Stop = 0x02,
    ModeChange = 0x03,  // payload[0] = ControlMode
    ParamSet = 0x04,    // payload[0] = param_id, payload[1] = value
}

pub enum ControlMode {
    Debug = 0,
    Pid = 1,
    Lqr = 2,
    Mrac = 3,
    Mpc = 4,
}
```

### 機能

- `SensorData::serialize(&self, buf: &mut [u8]) -> usize` — 構造体 → バイナリ (リトルエンディアン)
- `SensorData::deserialize(buf: &[u8]) -> Result<Self, ProtocolError>` — バイナリ → 構造体
- `Command::serialize(&self, buf: &mut [u8]) -> usize`
- `Command::deserialize(buf: &[u8]) -> Result<Self, ProtocolError>`
- `cobs_encode(data: &[u8], out: &mut [u8]) -> usize` — COBSエンコード
- `cobs_decode(data: &[u8], out: &mut [u8]) -> Result<usize, ProtocolError>` — COBSデコード
- `crc8(data: &[u8]) -> u8` — CRC8計算
- `verify_crc8(data: &[u8]) -> bool` — CRC8検証 (末尾1byteがCRC)
- フレームサイズ定数 (`SENSOR_DATA_SIZE`, `COMMAND_SIZE`, `MAX_COBS_FRAME_SIZE` 等)

### 依存

- `cobs` (no_std対応)
- feature `defmt`: `defmt::Format` derive追加

## inverted-pendulum-esp32 クレート

### ピン割り当て（KiCad準拠）

| GPIO | 機能 | 備考 |
|------|------|------|
| GPIO20 (RXD) | UART RX | ← STM32 TX |
| GPIO21 (TXD) | UART TX | → STM32 RX |
| GPIO3 | LED_1 | BLE接続状態表示 |
| GPIO6 | LED_2 | WiFi接続状態表示 |
| GPIO0 | BOOTボタン | 起動時のみ使用 |

### Embassyタスク構成

| タスク | 駆動方式 | 機能 |
|--------|----------|------|
| `uart_rx_task` | イベント駆動 | UART受信 → COBSデコード → CRC検証 → 共有ステートに書き込み |
| `uart_tx_task` | イベント駆動 | コマンドキューからpop → CRC付加 → COBSエンコード → UART送信 |
| `ble_task` | イベント駆動 | BLE GATTサーバー運用 (notify/write) |
| `wifi_task` | イベント駆動 | WiFi接続管理 + WebSocketサーバー |
| `bridge_task` | 周期(10-50ms) | 共有ステート → BLE notify / WebSocket broadcast、BLE/WiFi受信コマンド → uart_txキュー |
| `led_task` | 状態連動 | LED_1: BLE接続状態、LED_2: WiFi接続状態 |

### 共有データ構造

```rust
// センサデータ: uart_rx_task が書き込み、bridge_task が読み出し
// Signal<CriticalSectionRawMutex, SensorData> で最新値のみ保持

// コマンド: ble_task/wifi_task が投入、uart_tx_task が消費
// Channel<CriticalSectionRawMutex, Command, 4> キュー
```

### BLE設計

- デバイス名: `"InvertedPendulum"`
- BLEスタック: `trouble-host` (Embassy対応 pure-Rust BLE)
- GATT Service UUID: `ff94ca6f-91a5-4048-b805-755bca71075e` (リファレンス流用)
- TX Characteristic (Notify): `b2322be9-f43c-4431-8ec1-d90e3b8bd848`
  - SensorDataをバイナリのままnotify配信
- RX Characteristic (Write): `a3308588-5d66-4643-b0ef-0df52539d950`
  - Commandバイナリを受信

### WiFi設計

- モード: STA (既存ネットワーク接続)、接続失敗時APフォールバック
- APフォールバック SSID: `InvertedPendulum`
- WebSocketサーバー: ポート80
- 送信: SensorDataをJSON形式でbroadcast
  ```json
  {"theta":0.01,"position":0.05,"velocity":0.1,"theta_dot":0.02,"current_l":0.5,"current_r":0.5,"mode":2,"running":true}
  ```
- 受信: コマンドをJSON形式で受信
  ```json
  {"command":"mode_change","mode":2}
  {"command":"start"}
  {"command":"stop"}
  ```
- WiFi SSID/パスワードはコンパイル時定数 (env!マクロまたはconstants)

### UART設定

- ボーレート: 115200
- データ: 8N1
- フロー制御: なし
- フレーム区切り: COBS (0x00がデリミタ)

### 依存クレート

| クレート | 用途 |
|----------|------|
| `esp-hal` | ESP32-C3 HAL (GPIO, UART, タイマー) |
| `esp-wifi` | WiFi + BLEコントローラードライバ |
| `embassy-executor` | asyncタスクランタイム |
| `embassy-time` | タイマー・ディレイ |
| `embassy-sync` | Signal, Channel等の同期プリミティブ |
| `trouble-host` | BLE GATTスタック |
| `inverted-pendulum-protocol` | 共有プロトコル (path依存) |
| `defmt` + `esp-println` | デバッグログ (featureフラグ) |
| `embedded-io-async` | async UART read/write trait |
| `heapless` | no_std String/Vec (JSON生成用) |

### features

- `default = ["debug"]`
- `debug`: defmtログ有効化

## STM32側の変更 (inverted-pendulum-stm32)

### 追加タスク

| タスク | 機能 |
|--------|------|
| `uart_tx_task` | balance loop から10Hz decimationでSensorDataをCOBS送信 |
| `uart_rx_task` | UART受信 → COBSデコード → コマンド処理 (モード切替/Start/Stop) |

### UARTピン

KiCad回路図よりSTM32側のUARTピン:
- PA2 (TX) / PA3 (RX) — KiCad基板上でESP32-C3のRXD/TXDに接続済み (USART2)

### 依存追加

- `inverted-pendulum-protocol` (path依存)

### 変更範囲

- `Cargo.toml`: protocol依存追加
- `main.rs`: UARTペリフェラル初期化、uart_tx/rx_taskのspawn
- `controller/mod.rs`: `ControlMode` enumをprotocolクレートから再エクスポート
- 新規 `uart.rs`: UART送受信タスク実装

## エラーハンドリング

- UART: CRC不一致フレームは無言で破棄、COBSデコード失敗も同様
- BLE: 切断時はadvertising再開
- WiFi: 切断時は再接続ループ、3回失敗でAPモードフォールバック
- バッファオーバーフロー: 固定長バッファ、超過データは切り捨て

## LED表示パターン

| LED_1 (GPIO3) | 状態 |
|----------------|------|
| 消灯 | BLE未接続 |
| 点灯 | BLE接続中 |
| 点滅(200ms) | BLE advertising中 |

| LED_2 (GPIO6) | 状態 |
|----------------|------|
| 消灯 | WiFi未接続 |
| 点灯 | WiFi接続中 (STA) |
| 点滅(200ms) | WiFi接続試行中 |
| 高速点滅(100ms) | APモード動作中 |
