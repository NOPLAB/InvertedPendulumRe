# inverted-pendulum-esp32 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create an ESP32-C3 Embassy firmware that bridges STM32 sensor data to BLE and WiFi clients, plus a shared protocol crate.

**Architecture:** Three crates — `inverted-pendulum-protocol` (shared no_std lib for COBS-framed binary protocol), `inverted-pendulum-esp32` (ESP32-C3 Embassy firmware with BLE GATT + WiFi WebSocket), and modifications to `inverted-pendulum-stm32` (UART tx/rx tasks). The ESP32-C3 receives sensor data from STM32 via UART and bridges it to wireless clients. Commands flow in reverse.

**Tech Stack:** Rust (stable), Embassy async runtime, esp-hal 1.0.0, esp-rtos 0.2.0, esp-radio 0.17.0, trouble-host 0.6.0 (BLE GATT), COBS framing, CRC8 checksums.

**Note on Embassy versions:** The STM32 project uses embassy-executor 0.10.0 / embassy-sync 0.8.0 / embassy-time 0.5.1. The esp-rtos 0.2.0 pins embassy-executor 0.9.0 / embassy-sync 0.7.x. These are different minor versions but both use the Embassy framework. The protocol crate bridges the two with no Embassy dependency.

---

## File Structure

```
InvertedPendulumRe/
├── inverted-pendulum-protocol/
│   ├── Cargo.toml
│   └── src/
│       └── lib.rs          # SensorData, Command, ControlMode, COBS, CRC8
├── inverted-pendulum-esp32/
│   ├── .cargo/
│   │   └── config.toml     # target = riscv32imc, runner = espflash
│   ├── Cargo.toml
│   ├── build.rs            # linker script
│   ├── rust-toolchain.toml # stable + riscv32imc target
│   └── src/
│       ├── main.rs          # Embassy entry, task spawning, bridge task
│       ├── uart.rs          # UART rx/tx tasks with COBS framing
│       ├── ble.rs           # BLE GATT server (trouble-host)
│       ├── wifi.rs          # WiFi STA/AP + WebSocket server
│       └── led.rs           # LED status indicators
├── inverted-pendulum-stm32/
│   ├── Cargo.toml           # (modify: add protocol dependency)
│   ��── src/
│       ├── main.rs          # (modify: init UART, spawn uart tasks)
│       ├── uart.rs          # (new: UART tx/rx tasks)
│       └── controller/
│           └── mod.rs       # (modify: use ControlMode from protocol)
```

---

### Task 1: Create inverted-pendulum-protocol crate — data types

**Files:**
- Create: `inverted-pendulum-protocol/Cargo.toml`
- Create: `inverted-pendulum-protocol/src/lib.rs`

- [ ] **Step 1: Create Cargo.toml**

```toml
[package]
name = "inverted-pendulum-protocol"
version = "0.1.0"
edition = "2021"

[dependencies]
defmt = { version = "1.0.1", optional = true }

[features]
default = []
defmt = ["dep:defmt"]
```

- [ ] **Step 2: Write the data type definitions in lib.rs**

```rust
#![no_std]

#[cfg(feature = "defmt")]
use defmt::Format;

/// 制御モード
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum ControlMode {
    Debug = 0,
    Pid = 1,
    Lqr = 2,
    Mrac = 3,
    Mpc = 4,
}

impl ControlMode {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => ControlMode::Debug,
            1 => ControlMode::Pid,
            2 => ControlMode::Lqr,
            3 => ControlMode::Mrac,
            _ => ControlMode::Mpc,
        }
    }
}

/// コマンド��イプ
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum CommandType {
    Start = 0x01,
    Stop = 0x02,
    ModeChange = 0x03,
    ParamSet = 0x04,
}

impl CommandType {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x01 => Some(CommandType::Start),
            0x02 => Some(CommandType::Stop),
            0x03 => Some(CommandType::ModeChange),
            0x04 => Some(CommandType::ParamSet),
            _ => None,
        }
    }
}

/// センサデータフレーム (STM32 → ESP32)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct SensorData {
    pub theta: f32,
    pub position: f32,
    pub velocity: f32,
    pub theta_dot: f32,
    pub current_l: f32,
    pub current_r: f32,
    pub control_mode: u8,
    pub running: u8,
}

/// コマンドフレーム (ESP32 → STM32)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct Command {
    pub command_type: CommandType,
    pub payload: [u8; 2],
}
```

- [ ] **Step 3: Verify it compiles**

Run: `cd inverted-pendulum-protocol && cargo build`
Expected: Compiles with no errors.

- [ ] **Step 4: Commit**

```bash
git add inverted-pendulum-protocol/
git commit -m "feat(protocol): add shared protocol crate with data types"
```

---

### Task 2: Protocol crate — serialization, CRC8, COBS

**Files:**
- Modify: `inverted-pendulum-protocol/src/lib.rs`

- [ ] **Step 1: Write tests for CRC8, serialization, and COBS**

Add to the bottom of `lib.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc8_known_value() {
        let data = [0x01, 0x02, 0x03];
        let crc = crc8(&data);
        // CRC8/MAXIM polynomial 0x31
        assert_ne!(crc, 0);
        assert_eq!(crc, crc8(&data)); // deterministic
    }

    #[test]
    fn test_crc8_verify() {
        let data = [0x01, 0x02, 0x03];
        let crc = crc8(&data);
        let mut frame = [0x01, 0x02, 0x03, 0x00];
        frame[3] = crc;
        assert!(verify_crc8(&frame));
        frame[1] = 0xFF;
        assert!(!verify_crc8(&frame));
    }

    #[test]
    fn test_sensor_data_roundtrip() {
        let sd = SensorData {
            theta: 0.1,
            position: 0.2,
            velocity: 0.3,
            theta_dot: 0.4,
            current_l: 0.5,
            current_r: 0.6,
            control_mode: 2,
            running: 1,
        };
        let mut buf = [0u8; SENSOR_DATA_RAW_SIZE];
        sd.serialize(&mut buf);
        let sd2 = SensorData::deserialize(&buf).unwrap();
        assert_eq!(sd.theta.to_bits(), sd2.theta.to_bits());
        assert_eq!(sd.position.to_bits(), sd2.position.to_bits());
        assert_eq!(sd.velocity.to_bits(), sd2.velocity.to_bits());
        assert_eq!(sd.theta_dot.to_bits(), sd2.theta_dot.to_bits());
        assert_eq!(sd.current_l.to_bits(), sd2.current_l.to_bits());
        assert_eq!(sd.current_r.to_bits(), sd2.current_r.to_bits());
        assert_eq!(sd.control_mode, sd2.control_mode);
        assert_eq!(sd.running, sd2.running);
    }

    #[test]
    fn test_command_roundtrip() {
        let cmd = Command {
            command_type: CommandType::ModeChange,
            payload: [2, 0],
        };
        let mut buf = [0u8; COMMAND_RAW_SIZE];
        cmd.serialize(&mut buf);
        let cmd2 = Command::deserialize(&buf).unwrap();
        assert_eq!(cmd.command_type, cmd2.command_type);
        assert_eq!(cmd.payload, cmd2.payload);
    }

    #[test]
    fn test_cobs_roundtrip() {
        let data = [0x01, 0x00, 0x03, 0x00, 0x05];
        let mut encoded = [0u8; 16];
        let enc_len = cobs_encode(&data, &mut encoded);
        // COBSエンコード結果に0x00が含まれないこと
        for &b in &encoded[..enc_len] {
            assert_ne!(b, 0x00);
        }
        let mut decoded = [0u8; 16];
        let dec_len = cobs_decode(&encoded[..enc_len], &mut decoded).unwrap();
        assert_eq!(&data[..], &decoded[..dec_len]);
    }

    #[test]
    fn test_frame_sensor_data_roundtrip() {
        let sd = SensorData {
            theta: -0.05,
            position: 0.1,
            velocity: -0.2,
            theta_dot: 0.3,
            current_l: 1.5,
            current_r: 1.6,
            control_mode: 4,
            running: 0,
        };
        let mut frame_buf = [0u8; MAX_SENSOR_FRAME_SIZE];
        let frame_len = sd.to_frame(&mut frame_buf);
        let sd2 = SensorData::from_frame(&frame_buf[..frame_len]).unwrap();
        assert_eq!(sd.theta.to_bits(), sd2.theta.to_bits());
        assert_eq!(sd.control_mode, sd2.control_mode);
    }

    #[test]
    fn test_frame_command_roundtrip() {
        let cmd = Command {
            command_type: CommandType::Start,
            payload: [0, 0],
        };
        let mut frame_buf = [0u8; MAX_COMMAND_FRAME_SIZE];
        let frame_len = cmd.to_frame(&mut frame_buf);
        let cmd2 = Command::from_frame(&frame_buf[..frame_len]).unwrap();
        assert_eq!(cmd.command_type, cmd2.command_type);
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd inverted-pendulum-protocol && cargo test`
Expected: FAIL — functions `crc8`, `serialize`, `cobs_encode`, etc. not defined.

- [ ] **Step 3: Implement CRC8, serialization, COBS, and frame helpers**

Add above the `#[cfg(test)]` module in `lib.rs`:

```rust
/// プロトコルエラー
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum ProtocolError {
    BufferTooSmall,
    InvalidLength,
    InvalidCommandType,
    CobsDecodeError,
    CrcMismatch,
}

// --- サイズ定数 ---

/// SensorData生バイトサイズ: f32×6 + u8×2 = 26
pub const SENSOR_DATA_RAW_SIZE: usize = 6 * 4 + 2;
/// SensorData + CRC8 = 27
pub const SENSOR_DATA_WITH_CRC_SIZE: usize = SENSOR_DATA_RAW_SIZE + 1;
/// COBSエンコード後最大サイズ
pub const MAX_SENSOR_COBS_SIZE: usize = SENSOR_DATA_WITH_CRC_SIZE + (SENSOR_DATA_WITH_CRC_SIZE / 254) + 1;
/// フレーム最大サイズ (COBS + 0x00デリミタ)
pub const MAX_SENSOR_FRAME_SIZE: usize = MAX_SENSOR_COBS_SIZE + 1;

/// Command生バイトサイズ: u8 + u8×2 = 3
pub const COMMAND_RAW_SIZE: usize = 3;
/// Command + CRC8 = 4
pub const COMMAND_WITH_CRC_SIZE: usize = COMMAND_RAW_SIZE + 1;
/// COBSエンコード後最大サイズ
pub const MAX_COMMAND_COBS_SIZE: usize = COMMAND_WITH_CRC_SIZE + (COMMAND_WITH_CRC_SIZE / 254) + 1;
/// フレーム最大サイズ
pub const MAX_COMMAND_FRAME_SIZE: usize = MAX_COMMAND_COBS_SIZE + 1;

// --- CRC8 (polynomial 0x31, init 0x00) ---

pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

pub fn verify_crc8(data_with_crc: &[u8]) -> bool {
    if data_with_crc.len() < 2 {
        return false;
    }
    let (data, crc_byte) = data_with_crc.split_at(data_with_crc.len() - 1);
    crc8(data) == crc_byte[0]
}

// --- COBS エンコード/デコード ---

pub fn cobs_encode(data: &[u8], out: &mut [u8]) -> usize {
    let mut read_idx = 0;
    let mut write_idx = 1;
    let mut code_idx = 0;
    let mut code: u8 = 1;

    while read_idx < data.len() {
        if data[read_idx] == 0x00 {
            out[code_idx] = code;
            code_idx = write_idx;
            write_idx += 1;
            code = 1;
        } else {
            out[write_idx] = data[read_idx];
            write_idx += 1;
            code += 1;
            if code == 0xFF {
                out[code_idx] = code;
                code_idx = write_idx;
                write_idx += 1;
                code = 1;
            }
        }
        read_idx += 1;
    }
    out[code_idx] = code;
    write_idx
}

pub fn cobs_decode(data: &[u8], out: &mut [u8]) -> Result<usize, ProtocolError> {
    let mut read_idx = 0;
    let mut write_idx = 0;

    while read_idx < data.len() {
        let code = data[read_idx];
        if code == 0 {
            return Err(ProtocolError::CobsDecodeError);
        }
        read_idx += 1;
        for _ in 1..code {
            if read_idx >= data.len() {
                return Err(ProtocolError::CobsDecodeError);
            }
            out[write_idx] = data[read_idx];
            write_idx += 1;
            read_idx += 1;
        }
        if code < 0xFF && read_idx < data.len() {
            out[write_idx] = 0x00;
            write_idx += 1;
        }
    }
    // 末尾の余分な0x00を除去
    if write_idx > 0 && out[write_idx - 1] == 0x00 {
        write_idx -= 1;
    }
    Ok(write_idx)
}

// --- SensorData シリアライズ ---

impl SensorData {
    pub fn serialize(&self, buf: &mut [u8]) {
        let mut i = 0;
        for &val in &[
            self.theta,
            self.position,
            self.velocity,
            self.theta_dot,
            self.current_l,
            self.current_r,
        ] {
            buf[i..i + 4].copy_from_slice(&val.to_le_bytes());
            i += 4;
        }
        buf[i] = self.control_mode;
        buf[i + 1] = self.running;
    }

    pub fn deserialize(buf: &[u8]) -> Result<Self, ProtocolError> {
        if buf.len() < SENSOR_DATA_RAW_SIZE {
            return Err(ProtocolError::InvalidLength);
        }
        let f = |offset: usize| -> f32 {
            f32::from_le_bytes([buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]])
        };
        Ok(SensorData {
            theta: f(0),
            position: f(4),
            velocity: f(8),
            theta_dot: f(12),
            current_l: f(16),
            current_r: f(20),
            control_mode: buf[24],
            running: buf[25],
        })
    }

    /// serialize + CRC8 + COBSエンコード + 0x00デリミタ → フレーム
    pub fn to_frame(&self, buf: &mut [u8]) -> usize {
        let mut raw = [0u8; SENSOR_DATA_WITH_CRC_SIZE];
        self.serialize(&mut raw[..SENSOR_DATA_RAW_SIZE]);
        raw[SENSOR_DATA_RAW_SIZE] = crc8(&raw[..SENSOR_DATA_RAW_SIZE]);
        let cobs_len = cobs_encode(&raw, buf);
        buf[cobs_len] = 0x00; // デリミタ
        cobs_len + 1
    }

    /// 0x00デリミタなしのCOBSフレームからデコ���ド
    pub fn from_frame(frame: &[u8]) -> Result<Self, ProtocolError> {
        // デリミタ除去
        let cobs_data = if frame.last() == Some(&0x00) {
            &frame[..frame.len() - 1]
        } else {
            frame
        };
        let mut decoded = [0u8; SENSOR_DATA_WITH_CRC_SIZE];
        let len = cobs_decode(cobs_data, &mut decoded)?;
        if len != SENSOR_DATA_WITH_CRC_SIZE {
            return Err(ProtocolError::InvalidLength);
        }
        if !verify_crc8(&decoded[..len]) {
            return Err(ProtocolError::CrcMismatch);
        }
        Self::deserialize(&decoded[..SENSOR_DATA_RAW_SIZE])
    }
}

// --- Command シリアライズ ---

impl Command {
    pub fn serialize(&self, buf: &mut [u8]) {
        buf[0] = self.command_type as u8;
        buf[1] = self.payload[0];
        buf[2] = self.payload[1];
    }

    pub fn deserialize(buf: &[u8]) -> Result<Self, ProtocolError> {
        if buf.len() < COMMAND_RAW_SIZE {
            return Err(ProtocolError::InvalidLength);
        }
        let command_type =
            CommandType::from_u8(buf[0]).ok_or(ProtocolError::InvalidCommandType)?;
        Ok(Command {
            command_type,
            payload: [buf[1], buf[2]],
        })
    }

    pub fn to_frame(&self, buf: &mut [u8]) -> usize {
        let mut raw = [0u8; COMMAND_WITH_CRC_SIZE];
        self.serialize(&mut raw[..COMMAND_RAW_SIZE]);
        raw[COMMAND_RAW_SIZE] = crc8(&raw[..COMMAND_RAW_SIZE]);
        let cobs_len = cobs_encode(&raw, buf);
        buf[cobs_len] = 0x00;
        cobs_len + 1
    }

    pub fn from_frame(frame: &[u8]) -> Result<Self, ProtocolError> {
        let cobs_data = if frame.last() == Some(&0x00) {
            &frame[..frame.len() - 1]
        } else {
            frame
        };
        let mut decoded = [0u8; COMMAND_WITH_CRC_SIZE];
        let len = cobs_decode(cobs_data, &mut decoded)?;
        if len != COMMAND_WITH_CRC_SIZE {
            return Err(ProtocolError::InvalidLength);
        }
        if !verify_crc8(&decoded[..len]) {
            return Err(ProtocolError::CrcMismatch);
        }
        Self::deserialize(&decoded[..COMMAND_RAW_SIZE])
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd inverted-pendulum-protocol && cargo test`
Expected: All 7 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add inverted-pendulum-protocol/
git commit -m "feat(protocol): implement serialization, CRC8, and COBS framing"
```

---

### Task 3: Scaffold inverted-pendulum-esp32 crate

**Files:**
- Create: `inverted-pendulum-esp32/Cargo.toml`
- Create: `inverted-pendulum-esp32/build.rs`
- Create: `inverted-pendulum-esp32/rust-toolchain.toml`
- Create: `inverted-pendulum-esp32/.cargo/config.toml`
- Create: `inverted-pendulum-esp32/src/main.rs`

- [ ] **Step 1: Create rust-toolchain.toml**

```toml
[toolchain]
channel = "stable"
targets = ["riscv32imc-unknown-none-elf"]
```

- [ ] **Step 2: Create .cargo/config.toml**

```toml
[build]
target = "riscv32imc-unknown-none-elf"
rustflags = ["-C", "force-frame-pointers"]

[target.riscv32imc-unknown-none-elf]
runner = "espflash flash --monitor"

[env]
ESP_LOG = "INFO"
```

- [ ] **Step 3: Create build.rs**

```rust
fn main() {
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}
```

- [ ] **Step 4: Create Cargo.toml**

```toml
[package]
name = "inverted-pendulum-esp32"
version = "0.1.0"
edition = "2021"

[dependencies]
esp-hal = { version = "1.0.0", features = ["esp32c3"] }
esp-rtos = { version = "0.2.0", features = [
    "esp32c3",
    "embassy",
    "esp-alloc",
] }
esp-backtrace = { version = "0.18.1", features = [
    "esp32c3",
    "exception-handler",
    "panic-handler",
    "println",
] }
esp-println = { version = "0.13.0", features = ["esp32c3", "log"] }
embassy-sync = "0.7"
embassy-futures = "0.1"
embassy-time = "0.4"
embedded-io-async = "0.6"
log = "0.4"
inverted-pendulum-protocol = { path = "../inverted-pendulum-protocol" }

[[bin]]
name = "inverted-pendulum-esp32"
test = false
bench = false

[profile.dev]
opt-level = 1

[profile.release]
codegen-units = 1
opt-level = 3
lto = "fat"
overflow-checks = false
```

- [ ] **Step 5: Create minimal main.rs**

```rust
#![no_std]
#![no_main]

use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Level, Output, OutputConfig};
use log::info;

extern crate esp_backtrace as _;
extern crate esp_println as _;

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    info!("inverted-pendulum-esp32 started");

    // LED_1 (GPIO3) — 動作確認用に点滅
    let mut led1 = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());

    loop {
        led1.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
```

- [ ] **Step 6: Verify it compiles**

Run: `cd inverted-pendulum-esp32 && cargo build`
Expected: Compiles successfully. (Do not flash — board may not be connected.)

Note: If dependency version conflicts occur (esp-rtos pinning different embassy-time/embassy-sync versions), adjust versions in Cargo.toml to match what esp-rtos requires. Check `cargo tree` output.

- [ ] **Step 7: Commit**

```bash
git add inverted-pendulum-esp32/
git commit -m "feat(esp32): scaffold ESP32-C3 Embassy project with LED blink"
```

---

### Task 4: Implement UART communication (ESP32 side)

**Files:**
- Create: `inverted-pendulum-esp32/src/uart.rs`
- Modify: `inverted-pendulum-esp32/src/main.rs`

- [ ] **Step 1: Create uart.rs with rx and tx tasks**

```rust
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embedded_io_async::{Read, Write};
use esp_hal::uart::Uart;
use esp_hal::Async;
use inverted_pendulum_protocol::{
    Command, SensorData, MAX_SENSOR_FRAME_SIZE, SENSOR_DATA_WITH_CRC_SIZE,
};
use log::{info, warn};

/// 最新センサデータ (uart_rx → bridge)
pub static SENSOR_SIGNAL: Signal<CriticalSectionRawMutex, SensorData> = Signal::new();

/// コマンドキュー (ble/wifi → uart_tx)
pub static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, Command, 4> = Channel::new();

/// UART受信タスク: COBSフレームを読み取り、SensorDataをデコード
#[embassy_executor::task]
pub async fn uart_rx_task(mut rx: esp_hal::uart::UartRx<'static, Async>) {
    info!("uart_rx_task started");
    let mut buf = [0u8; 1];
    let mut frame_buf = [0u8; MAX_SENSOR_FRAME_SIZE];
    let mut frame_pos: usize = 0;

    loop {
        match rx.read(&mut buf).await {
            Ok(_) => {
                let byte = buf[0];
                if byte == 0x00 {
                    // フレーム終端
                    if frame_pos > 0 {
                        match SensorData::from_frame(&frame_buf[..frame_pos]) {
                            Ok(data) => {
                                SENSOR_SIGNAL.signal(data);
                            }
                            Err(_e) => {
                                warn!("sensor frame decode failed");
                            }
                        }
                        frame_pos = 0;
                    }
                } else if frame_pos < frame_buf.len() {
                    frame_buf[frame_pos] = byte;
                    frame_pos += 1;
                } else {
                    // オーバーフロー: フレーム破棄
                    frame_pos = 0;
                }
            }
            Err(_e) => {
                warn!("uart rx error");
            }
        }
    }
}

/// UART送信タス��: コマンドをCOBSフレームとしてシリアル送信
#[embassy_executor::task]
pub async fn uart_tx_task(mut tx: esp_hal::uart::UartTx<'static, Async>) {
    info!("uart_tx_task started");
    let mut frame_buf = [0u8; 8]; // MAX_COMMAND_FRAME_SIZE は6以下

    loop {
        let cmd = COMMAND_CHANNEL.receive().await;
        let frame_len = cmd.to_frame(&mut frame_buf);
        if let Err(_e) = tx.write_all(&frame_buf[..frame_len]).await {
            warn!("uart tx error");
        }
    }
}
```

- [ ] **Step 2: Update main.rs to initialize UART and spawn tasks**

Replace `main.rs` content:

```rust
#![no_std]
#![no_main]

mod ble;
mod led;
mod uart;
mod wifi;

use embassy_time::{Duration, Ticker};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::uart::{Config as UartConfig, Uart};
use log::info;

extern crate esp_backtrace as _;
extern crate esp_println as _;

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    info!("inverted-pendulum-esp32 started");

    // UART: GPIO20 (RX) / GPIO21 (TX) — STM32と通信
    let uart_config = UartConfig::default().with_baudrate(115200);
    let uart = Uart::new(peripherals.UART0, uart_config)
        .unwrap()
        .with_rx(peripherals.GPIO20)
        .with_tx(peripherals.GPIO21)
        .into_async();
    let (uart_rx, uart_tx) = uart.split();

    // LED
    let led1 = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    let led2 = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());

    // タスク起動
    spawner.must_spawn(uart::uart_rx_task(uart_rx));
    spawner.must_spawn(uart::uart_tx_task(uart_tx));
    spawner.must_spawn(led::led_task(led1, led2));

    // bridge loop: センサデータの配信は BLE/WiFi 実装後に追加
    let mut ticker = Ticker::every(Duration::from_millis(20));
    loop {
        ticker.next().await;
    }
}
```

- [ ] **Step 3: Create stub modules for ble, wifi, led**

Create `inverted-pendulum-esp32/src/ble.rs`:
```rust
// BLE implementation — Task 6
```

Create `inverted-pendulum-esp32/src/wifi.rs`:
```rust
// WiFi implementation — Task 7
```

Create `inverted-pendulum-esp32/src/led.rs`:
```rust
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Output;
use log::info;

/// LED状態表示タスク
#[embassy_executor::task]
pub async fn led_task(mut led1: Output<'static>, mut led2: Output<'static>) {
    info!("led_task started");
    // 初期状態: 両方点滅（未接続表示）
    loop {
        led1.toggle();
        led2.toggle();
        Timer::after(Duration::from_millis(200)).await;
    }
}
```

- [ ] **Step 4: Verify it compiles**

Run: `cd inverted-pendulum-esp32 && cargo build`
Expected: Compiles successfully.

- [ ] **Step 5: Commit**

```bash
git add inverted-pendulum-esp32/src/
git commit -m "feat(esp32): implement UART rx/tx tasks with COBS framing"
```

---

### Task 5: Implement LED status indicators

**Files:**
- Modify: `inverted-pendulum-esp32/src/led.rs`

- [ ] **Step 1: Implement full LED state machine**

Replace `led.rs`:

```rust
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Output;
use log::info;

/// BLE接続状態
#[derive(Clone, Copy, PartialEq)]
pub enum BleStatus {
    Disconnected,
    Advertising,
    Connected,
}

/// WiFi接続状態
#[derive(Clone, Copy, PartialEq)]
pub enum WifiStatus {
    Disconnected,
    Connecting,
    StaConnected,
    ApMode,
}

pub static BLE_STATUS: Signal<CriticalSectionRawMutex, BleStatus> = Signal::new();
pub static WIFI_STATUS: Signal<CriticalSectionRawMutex, WifiStatus> = Signal::new();

#[embassy_executor::task]
pub async fn led_task(mut led1: Output<'static>, mut led2: Output<'static>) {
    info!("led_task started");
    let mut ble_status = BleStatus::Disconnected;
    let mut wifi_status = WifiStatus::Disconnected;
    let mut tick: u32 = 0;

    loop {
        // 非ブロッキングでステータス更新を確認
        if BLE_STATUS.signaled() {
            ble_status = BLE_STATUS.wait().await;
        }
        if WIFI_STATUS.signaled() {
            wifi_status = WIFI_STATUS.wait().await;
        }

        // LED_1: BLE状態
        match ble_status {
            BleStatus::Disconnected => led1.set_low(),
            BleStatus::Connected => led1.set_high(),
            BleStatus::Advertising => {
                // 200ms点滅
                if tick % 4 < 2 {
                    led1.set_high();
                } else {
                    led1.set_low();
                }
            }
        }

        // LED_2: WiFi状態
        match wifi_status {
            WifiStatus::Disconnected => led2.set_low(),
            WifiStatus::StaConnected => led2.set_high(),
            WifiStatus::Connecting => {
                // 200ms���滅
                if tick % 4 < 2 {
                    led2.set_high();
                } else {
                    led2.set_low();
                }
            }
            WifiStatus::ApMode => {
                // 100ms高速点滅
                if tick % 2 == 0 {
                    led2.set_high();
                } else {
                    led2.set_low();
                }
            }
        }

        tick = tick.wrapping_add(1);
        Timer::after(Duration::from_millis(50)).await;
    }
}
```

- [ ] **Step 2: Verify it compiles**

Run: `cd inverted-pendulum-esp32 && cargo build`
Expected: Compiles successfully.

- [ ] **Step 3: Commit**

```bash
git add inverted-pendulum-esp32/src/led.rs
git commit -m "feat(esp32): implement LED status indicators for BLE/WiFi"
```

---

### Task 6: Implement BLE GATT server

**Files:**
- Modify: `inverted-pendulum-esp32/src/ble.rs`
- Modify: `inverted-pendulum-esp32/src/main.rs`
- Modify: `inverted-pendulum-esp32/Cargo.toml`

- [ ] **Step 1: Add BLE dependencies to Cargo.toml**

Add to `[dependencies]`:
```toml
esp-radio = { version = "0.17.0", features = ["esp32c3", "ble", "wifi", "coex"] }
trouble-host = "0.6.0"
bt-hci = "0.2"
```

Also add `"esp-radio"` to esp-rtos features:
```toml
esp-rtos = { version = "0.2.0", features = [
    "esp32c3",
    "embassy",
    "esp-alloc",
    "esp-radio",
] }
```

- [ ] **Step 2: Implement BLE GATT server in ble.rs**

```rust
use bt_hci::controller::ExternalController;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_radio::ble::connector::BleConnector;
use inverted_pendulum_protocol::{Command, CommandType, SensorData, SENSOR_DATA_RAW_SIZE};
use log::{info, warn};
use trouble_host::prelude::*;

use crate::led::{BleStatus, BLE_STATUS};
use crate::uart::COMMAND_CHANNEL;

/// BLEからのセンサデータ配信用チャンネル
pub static BLE_SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorData, 2> = Channel::new();

// GATT UUIDs (リファレンスプロジェクトから流用)
const SERVICE_UUID: Uuid = Uuid::new_long([
    0x5e, 0x07, 0x71, 0xca, 0x5b, 0x75, 0x05, 0xb8, 0x48, 0x40, 0xa5, 0x91, 0x6f, 0xca, 0x94,
    0xff,
]);
const TX_CHAR_UUID: Uuid = Uuid::new_long([
    0x48, 0xd8, 0xbd, 0x3b, 0x0e, 0xd9, 0xc1, 0x8e, 0x31, 0x44, 0x3c, 0xf4, 0xe9, 0x2b, 0x32,
    0xb2,
]);
const RX_CHAR_UUID: Uuid = Uuid::new_long([
    0x50, 0xd9, 0x39, 0x25, 0xf5, 0x0d, 0xef, 0xb0, 0x43, 0x46, 0x66, 0x5d, 0x88, 0x85, 0x30,
    0xa3,
]);

/// GATTサーバー定義
#[gatt_server]
struct InvertedPendulumServer {
    uart_service: UartService,
}

/// UART BLEサービス
#[gatt_service(uuid = service_uuid())]
struct UartService {
    /// センサデータ通知 (Notify)
    #[descriptor(uuid = "2901", value = "Sensor Data")]
    #[characteristic(uuid = tx_char_uuid(), notify)]
    tx: [u8; SENSOR_DATA_RAW_SIZE],

    /// コマンド受信 (Write)
    #[descriptor(uuid = "2901", value = "Command")]
    #[characteristic(uuid = rx_char_uuid(), write)]
    rx: [u8; 3],
}

const fn service_uuid() -> &'static str {
    "ff94ca6f-91a5-4048-b805-755bca71075e"
}
const fn tx_char_uuid() -> &'static str {
    "b2322be9-f43c-4431-8ec1-d90e3b8bd848"
}
const fn rx_char_uuid() -> &'static str {
    "a3308588-5d66-4643-b0ef-0df52539d950"
}

#[embassy_executor::task]
pub async fn ble_task(ble_connector: BleConnector<'static>) {
    info!("ble_task started");

    let controller: ExternalController<_, 20> =
        ExternalController::new(ble_connector);

    // trouble-hostスタック
    let mut resources: HostResources<2, 4, 27> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    // GATTサーバー
    let server = InvertedPendulumServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "InvertedPendulum",
        appearance: &appearance::power_device::GENERIC,
    }))
    .unwrap();

    // BLEコントローラーをバックグラウンド実行
    let ble_runner = async { runner.run().await };

    // メインBLEループ
    let ble_main = async {
        loop {
            BLE_STATUS.signal(BleStatus::Advertising);
            info!("BLE advertising...");

            match peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data(&server),
                        scan_data: &[],
                    },
                )
                .await
            {
                Ok(conn) => {
                    BLE_STATUS.signal(BleStatus::Connected);
                    info!("BLE connected");

                    // 接続中のGATTイベント処理 + notify配信
                    let gatt_events = gatt_events_task(&server, &conn);
                    let notify_task = ble_notify_task(&server, &conn);
                    embassy_futures::select::select(gatt_events, notify_task).await;

                    info!("BLE disconnected");
                    BLE_STATUS.signal(BleStatus::Disconnected);
                }
                Err(e) => {
                    warn!("BLE advertise error: {:?}", e);
                }
            }
        }
    };

    embassy_futures::select::select(ble_runner, ble_main).await;
}

fn adv_data(server: &InvertedPendulumServer) -> Vec<u8, 31> {
    let mut data = Vec::new();
    // Flags
    data.extend_from_slice(&[2, 0x01, 0x06]).ok();
    // Complete local name
    let name = b"InvPend";
    data.push((name.len() + 1) as u8).ok();
    data.push(0x09).ok();
    data.extend_from_slice(name).ok();
    data
}

async fn gatt_events_task(server: &InvertedPendulumServer, conn: &Connection<'_>) {
    loop {
        match server.next().await {
            Ok(GattEvent::Write(event)) => {
                // RX characteristic書き込み → コマンドとして処理
                let handle = event.handle();
                if handle == server.uart_service.rx.handle {
                    let data = event.data();
                    if data.len() >= 3 {
                        if let Some(cmd_type) = CommandType::from_u8(data[0]) {
                            let cmd = Command {
                                command_type: cmd_type,
                                payload: [data[1], data[2]],
                            };
                            COMMAND_CHANNEL.try_send(cmd).ok();
                            info!("BLE command received: {:?}", data[0]);
                        }
                    }
                }
            }
            Ok(_) => {}
            Err(_) => break,
        }
    }
}

async fn ble_notify_task(server: &InvertedPendulumServer, conn: &Connection<'_>) {
    loop {
        let data = BLE_SENSOR_CHANNEL.receive().await;
        let mut buf = [0u8; SENSOR_DATA_RAW_SIZE];
        data.serialize(&mut buf);
        if let Err(_e) = server.uart_service.tx.notify(conn, &buf).await {
            break; // 切断
        }
    }
}
```

Note: The exact trouble-host API may differ between versions. The `#[gatt_server]` and `#[gatt_service]` macros are provided by trouble-host's proc macros. If the API surface differs, adjust the GATT server definition to match the actual trouble-host 0.6.0 API. Refer to the trouble-host examples at `https://github.com/embassy-rs/trouble/tree/main/examples`.

- [ ] **Step 3: Update main.rs to initialize BLE and spawn task**

Add BLE initialization after UART setup in main.rs, before task spawning:

```rust
    // Radio初期化 (WiFi + BLE coex)
    let radio_init = esp_radio::init(peripherals.RADIO_CLK, peripherals.TIMG0).unwrap();
    let ble_connector = BleConnector::new(&radio_init, peripherals.BT, Default::default());

    // タスク起動
    spawner.must_spawn(uart::uart_rx_task(uart_rx));
    spawner.must_spawn(uart::uart_tx_task(uart_tx));
    spawner.must_spawn(led::led_task(led1, led2));
    spawner.must_spawn(ble::ble_task(ble_connector));
```

- [ ] **Step 4: Verify it compiles**

Run: `cd inverted-pendulum-esp32 && cargo build`
Expected: Compiles. Fix any trouble-host API differences.

- [ ] **Step 5: Commit**

```bash
git add inverted-pendulum-esp32/
git commit -m "feat(esp32): implement BLE GATT server with sensor notify and command write"
```

---

### Task 7: Implement WiFi + WebSocket server

**Files:**
- Modify: `inverted-pendulum-esp32/src/wifi.rs`
- Modify: `inverted-pendulum-esp32/src/main.rs`
- Modify: `inverted-pendulum-esp32/Cargo.toml`

- [ ] **Step 1: Add WiFi dependencies to Cargo.toml**

Add to `[dependencies]`:
```toml
heapless = "0.8"
```

- [ ] **Step 2: Implement WiFi connection and WebSocket server in wifi.rs**

```rust
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use esp_radio::wifi::{
    ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
    WifiState,
};
use inverted_pendulum_protocol::{Command, CommandType, ControlMode, SensorData};
use log::{info, warn};

use crate::led::{WifiStatus, WIFI_STATUS};
use crate::uart::COMMAND_CHANNEL;

/// WiFiへのセンサデータ配信用チャンネル
pub static WIFI_SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorData, 2> = Channel::new();

/// WiFi SSID/パスワード (コンパイル時定数)
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

/// WiFi接続管理タスク
#[embassy_executor::task]
pub async fn wifi_task(
    mut controller: WifiController<'static>,
    sta_device: WifiStaDevice<'static>,
) {
    info!("wifi_task started");
    let mut retry_count: u8 = 0;
    const MAX_RETRIES: u8 = 3;

    loop {
        WIFI_STATUS.signal(WifiStatus::Connecting);
        info!("WiFi connecting to {}...", WIFI_SSID);

        // STA設定
        let config = Configuration::Client(ClientConfiguration {
            ssid: WIFI_SSID.try_into().unwrap(),
            password: WIFI_PASSWORD.try_into().unwrap(),
            ..Default::default()
        });
        controller.set_configuration(&config).unwrap();
        controller.start().await.unwrap();

        match controller.connect().await {
            Ok(()) => {
                info!("WiFi connected");
                WIFI_STATUS.signal(WifiStatus::StaConnected);
                retry_count = 0;

                // DHCP待ち + WebSocketサーバー起動
                // TODO: embassy-netスタック統合後にWebSocketサーバーを実装
                // 現時点ではWiFi接続維持のみ

                // 切断待ち
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                warn!("WiFi disconnected");
                WIFI_STATUS.signal(WifiStatus::Disconnected);
            }
            Err(e) => {
                warn!("WiFi connect failed: {:?}", e);
                retry_count += 1;
                if retry_count >= MAX_RETRIES {
                    info!("WiFi: switching to AP mode");
                    WIFI_STATUS.signal(WifiStatus::ApMode);
                    // AP mode実装は将来対応
                    Timer::after(Duration::from_secs(30)).await;
                    retry_count = 0;
                } else {
                    Timer::after(Duration::from_secs(5)).await;
                }
            }
        }
    }
}
```

Note: Full WebSocket server requires `embassy-net` TCP stack integration, which adds significant complexity (DHCP client, TCP socket management, HTTP upgrade, WebSocket frame parsing). The WiFi connection management is implemented first. WebSocket server implementation should be added as a follow-up once the basic WiFi connectivity is verified on hardware. Mark this with a clear `// TODO` in the code.

- [ ] **Step 3: Update main.rs to initialize WiFi and spawn task**

Add WiFi initialization in main.rs after BLE connector setup:

```rust
    // WiFi
    let (wifi_device, wifi_controller) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default()).unwrap();
    let sta_device = wifi_device.sta();

    spawner.must_spawn(wifi::wifi_task(wifi_controller, sta_device));
```

- [ ] **Step 4: Verify it compiles**

Run: `WIFI_SSID=test WIFI_PASSWORD=test cargo build`
Expected: Compiles. The exact esp-radio WiFi API may need adjustments based on the actual 0.17.0 API.

- [ ] **Step 5: Commit**

```bash
git add inverted-pendulum-esp32/
git commit -m "feat(esp32): implement WiFi STA connection management"
```

---

### Task 8: Implement bridge task

**Files:**
- Modify: `inverted-pendulum-esp32/src/main.rs`

- [ ] **Step 1: Implement bridge loop in main.rs**

Replace the placeholder bridge loop at the end of `main()`:

```rust
    // Bridge loop: UARTセンサデータ → BLE/WiFi配信
    loop {
        // 最新のセンサデータを待つ
        let data = uart::SENSOR_SIGNAL.wait().await;

        // BLE配信 (チャンネルが一杯なら最新を優先)
        ble::BLE_SENSOR_CHANNEL.try_send(data).ok();

        // WiFi配信
        wifi::WIFI_SENSOR_CHANNEL.try_send(data).ok();
    }
```

Remove the Ticker import if no longer needed.

- [ ] **Step 2: Verify it compiles**

Run: `WIFI_SSID=test WIFI_PASSWORD=test cargo build`
Expected: Compiles.

- [ ] **Step 3: Commit**

```bash
git add inverted-pendulum-esp32/src/main.rs
git commit -m "feat(esp32): implement bridge task for sensor data distribution"
```

---

### Task 9: STM32 side — add protocol dependency and UART tasks

**Files:**
- Modify: `inverted-pendulum-stm32/Cargo.toml`
- Modify: `inverted-pendulum-stm32/src/controller/mod.rs`
- Create: `inverted-pendulum-stm32/src/uart.rs`
- Modify: `inverted-pendulum-stm32/src/main.rs`

- [ ] **Step 1: Add protocol dependency to STM32 Cargo.toml**

Add to `[dependencies]`:
```toml
inverted-pendulum-protocol = { path = "../inverted-pendulum-protocol", features = ["defmt"] }
```

- [ ] **Step 2: Replace ControlMode in controller/mod.rs with re-export from protocol**

In `inverted-pendulum-stm32/src/controller/mod.rs`, replace:

```rust
/// 制御モード
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum ControlMode {
    Debug = 0,
    Pid = 1,
    Lqr = 2,
    Mrac = 3,
    Mpc = 4,
}

impl ControlMode {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => ControlMode::Debug,
            1 => ControlMode::Pid,
            2 => ControlMode::Lqr,
            3 => ControlMode::Mrac,
            _ => ControlMode::Mpc,
        }
    }
}
```

With:

```rust
pub use inverted_pendulum_protocol::ControlMode;
```

- [ ] **Step 3: Verify STM32 still compiles with the re-export**

Run: `cd inverted-pendulum-stm32 && cargo build`
Expected: Compiles. All existing code that uses `ControlMode` should work unchanged since the API is identical.

Note: If the existing STM32 code uses `#[cfg(feature = "defmt")]` conditional derives on ControlMode, verify that the protocol crate's `defmt` feature provides the same derives. The protocol crate's ControlMode has `#[cfg_attr(feature = "defmt", derive(Format))]`, matching the STM32's needs.

- [ ] **Step 4: Commit the ControlMode refactor**

```bash
git add inverted-pendulum-stm32/Cargo.toml inverted-pendulum-stm32/src/controller/mod.rs
git commit -m "refactor(stm32): use ControlMode from shared protocol crate"
```

- [ ] **Step 5: Create uart.rs for STM32 UART communication**

Create `inverted-pendulum-stm32/src/uart.rs`:

```rust
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_stm32::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_io_async::{Read, Write};
use inverted_pendulum_protocol::{
    Command, SensorData, MAX_COMMAND_FRAME_SIZE, MAX_SENSOR_FRAME_SIZE,
};

use crate::defmt_println;

/// 送信用センサデータ (control_task → uart_tx)
pub static TX_SENSOR_SIGNAL: Signal<CriticalSectionRawMutex, SensorData> = Signal::new();

/// 受信コマンド (uart_rx → main)
pub static RX_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// UART送信タスク: SensorDataをCOBSフレームで送信
#[embassy_executor::task]
pub async fn uart_tx_task(mut tx: UartTx<'static, Async>) {
    defmt_println!("uart_tx_task started");
    let mut frame_buf = [0u8; MAX_SENSOR_FRAME_SIZE];

    loop {
        let data = TX_SENSOR_SIGNAL.wait().await;
        let frame_len = data.to_frame(&mut frame_buf);
        if let Err(_e) = tx.write_all(&frame_buf[..frame_len]).await {
            defmt_println!("uart tx error");
        }
    }
}

/// UART受信タスク: COBSフレームからCommandをデコード
#[embassy_executor::task]
pub async fn uart_rx_task(mut rx: UartRx<'static, Async>) {
    defmt_println!("uart_rx_task started");
    let mut buf = [0u8; 1];
    let mut frame_buf = [0u8; MAX_COMMAND_FRAME_SIZE];
    let mut frame_pos: usize = 0;

    loop {
        match rx.read(&mut buf).await {
            Ok(_) => {
                let byte = buf[0];
                if byte == 0x00 {
                    if frame_pos > 0 {
                        match Command::from_frame(&frame_buf[..frame_pos]) {
                            Ok(cmd) => {
                                RX_COMMAND_SIGNAL.signal(cmd);
                            }
                            Err(_) => {
                                defmt_println!("command frame decode failed");
                            }
                        }
                        frame_pos = 0;
                    }
                } else if frame_pos < frame_buf.len() {
                    frame_buf[frame_pos] = byte;
                    frame_pos += 1;
                } else {
                    frame_pos = 0;
                }
            }
            Err(_) => {
                defmt_println!("uart rx error");
            }
        }
    }
}
```

- [ ] **Step 6: Update STM32 main.rs — add UART initialization and task spawning**

In `inverted-pendulum-stm32/src/main.rs`:

Add `mod uart;` with the other module declarations.

Add UART peripheral initialization in `main()` after existing peripheral setup, before task spawning. Use USART2 with PA2 (TX) / PA3 (RX):

```rust
    // UART: PA2 (TX) / PA3 (RX) — ESP32と通信
    let mut uart_config = embassy_stm32::usart::Config::default();
    uart_config.baudrate = 115200;
    let uart = embassy_stm32::usart::Uart::new(
        p.USART2, p.PA3, p.PA2, Irqs, p.DMA1_CH6, p.DMA1_CH7, uart_config,
    )
    .unwrap()
    .into_async();
    let (uart_tx, uart_rx) = uart.split();
```

Add task spawning:
```rust
    spawner.must_spawn(uart::uart_tx_task(uart_tx));
    spawner.must_spawn(uart::uart_rx_task(uart_rx));
```

Add interrupt binding for USART2 in the `bind_interrupts!` macro:
```rust
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
```

- [ ] **Step 7: Add sensor data transmission in control_task**

In the `control_task` function in `main.rs`, add a decimation counter and signal the sensor data to UART at 10Hz. Inside the balance control block (the `if balance_tick >= BALANCE_DECIMATION` section), add:

```rust
    // UART送信用decimation (10Hz = 100回に1回)
    static mut UART_DECIMATION: u32 = 0;
    unsafe {
        UART_DECIMATION += 1;
        if UART_DECIMATION >= 100 {
            UART_DECIMATION = 0;
            let sensor_data = inverted_pendulum_protocol::SensorData {
                theta: processed.theta,
                position: processed.position,
                velocity: processed.velocity,
                theta_dot: processed.theta_dot,
                current_l: state.current_l,
                current_r: state.current_r,
                control_mode: CONTROL_MODE.load(core::sync::atomic::Ordering::Relaxed),
                running: if RUNNING.load(core::sync::atomic::Ordering::Relaxed) { 1 } else { 0 },
            };
            uart::TX_SENSOR_SIGNAL.signal(sensor_data);
        }
    }
```

- [ ] **Step 8: Add command handling in main loop**

In the main button-handling loop, add non-blocking command check. After the existing button handling logic:

```rust
    // UART受信コマンド処理
    if uart::RX_COMMAND_SIGNAL.signaled() {
        let cmd = uart::RX_COMMAND_SIGNAL.wait().await;
        match cmd.command_type {
            inverted_pendulum_protocol::CommandType::Start => {
                RUNNING.store(true, core::sync::atomic::Ordering::Relaxed);
            }
            inverted_pendulum_protocol::CommandType::Stop => {
                RUNNING.store(false, core::sync::atomic::Ordering::Relaxed);
            }
            inverted_pendulum_protocol::CommandType::ModeChange => {
                let mode = cmd.payload[0] % 5;
                CONTROL_MODE.store(mode, core::sync::atomic::Ordering::Relaxed);
            }
            inverted_pendulum_protocol::CommandType::ParamSet => {
                // 将来対応
            }
        }
    }
```

- [ ] **Step 9: Verify STM32 compiles**

Run: `cd inverted-pendulum-stm32 && cargo build`
Expected: Compiles. The exact USART2 DMA channel assignments and interrupt bindings may need adjustment based on the STM32F303K8 DMA channel availability. Check embassy-stm32 docs if needed.

- [ ] **Step 10: Commit**

```bash
git add inverted-pendulum-stm32/
git commit -m "feat(stm32): add UART communication with ESP32 via protocol crate"
```

---

### Task 10: Final integration verification

**Files:**
- All three crates

- [ ] **Step 1: Run protocol tests**

Run: `cd inverted-pendulum-protocol && cargo test`
Expected: All tests pass.

- [ ] **Step 2: Build STM32 firmware**

Run: `cd inverted-pendulum-stm32 && cargo build`
Expected: Compiles without errors.

- [ ] **Step 3: Build ESP32 firmware**

Run: `cd inverted-pendulum-esp32 && WIFI_SSID=test WIFI_PASSWORD=test cargo build`
Expected: Compiles without errors.

- [ ] **Step 4: Build ESP32 release**

Run: `cd inverted-pendulum-esp32 && WIFI_SSID=test WIFI_PASSWORD=test cargo build --release`
Expected: Compiles without errors.

- [ ] **Step 5: Commit any fixups**

If any fixes were needed:
```bash
git add -A
git commit -m "fix: resolve integration build issues"
```
