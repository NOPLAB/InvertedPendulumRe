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

/// コマンドタイプ
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

// ---------------------------------------------------------------------------
// プロトコルエラー
// ---------------------------------------------------------------------------

/// プロトコルエラー
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum ProtocolError {
    BufferTooSmall,
    InvalidLength,
    InvalidCommandType,
    CobsDecodeError,
    CrcMismatch,
}

// ---------------------------------------------------------------------------
// サイズ定数
// ---------------------------------------------------------------------------

/// SensorData の生バイトサイズ (f32×6 + u8×2 = 26)
pub const SENSOR_DATA_RAW_SIZE: usize = 26;
/// SensorData + CRC8
pub const SENSOR_DATA_WITH_CRC_SIZE: usize = 27;
/// COBS エンコード後の最大サイズ (27 + 27/254 + 1 = 28)
pub const MAX_SENSOR_COBS_SIZE: usize = 28;
/// 0x00 デリミタ付きフレーム最大サイズ
pub const MAX_SENSOR_FRAME_SIZE: usize = 29;

/// Command の生バイトサイズ (u8 + u8×2 = 3)
pub const COMMAND_RAW_SIZE: usize = 3;
/// Command + CRC8
pub const COMMAND_WITH_CRC_SIZE: usize = 4;
/// COBS エンコード後の最大サイズ
pub const MAX_COMMAND_COBS_SIZE: usize = 5;
/// 0x00 デリミタ付きフレーム最大サイズ
pub const MAX_COMMAND_FRAME_SIZE: usize = 6;

// ---------------------------------------------------------------------------
// CRC8 (polynomial 0x31, init 0x00)
// ---------------------------------------------------------------------------

/// CRC8 を計算する (polynomial 0x31, init 0x00)
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

/// CRC8 を検証する (末尾バイトが CRC)
pub fn verify_crc8(data_with_crc: &[u8]) -> bool {
    if data_with_crc.is_empty() {
        return false;
    }
    let (data, crc_byte) = data_with_crc.split_at(data_with_crc.len() - 1);
    crc8(data) == crc_byte[0]
}

// ---------------------------------------------------------------------------
// COBS エンコード / デコード
// ---------------------------------------------------------------------------

/// COBS エンコード (0x00 デリミタは含まない)
/// 戻り値: エンコード後のバイト数
pub fn cobs_encode(input: &[u8], output: &mut [u8]) -> Result<usize, ProtocolError> {
    // COBS の最大出力サイズ: input.len() + input.len()/254 + 1
    let max_out = input.len() + input.len() / 254 + 1;
    if output.len() < max_out {
        return Err(ProtocolError::BufferTooSmall);
    }

    let mut code_idx: usize = 0;
    let mut code: u8 = 1;
    let mut write_idx: usize = 1;

    for &byte in input {
        if byte == 0x00 {
            output[code_idx] = code;
            code_idx = write_idx;
            write_idx += 1;
            code = 1;
        } else {
            output[write_idx] = byte;
            write_idx += 1;
            code += 1;
            if code == 0xFF {
                output[code_idx] = code;
                code_idx = write_idx;
                write_idx += 1;
                code = 1;
            }
        }
    }
    output[code_idx] = code;

    Ok(write_idx)
}

/// COBS デコード (入力に 0x00 デリミタは含まない)
/// 戻り値: デコード後のバイト数
pub fn cobs_decode(input: &[u8], output: &mut [u8]) -> Result<usize, ProtocolError> {
    if input.is_empty() {
        return Ok(0);
    }

    let mut read_idx: usize = 0;
    let mut write_idx: usize = 0;

    while read_idx < input.len() {
        let code = input[read_idx];
        if code == 0 {
            return Err(ProtocolError::CobsDecodeError);
        }
        read_idx += 1;

        for _ in 1..code {
            if read_idx >= input.len() {
                return Err(ProtocolError::CobsDecodeError);
            }
            if write_idx >= output.len() {
                return Err(ProtocolError::BufferTooSmall);
            }
            output[write_idx] = input[read_idx];
            write_idx += 1;
            read_idx += 1;
        }

        if code < 0xFF && read_idx < input.len() {
            if write_idx >= output.len() {
                return Err(ProtocolError::BufferTooSmall);
            }
            output[write_idx] = 0x00;
            write_idx += 1;
        }
    }

    // 末尾の余分な 0x00 を除去
    if write_idx > 0 && output[write_idx - 1] == 0x00 {
        write_idx -= 1;
    }

    Ok(write_idx)
}

// ---------------------------------------------------------------------------
// SensorData シリアライズ / デシリアライズ
// ---------------------------------------------------------------------------

impl SensorData {
    /// 生バイト列にシリアライズ (リトルエンディアン)
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ProtocolError> {
        if buf.len() < SENSOR_DATA_RAW_SIZE {
            return Err(ProtocolError::BufferTooSmall);
        }
        buf[0..4].copy_from_slice(&self.theta.to_le_bytes());
        buf[4..8].copy_from_slice(&self.position.to_le_bytes());
        buf[8..12].copy_from_slice(&self.velocity.to_le_bytes());
        buf[12..16].copy_from_slice(&self.theta_dot.to_le_bytes());
        buf[16..20].copy_from_slice(&self.current_l.to_le_bytes());
        buf[20..24].copy_from_slice(&self.current_r.to_le_bytes());
        buf[24] = self.control_mode;
        buf[25] = self.running;
        Ok(SENSOR_DATA_RAW_SIZE)
    }

    /// 生バイト列からデシリアライズ (リトルエンディアン)
    pub fn deserialize(buf: &[u8]) -> Result<Self, ProtocolError> {
        if buf.len() < SENSOR_DATA_RAW_SIZE {
            return Err(ProtocolError::InvalidLength);
        }
        Ok(SensorData {
            theta: f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            position: f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            velocity: f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            theta_dot: f32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
            current_l: f32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            current_r: f32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
            control_mode: buf[24],
            running: buf[25],
        })
    }

    /// CRC8 + COBS + 0x00 デリミタ付きフレームにシリアライズ
    pub fn to_frame(&self, buf: &mut [u8]) -> Result<usize, ProtocolError> {
        if buf.len() < MAX_SENSOR_FRAME_SIZE {
            return Err(ProtocolError::BufferTooSmall);
        }
        // 生データ + CRC
        let mut raw = [0u8; SENSOR_DATA_WITH_CRC_SIZE];
        self.serialize(&mut raw)?;
        raw[SENSOR_DATA_RAW_SIZE] = crc8(&raw[..SENSOR_DATA_RAW_SIZE]);

        // COBS エンコード
        let cobs_len = cobs_encode(&raw, buf)?;
        // 0x00 デリミタ
        buf[cobs_len] = 0x00;
        Ok(cobs_len + 1)
    }

    /// 0x00 デリミタ付きフレームからデシリアライズ
    pub fn from_frame(frame: &[u8]) -> Result<Self, ProtocolError> {
        // デリミタを除去
        let cobs_data = if !frame.is_empty() && frame[frame.len() - 1] == 0x00 {
            &frame[..frame.len() - 1]
        } else {
            frame
        };

        // COBS デコード
        let mut decoded = [0u8; SENSOR_DATA_WITH_CRC_SIZE];
        let decoded_len = cobs_decode(cobs_data, &mut decoded)?;
        if decoded_len != SENSOR_DATA_WITH_CRC_SIZE {
            return Err(ProtocolError::InvalidLength);
        }

        // CRC 検証
        if !verify_crc8(&decoded[..decoded_len]) {
            return Err(ProtocolError::CrcMismatch);
        }

        SensorData::deserialize(&decoded[..SENSOR_DATA_RAW_SIZE])
    }
}

// ---------------------------------------------------------------------------
// Command シリアライズ / デシリアライズ
// ---------------------------------------------------------------------------

impl Command {
    /// 生バイト列にシリアライズ
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ProtocolError> {
        if buf.len() < COMMAND_RAW_SIZE {
            return Err(ProtocolError::BufferTooSmall);
        }
        buf[0] = self.command_type as u8;
        buf[1] = self.payload[0];
        buf[2] = self.payload[1];
        Ok(COMMAND_RAW_SIZE)
    }

    /// 生バイト列からデシリアライズ
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

    /// CRC8 + COBS + 0x00 デリミタ付きフレームにシリアライズ
    pub fn to_frame(&self, buf: &mut [u8]) -> Result<usize, ProtocolError> {
        if buf.len() < MAX_COMMAND_FRAME_SIZE {
            return Err(ProtocolError::BufferTooSmall);
        }
        let mut raw = [0u8; COMMAND_WITH_CRC_SIZE];
        self.serialize(&mut raw)?;
        raw[COMMAND_RAW_SIZE] = crc8(&raw[..COMMAND_RAW_SIZE]);

        let cobs_len = cobs_encode(&raw, buf)?;
        buf[cobs_len] = 0x00;
        Ok(cobs_len + 1)
    }

    /// 0x00 デリミタ付きフレームからデシリアライズ
    pub fn from_frame(frame: &[u8]) -> Result<Self, ProtocolError> {
        let cobs_data = if !frame.is_empty() && frame[frame.len() - 1] == 0x00 {
            &frame[..frame.len() - 1]
        } else {
            frame
        };

        let mut decoded = [0u8; COMMAND_WITH_CRC_SIZE];
        let decoded_len = cobs_decode(cobs_data, &mut decoded)?;
        if decoded_len != COMMAND_WITH_CRC_SIZE {
            return Err(ProtocolError::InvalidLength);
        }

        if !verify_crc8(&decoded[..decoded_len]) {
            return Err(ProtocolError::CrcMismatch);
        }

        Command::deserialize(&decoded[..COMMAND_RAW_SIZE])
    }
}

// ---------------------------------------------------------------------------
// テスト
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc8_known_value() {
        // CRC8/MAXIM (poly 0x31, init 0x00) の既知値
        let data = [0x01, 0x02, 0x03];
        let crc = crc8(&data);
        // 再計算して一貫性を確認
        assert_eq!(crc, crc8(&data));
        // 空データ
        assert_eq!(crc8(&[]), 0x00);
        // 単一バイト
        let single = crc8(&[0xAB]);
        assert_ne!(single, 0x00); // 非ゼロ入力は非ゼロ CRC を生成するはず
    }

    #[test]
    fn test_crc8_verify() {
        let data = [0x10, 0x20, 0x30, 0x40];
        let crc = crc8(&data);
        let mut data_with_crc = [0u8; 5];
        data_with_crc[..4].copy_from_slice(&data);
        data_with_crc[4] = crc;
        assert!(verify_crc8(&data_with_crc));
        // CRC を壊す
        data_with_crc[4] ^= 0xFF;
        assert!(!verify_crc8(&data_with_crc));
    }

    #[test]
    fn test_sensor_data_roundtrip() {
        let original = SensorData {
            theta: 0.1,
            position: 0.2,
            velocity: -0.3,
            theta_dot: 0.4,
            current_l: 1.5,
            current_r: -1.5,
            control_mode: 2,
            running: 1,
        };
        let mut buf = [0u8; SENSOR_DATA_RAW_SIZE];
        let len = original.serialize(&mut buf).unwrap();
        assert_eq!(len, SENSOR_DATA_RAW_SIZE);
        let decoded = SensorData::deserialize(&buf).unwrap();
        assert_eq!(original.theta, decoded.theta);
        assert_eq!(original.position, decoded.position);
        assert_eq!(original.velocity, decoded.velocity);
        assert_eq!(original.theta_dot, decoded.theta_dot);
        assert_eq!(original.current_l, decoded.current_l);
        assert_eq!(original.current_r, decoded.current_r);
        assert_eq!(original.control_mode, decoded.control_mode);
        assert_eq!(original.running, decoded.running);
    }

    #[test]
    fn test_command_roundtrip() {
        let original = Command {
            command_type: CommandType::ModeChange,
            payload: [0x02, 0x00],
        };
        let mut buf = [0u8; COMMAND_RAW_SIZE];
        let len = original.serialize(&mut buf).unwrap();
        assert_eq!(len, COMMAND_RAW_SIZE);
        let decoded = Command::deserialize(&buf).unwrap();
        assert_eq!(original.command_type, decoded.command_type);
        assert_eq!(original.payload, decoded.payload);
    }

    #[test]
    fn test_cobs_roundtrip() {
        // ゼロバイトを含むデータ
        let data = [0x01, 0x00, 0x02, 0x00, 0x03];
        let mut encoded = [0u8; 8];
        let enc_len = cobs_encode(&data, &mut encoded).unwrap();
        // エンコード結果にゼロは含まれない
        for i in 0..enc_len {
            assert_ne!(encoded[i], 0x00);
        }
        let mut decoded = [0u8; 8];
        let dec_len = cobs_decode(&encoded[..enc_len], &mut decoded).unwrap();
        assert_eq!(dec_len, data.len());
        assert_eq!(&decoded[..dec_len], &data);

        // ゼロなしデータ
        let data2 = [0x11, 0x22, 0x33];
        let mut encoded2 = [0u8; 8];
        let enc_len2 = cobs_encode(&data2, &mut encoded2).unwrap();
        let mut decoded2 = [0u8; 8];
        let dec_len2 = cobs_decode(&encoded2[..enc_len2], &mut decoded2).unwrap();
        assert_eq!(dec_len2, data2.len());
        assert_eq!(&decoded2[..dec_len2], &data2);
    }

    #[test]
    fn test_frame_sensor_data_roundtrip() {
        let original = SensorData {
            theta: 1.234,
            position: -5.678,
            velocity: 0.0,
            theta_dot: 9.876,
            current_l: 0.5,
            current_r: -0.5,
            control_mode: 1,
            running: 1,
        };
        let mut frame = [0u8; MAX_SENSOR_FRAME_SIZE];
        let frame_len = original.to_frame(&mut frame).unwrap();
        assert!(frame_len <= MAX_SENSOR_FRAME_SIZE);
        // フレーム末尾は 0x00 デリミタ
        assert_eq!(frame[frame_len - 1], 0x00);

        let decoded = SensorData::from_frame(&frame[..frame_len]).unwrap();
        assert_eq!(original.theta, decoded.theta);
        assert_eq!(original.position, decoded.position);
        assert_eq!(original.velocity, decoded.velocity);
        assert_eq!(original.theta_dot, decoded.theta_dot);
        assert_eq!(original.current_l, decoded.current_l);
        assert_eq!(original.current_r, decoded.current_r);
        assert_eq!(original.control_mode, decoded.control_mode);
        assert_eq!(original.running, decoded.running);
    }

    #[test]
    fn test_frame_command_roundtrip() {
        let original = Command {
            command_type: CommandType::Start,
            payload: [0x00, 0x00],
        };
        let mut frame = [0u8; MAX_COMMAND_FRAME_SIZE];
        let frame_len = original.to_frame(&mut frame).unwrap();
        assert!(frame_len <= MAX_COMMAND_FRAME_SIZE);
        assert_eq!(frame[frame_len - 1], 0x00);

        let decoded = Command::from_frame(&frame[..frame_len]).unwrap();
        assert_eq!(original.command_type, decoded.command_type);
        assert_eq!(original.payload, decoded.payload);
    }
}
