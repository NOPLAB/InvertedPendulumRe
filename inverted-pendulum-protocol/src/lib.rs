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
