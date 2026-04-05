pub mod lqr;
pub mod mrac;
pub mod observer;
pub mod pid_balance;

use crate::constants::*;
use crate::filter::LowPassFilter;
use crate::pid::Pid;

use lqr::LqrController;
use mrac::MracController;
use observer::StateObserver;
use pid_balance::PidBalanceController;

/// センサー状態（制御ループへの入力）
pub struct State {
    pub theta: f32,      // 振り子角度 [rad]
    pub position_r: f32, // 右車輪位置 [m]
    pub position_l: f32, // 左車輪位置 [m]
    pub velocity_r: f32, // 右車輪速度 [m/s]
    pub velocity_l: f32, // 左車輪速度 [m/s]
    pub current_r: f32,  // 右モーター電流 [A]
    pub current_l: f32,  // 左モーター電流 [A]
    pub vin: f32,        // バッテリー電圧 [V]
}

/// モーター出力デューティ比
pub struct MotorOutput {
    pub left: f32,
    pub right: f32,
}

/// 前処理済み状態（内部制御器への入力）
pub struct ProcessedState {
    pub position: f32,
    pub velocity: f32,
    pub theta: f32,
    pub theta_dot: f32,
}

/// 制御モード
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum ControlMode {
    Debug = 0,
    Pid = 1,
    Lqr = 2,
    Mrac = 3,
}

impl ControlMode {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => ControlMode::Debug,
            1 => ControlMode::Pid,
            2 => ControlMode::Lqr,
            _ => ControlMode::Mrac,
        }
    }
}

/// 統合制御システム
/// 共通の前処理・電流制御パイプラインと、モード切り替え可能な力計算を統合
pub struct ControlSystem {
    mode: ControlMode,
    lqr: LqrController,
    pid_balance: PidBalanceController,
    mrac: MracController,

    // 状態推定
    theta_filter: LowPassFilter,
    theta_dot_filter: LowPassFilter,
    prev_theta: f32,
    theta_initialized: bool,
    observer: StateObserver,

    // 電流制御（共通パイプライン）
    target_current: f32,
    target_voltage: f32,
    pid_l: Pid,
    pid_r: Pid,
    current_filter_l: LowPassFilter,
    current_filter_r: LowPassFilter,
    prev_voltage_l: f32,
    prev_voltage_r: f32,
}

impl ControlSystem {
    pub fn new() -> Self {
        Self {
            mode: ControlMode::Lqr,
            lqr: LqrController::new(),
            pid_balance: PidBalanceController::new(),
            mrac: MracController::new(),
            theta_filter: LowPassFilter::new(BALANCE_DT, THETA_FILTER_CUTOFF),
            theta_dot_filter: LowPassFilter::new(BALANCE_DT, THETA_DOT_FILTER_CUTOFF),
            prev_theta: 0.0,
            theta_initialized: false,
            observer: StateObserver::new(),
            target_current: 0.0,
            target_voltage: 0.0,
            pid_l: Pid::new(
                CURRENT_PID_KP,
                CURRENT_PID_KI,
                CURRENT_PID_KD,
                CURRENT_DT,
                -MAX_VOLTAGE,
                MAX_VOLTAGE,
            ),
            pid_r: Pid::new(
                CURRENT_PID_KP,
                CURRENT_PID_KI,
                CURRENT_PID_KD,
                CURRENT_DT,
                -MAX_VOLTAGE,
                MAX_VOLTAGE,
            ),
            current_filter_l: LowPassFilter::new(CURRENT_DT, CURRENT_FILTER_CUTOFF),
            current_filter_r: LowPassFilter::new(CURRENT_DT, CURRENT_FILTER_CUTOFF),
            prev_voltage_l: 0.0,
            prev_voltage_r: 0.0,
        }
    }

    pub fn set_mode(&mut self, mode: ControlMode) {
        if self.mode != mode {
            self.mode = mode;
            // 新モードの内部状態をリセット
            match mode {
                ControlMode::Debug => {}
                ControlMode::Lqr => self.lqr.reset(),
                ControlMode::Pid => self.pid_balance.reset(),
                ControlMode::Mrac => self.mrac.reset(),
            }
        }
    }

    /// 振り子制御ループ (1kHz): 状態 → 力 → 電流目標値を更新
    pub fn update_balance(&mut self, state: &State) {
        // 前処理: 左右平均
        let position = (state.position_r + state.position_l) / 2.0;
        let velocity = (state.velocity_r + state.velocity_l) / 2.0;
        let theta = self.theta_filter.update(state.theta);
        let theta_dot_raw = if self.theta_initialized {
            (theta - self.prev_theta) / BALANCE_DT
        } else {
            self.theta_initialized = true;
            0.0
        };
        self.prev_theta = theta;
        let theta_dot = self.theta_dot_filter.update(theta_dot_raw);

        let estimated = self.observer.update(position, theta, self.target_current);

        let processed_observer = ProcessedState {
            position,
            velocity: estimated.velocity,
            theta,
            theta_dot: estimated.theta_dot,
        };
        let processed_direct = ProcessedState {
            position,
            velocity,
            theta,
            theta_dot,
        };

        // モード依存: 力の計算
        let force = match self.mode {
            ControlMode::Debug => {
                self.target_voltage = 0.0;
                0.0
            }
            ControlMode::Lqr => {
                self.target_voltage = 0.0;
                self.lqr.compute_force(&processed_observer)
            }
            ControlMode::Pid => {
                self.target_voltage = 0.0;
                self.pid_balance.compute_force(&processed_observer)
            }
            // mode4 は電流状態込みの拡大系を使うため、電圧を直接出力する
            ControlMode::Mrac => {
                self.target_current = 0.0;
                self.target_voltage = self.mrac.compute_voltage(&processed_direct, state.vin);
                return;
            }
        };
        self.target_current = clamp(force, -MAX_FORCE, MAX_FORCE) * FORCE_TO_CURRENT;
    }

    /// 電流制御ループ (10kHz): 電流PID → デューティ比
    pub fn update_current(&mut self, state: &State) -> MotorOutput {
        let vin = if state.vin > 1.0 { state.vin } else { 7.2 };

        if self.mode == ControlMode::Mrac {
            let voltage = clamp(self.target_voltage, -vin, vin);
            let duty = clamp(voltage / vin, -1.0, 1.0);
            self.prev_voltage_l = voltage;
            self.prev_voltage_r = voltage;
            return MotorOutput {
                left: duty,
                right: duty,
            };
        }

        // 電流センサーは絶対値のみ計測するため、電圧コマンドの符号で電流の方向を補正
        let corrected_l = correct_current_sign(state.current_l, self.prev_voltage_l);
        let corrected_r = correct_current_sign(state.current_r, self.prev_voltage_r);

        let filtered_l = self.current_filter_l.update(corrected_l);
        let filtered_r = self.current_filter_r.update(corrected_r);

        let voltage_l = self.pid_l.update(self.target_current, filtered_l);
        let voltage_r = self.pid_r.update(self.target_current, filtered_r);

        self.prev_voltage_l = voltage_l;
        self.prev_voltage_r = voltage_r;

        let duty_l = clamp(voltage_l / vin, -1.0, 1.0);
        let duty_r = clamp(voltage_r / vin, -1.0, 1.0);

        MotorOutput {
            left: duty_l,
            right: duty_r,
        }
    }

    pub fn reset(&mut self) {
        self.theta_filter.reset();
        self.theta_dot_filter.reset();
        self.prev_theta = 0.0;
        self.theta_initialized = false;
        self.observer.reset();
        self.target_current = 0.0;
        self.target_voltage = 0.0;
        self.pid_l.reset();
        self.pid_r.reset();
        self.current_filter_l.reset();
        self.current_filter_r.reset();
        self.prev_voltage_l = 0.0;
        self.prev_voltage_r = 0.0;
        self.lqr.reset();
        self.pid_balance.reset();
        self.mrac.reset();
    }
}

/// 電流センサーの符号補正
/// シャント抵抗は電流の絶対値のみ計測するため、
/// 前回の電圧コマンドの方向から電流の符号を推定する
/// ゼロ近傍では滑らかに遷移させて不連続を防止
fn correct_current_sign(measured_current: f32, voltage_command: f32) -> f32 {
    const DEADBAND: f32 = 0.5;
    let abs_current = measured_current.abs();
    if voltage_command.abs() < DEADBAND {
        // デッドバンド内: 線形補間で滑らかに遷移
        abs_current * (voltage_command / DEADBAND)
    } else if voltage_command > 0.0 {
        abs_current
    } else {
        -abs_current
    }
}
