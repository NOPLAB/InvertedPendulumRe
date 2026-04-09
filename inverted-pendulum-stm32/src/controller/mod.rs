mod filter;
mod pid;

use crate::config::{
    clamp, BALANCE_DT, CURRENT_DT, FORCE_TO_CURRENT, MAX_FORCE, MAX_VOLTAGE,
};
use filter::LowPassFilter;
use pid::Pid;

use inverted_pendulum_controller as ctrl;

// フィルタカットオフ周波数
const THETA_FILTER_CUTOFF: f32 = 50.0; // [Hz]
const THETA_DOT_FILTER_CUTOFF: f32 = 25.0; // [Hz]
const CURRENT_FILTER_CUTOFF: f32 = 500.0; // [Hz]

// 電流PID
const CURRENT_PID_KP: f32 = 0.928;
const CURRENT_PID_KI: f32 = 10178.8;
const CURRENT_PID_KD: f32 = 0.0;

// MPC間引き: 1kHz / 100Hz = 10
const MPC_DECIMATION: u32 = crate::config::BALANCE_CONTROL_FREQUENCY / crate::config::MPC_CONTROL_FREQUENCY;

pub use inverted_pendulum_protocol::ControlMode;

/// センサー状態（バランス制御への入力）
pub struct BalanceState {
    pub theta: f32,      // 振り子角度 [rad]
    pub position_r: f32, // 右車輪位置 [m]
    pub position_l: f32, // 左車輪位置 [m]
}

/// センサー状態（電流制御への入力）
pub struct CurrentState {
    pub current_r: f32, // 右モーター電流 [A]
    pub current_l: f32, // 左モーター電流 [A]
    pub vin: f32,       // バッテリー電圧 [V]
}

/// モーター出力デューティ比
pub struct MotorOutput {
    pub left: f32,
    pub right: f32,
}

/// バランス制御（1kHz タスクで使用）
/// 状態推定 → 力計算 → target_current を出力
pub struct BalanceController {
    mode: ControlMode,

    // 状態推定
    theta_filter: LowPassFilter,
    theta_dot_filter: LowPassFilter,
    prev_theta: f32,
    theta_initialized: bool,

    // 出力
    target_current: f32,

    // MPC間引き
    mpc_counter: u32,
    prev_force: f32,
}

impl BalanceController {
    pub fn new() -> Self {
        ctrl::init_lqr();
        ctrl::init_pid();
        ctrl::init_mpc();
        ctrl::init_mrac();
        ctrl::init_observer();

        Self {
            mode: ControlMode::Lqr,
            theta_filter: LowPassFilter::new(BALANCE_DT, THETA_FILTER_CUTOFF),
            theta_dot_filter: LowPassFilter::new(BALANCE_DT, THETA_DOT_FILTER_CUTOFF),
            prev_theta: 0.0,
            theta_initialized: false,
            target_current: 0.0,
            mpc_counter: 0,
            prev_force: 0.0,
        }
    }

    pub fn set_mode(&mut self, mode: ControlMode) {
        if self.mode != mode {
            self.mode = mode;
            match mode {
                ControlMode::Debug => {}
                ControlMode::Lqr => ctrl::init_lqr(),
                ControlMode::Pid => ctrl::init_pid(),
                ControlMode::Mrac => ctrl::init_mrac(),
                ControlMode::Mpc => ctrl::init_mpc(),
            }
            ctrl::init_observer();
            self.mpc_counter = 0;
        }
    }

    /// バランス制御ループ (1kHz): 状態 → 力 → target_current を返す
    pub fn update(&mut self, state: &BalanceState) -> f32 {
        // 前処理: 左右平均
        let position = (state.position_r + state.position_l) / 2.0;
        let theta = self.theta_filter.update(state.theta);
        let theta_dot_raw = if self.theta_initialized {
            (theta - self.prev_theta) / BALANCE_DT
        } else {
            self.theta_initialized = true;
            0.0
        };
        self.prev_theta = theta;
        let _theta_dot = self.theta_dot_filter.update(theta_dot_raw);

        // オブザーバで状態推定
        let prev_force = self.target_current / FORCE_TO_CURRENT;
        let x_hat = ctrl::observer_update(position, theta, prev_force);

        // モード依存: 力の計算
        let force = match self.mode {
            ControlMode::Debug => 0.0,
            ControlMode::Lqr => ctrl::lqr_compute(&x_hat),
            ControlMode::Pid => ctrl::pid_compute(&x_hat),
            ControlMode::Mrac => ctrl::mrac_compute(&x_hat),
            ControlMode::Mpc => {
                // MPC: 100Hzに間引き、非実行ティックは前回値を保持
                if self.mpc_counter == 0 {
                    self.prev_force = ctrl::mpc_compute(&x_hat);
                }
                self.mpc_counter = (self.mpc_counter + 1) % MPC_DECIMATION;
                self.prev_force
            }
        };

        self.target_current = clamp(force, -MAX_FORCE, MAX_FORCE) * FORCE_TO_CURRENT;
        self.target_current
    }

    pub fn reset(&mut self) {
        self.theta_filter.reset();
        self.theta_dot_filter.reset();
        self.prev_theta = 0.0;
        self.theta_initialized = false;
        self.target_current = 0.0;
        self.mpc_counter = 0;
        self.prev_force = 0.0;
        ctrl::init_observer();
        match self.mode {
            ControlMode::Debug => {}
            ControlMode::Lqr => ctrl::init_lqr(),
            ControlMode::Pid => ctrl::init_pid(),
            ControlMode::Mrac => ctrl::init_mrac(),
            ControlMode::Mpc => ctrl::init_mpc(),
        }
    }
}

/// 電流制御（5kHz タスクで使用）
/// target_current → 電流PID → デューティ比
pub struct CurrentController {
    pid_l: Pid,
    pid_r: Pid,
    current_filter_l: LowPassFilter,
    current_filter_r: LowPassFilter,
    prev_voltage_l: f32,
    prev_voltage_r: f32,
}

impl CurrentController {
    pub fn new() -> Self {
        Self {
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

    /// 電流制御ループ (5kHz): target_current → デューティ比
    pub fn update(&mut self, target_current: f32, state: &CurrentState) -> MotorOutput {
        let vin = if state.vin > 1.0 { state.vin } else { 7.2 };

        let corrected_l = correct_current_sign(state.current_l, self.prev_voltage_l);
        let corrected_r = correct_current_sign(state.current_r, self.prev_voltage_r);

        let filtered_l = self.current_filter_l.update(corrected_l);
        let filtered_r = self.current_filter_r.update(corrected_r);

        let voltage_l = self.pid_l.update(target_current, filtered_l);
        let voltage_r = self.pid_r.update(target_current, filtered_r);

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
        self.pid_l.reset();
        self.pid_r.reset();
        self.current_filter_l.reset();
        self.current_filter_r.reset();
        self.prev_voltage_l = 0.0;
        self.prev_voltage_r = 0.0;
    }
}

/// 電流センサーの符号補正
fn correct_current_sign(measured_current: f32, voltage_command: f32) -> f32 {
    const DEADBAND: f32 = 0.5;
    let abs_current = measured_current.abs();
    if voltage_command.abs() < DEADBAND {
        abs_current * (voltage_command / DEADBAND)
    } else if voltage_command > 0.0 {
        abs_current
    } else {
        -abs_current
    }
}
