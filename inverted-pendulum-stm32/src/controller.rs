use crate::constants::*;
use crate::filter::LowPassFilter;
use crate::pid::Pid;

/// Sensor state fed into the controller each cycle
pub struct State {
    pub theta: f32,        // Pendulum angle [rad]
    pub position_r: f32,   // Right wheel position [m]
    pub position_l: f32,   // Left wheel position [m]
    pub velocity_r: f32,   // Right wheel velocity [m/s]
    pub velocity_l: f32,   // Left wheel velocity [m/s]
    pub current_r: f32,    // Right motor current [A]
    pub current_l: f32,    // Left motor current [A]
    pub vin: f32,          // Battery voltage [V]
}

/// Output duty cycles for left/right motors
pub struct MotorOutput {
    pub left: f32,
    pub right: f32,
}

/// LQR state feedback + current PID controller
pub struct Controller {
    // LQR gains
    k_pos: f32,
    k_vel: f32,
    k_ang: f32,
    k_ang_vel: f32,

    // Theta processing
    theta_filter: LowPassFilter,
    prev_theta: f32,
    theta_vel: f32,

    // Current control
    pid_l: Pid,
    pid_r: Pid,
    current_filter_l: LowPassFilter,
    current_filter_r: LowPassFilter,
}

impl Controller {
    pub fn new() -> Self {
        Self {
            k_pos: K_POSITION,
            k_vel: K_VELOCITY,
            k_ang: K_ANGLE,
            k_ang_vel: K_ANGULAR_VELOCITY,
            theta_filter: LowPassFilter::new(DT, THETA_FILTER_CUTOFF),
            prev_theta: 0.0,
            theta_vel: 0.0,
            pid_l: Pid::new(
                CURRENT_PID_KP,
                CURRENT_PID_KI,
                CURRENT_PID_KD,
                DT,
                -MAX_VOLTAGE,
                MAX_VOLTAGE,
            ),
            pid_r: Pid::new(
                CURRENT_PID_KP,
                CURRENT_PID_KI,
                CURRENT_PID_KD,
                DT,
                -MAX_VOLTAGE,
                MAX_VOLTAGE,
            ),
            current_filter_l: LowPassFilter::new(DT, CURRENT_FILTER_CUTOFF),
            current_filter_r: LowPassFilter::new(DT, CURRENT_FILTER_CUTOFF),
        }
    }

    #[allow(dead_code)]
    pub fn set_gains(&mut self, k_pos: f32, k_vel: f32, k_ang: f32, k_ang_vel: f32) {
        self.k_pos = k_pos;
        self.k_vel = k_vel;
        self.k_ang = k_ang;
        self.k_ang_vel = k_ang_vel;
    }

    /// Compute motor duty cycles from sensor state.
    /// Full pipeline: LQR force -> current reference -> PID voltage -> duty cycle
    pub fn update(&mut self, state: &State) -> MotorOutput {
        // Average position and velocity
        let position = (state.position_r + state.position_l) / 2.0;
        let velocity = (state.velocity_r + state.velocity_l) / 2.0;

        // Filter theta and compute angular velocity
        let theta = self.theta_filter.update(state.theta);
        self.theta_vel = (theta - self.prev_theta) / DT;
        self.prev_theta = theta;

        // LQR: u = -K * x
        let force = -(self.k_pos * position
            + self.k_vel * velocity
            + self.k_ang * theta
            + self.k_ang_vel * self.theta_vel);
        let force = clamp(force, -MAX_FORCE, MAX_FORCE);

        // Force -> current reference
        let target_current = force * FORCE_TO_CURRENT;

        // Filter measured currents
        let filtered_l = self.current_filter_l.update(state.current_l);
        let filtered_r = self.current_filter_r.update(state.current_r);

        // Current PID -> voltage
        let voltage_l = self.pid_l.update(target_current, filtered_l);
        let voltage_r = self.pid_r.update(target_current, filtered_r);

        // Voltage -> duty cycle
        let vin = if state.vin > 1.0 { state.vin } else { 7.2 };
        let duty_l = clamp(voltage_l / vin, -1.0, 1.0);
        let duty_r = clamp(voltage_r / vin, -1.0, 1.0);

        MotorOutput {
            left: duty_l,
            right: duty_r,
        }
    }

    pub fn reset(&mut self) {
        self.theta_filter.reset();
        self.prev_theta = 0.0;
        self.theta_vel = 0.0;
        self.pid_l.reset();
        self.pid_r.reset();
        self.current_filter_l.reset();
        self.current_filter_r.reset();
    }
}
