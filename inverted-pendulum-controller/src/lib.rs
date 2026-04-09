#![no_std]

mod ffi;

/// LQR制御器の内部状態をリセット
pub fn init_lqr() {
    unsafe { ffi::lqr_step_initialize(); }
}

/// PID制御器の内部状態（積分器等）をリセット
pub fn init_pid() {
    unsafe { ffi::pid_step_initialize(); }
}

/// MPC制御器の内部状態（ADMM warm start）をリセット
pub fn init_mpc() {
    unsafe { ffi::mpc_step_initialize(); }
}

/// MRAC制御器の内部状態（適応パラメータ等）をリセット
pub fn init_mrac() {
    unsafe { ffi::mrac_step_initialize(); }
}

/// 状態オブザーバの内部状態をリセット
pub fn init_observer() {
    unsafe { ffi::observer_step_initialize(); }
}

/// LQR制御力を計算
/// state: [x, x_dot, theta, theta_dot]
/// 戻り値: 制御力 [N]
pub fn lqr_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::lqr_step(state.as_ptr()) }
}

/// PID制御力を計算（カスケードPID: 角度+位置）
pub fn pid_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::pid_step(state.as_ptr()) }
}

/// MPC制御力を計算（ADMM QPソルバ）
pub fn mpc_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::mpc_step(state.as_ptr()) }
}

/// MRAC制御力を計算（適応制御）
pub fn mrac_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::mrac_step(state.as_ptr()) }
}

/// 状態オブザーバ更新
/// 戻り値: 推定状態 [x, x_dot, theta, theta_dot]
pub fn observer_update(position: f32, theta: f32, force: f32) -> [f32; 4] {
    let mut x_hat = [0.0f32; 4];
    unsafe { ffi::observer_step(position, theta, force, x_hat.as_mut_ptr()); }
    x_hat
}
