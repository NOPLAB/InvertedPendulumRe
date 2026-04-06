// Designed in MATLAB/control/design_pid.m
// 角度制御（内側ループ）
pub const BALANCE_ANGLE_KP: f32 = 50.0;
pub const BALANCE_ANGLE_KI: f32 = 5.0;
pub const BALANCE_ANGLE_KD: f32 = 15.0;

// 位置制御（外側ループ）
pub const BALANCE_POS_KP: f32 = 10.0;
pub const BALANCE_POS_KI: f32 = 0.5;
pub const BALANCE_POS_KD: f32 = 15.0;
