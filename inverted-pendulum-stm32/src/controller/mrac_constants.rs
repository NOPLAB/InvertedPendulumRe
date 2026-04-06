// Designed in MATLAB/control/design_mrac.m
// 4状態 力出力 MRAC 定数

pub const MRAC_KX: [f32; 4] = [-3.162_277_7, -3.504_865_6, -24.942_75, -4.399_181_4];

pub const MRAC_AM: [[f32; 4]; 4] = [
    [0.0, 1.0, 0.0, 0.0],
    [5.475_094_3, 6.068_243_5, 42.799_32, 7.631_178],
    [0.0, 0.0, 0.0, 1.0],
    [-27.375_473, -30.341_217, -164.94661, -40.003_26],
];

pub const MRAC_PB: [f32; 4] = [-1.8973666, -2.0386274, -8.406_606, -1.395_032_3];
pub const MRAC_GAMMA_DIAG: [f32; 5] = [0.035, 0.006, 0.12, 0.012, 0.08];
pub const MRAC_SIGMA: f32 = 2.0;
pub const MRAC_NORMALIZATION_EPS: f32 = 0.1;
pub const MRAC_ADAPTATION_DELAY_SEC: f32 = 0.75;
pub const MRAC_ERROR_DEADZONE: f32 = 0.002;
pub const MRAC_MAX_ADAPTIVE_GAIN: f32 = 1.5;
pub const MRAC_MAX_ADAPTIVE_FORCE: f32 = 1.5;
pub const MRAC_ENABLE_ANGLE_LIMIT: f32 = 0.174_532_92;
pub const MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT: f32 = 3.5;
pub const MRAC_ENABLE_POSITION_LIMIT: f32 = 0.35;
pub const MRAC_ENABLE_VELOCITY_LIMIT: f32 = 1.2;
