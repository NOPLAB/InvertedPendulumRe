use crate::constants::FORCE_TO_CURRENT;

/// 離散時間Luenberger observer
/// 状態: [x, x_dot, theta, theta_dot]
/// 出力: [x, theta]
pub struct StateObserver {
    x_hat: [f32; 4],
    prev_force: f32,
    initialized: bool,
}

pub struct ObserverEstimate {
    pub velocity: f32,
    pub theta_dot: f32,
}

impl StateObserver {
    pub fn new() -> Self {
        Self {
            x_hat: [0.0; 4],
            prev_force: 0.0,
            initialized: false,
        }
    }

    pub fn update(&mut self, position: f32, theta: f32, target_current: f32) -> ObserverEstimate {
        let applied_force = target_current / FORCE_TO_CURRENT;

        if !self.initialized {
            self.x_hat = [position, 0.0, theta, 0.0];
            self.prev_force = applied_force;
            self.initialized = true;
            return ObserverEstimate {
                velocity: 0.0,
                theta_dot: 0.0,
            };
        }

        let innovation = [position - self.x_hat[0], theta - self.x_hat[2]];
        let prev = self.x_hat;
        let mut next = [0.0; 4];

        for i in 0..4 {
            next[i] = AD[i][0] * prev[0]
                + AD[i][1] * prev[1]
                + AD[i][2] * prev[2]
                + AD[i][3] * prev[3]
                + BD[i] * self.prev_force
                + LD[i][0] * innovation[0]
                + LD[i][1] * innovation[1];
        }

        self.x_hat = next;
        self.prev_force = applied_force;

        ObserverEstimate {
            velocity: self.x_hat[1],
            theta_dot: self.x_hat[3],
        }
    }

    pub fn reset(&mut self) {
        self.x_hat = [0.0; 4];
        self.prev_force = 0.0;
        self.initialized = false;
    }
}

// Designed in MATLAB/control/design_observer.m
const AD: [[f32; 4]; 4] = [
    [1.0, 0.001, -1.92867233e-7, 7.19966164e-9],
    [0.0, 1.0, -3.85612705e-4, 1.43304687e-5],
    [0.0, 0.0, 1.00002547, 9.99049074e-4],
    [0.0, 0.0, 5.09314206e-2, 9.98107243e-1],
];

const BD: [f32; 4] = [8.65667613e-7, 1.73131455e-3, -4.32569161e-6, -8.64865233e-3];

const LD: [[f32; 2]; 4] = [
    [6.51586527e-2, 5.20789139e-3],
    [1.0445936, 1.67298784e-1],
    [4.21340529e-3, 6.08237631e-2],
    [1.25858405e-1, 9.00159305e-1],
];
