use crate::constants::clamp;

/// PID controller with anti-windup
pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    dt: f32,
    min_output: f32,
    max_output: f32,
    integral: f32,
    prev_error: f32,
    first_call: bool,
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32, dt: f32, min_output: f32, max_output: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            dt,
            min_output,
            max_output,
            integral: 0.0,
            prev_error: 0.0,
            first_call: true,
        }
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32) -> f32 {
        self.update_error(setpoint - measurement)
    }

    pub fn update_error(&mut self, error: f32) -> f32 {
        let p = self.kp * error;
        self.integral += self.ki * error * self.dt;

        let d = if self.first_call {
            self.first_call = false;
            0.0
        } else {
            self.kd * (error - self.prev_error) / self.dt
        };

        let unclamped = p + self.integral + d;
        let output = clamp(unclamped, self.min_output, self.max_output);

        // Anti-windup
        if unclamped > self.max_output && self.integral > 0.0 {
            self.integral = (self.max_output - p - d).max(0.0);
        } else if unclamped < self.min_output && self.integral < 0.0 {
            self.integral = (self.min_output - p - d).min(0.0);
        }

        self.prev_error = error;
        output
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_call = true;
    }
}
