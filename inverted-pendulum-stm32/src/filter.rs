use crate::constants::lpf_alpha;

/// First-order low-pass filter
#[derive(Debug, Clone, Copy)]
pub struct LowPassFilter {
    alpha: f32,
    output: f32,
    initialized: bool,
}

impl LowPassFilter {
    pub fn new(sample_time: f32, cutoff_freq: f32) -> Self {
        Self {
            alpha: lpf_alpha(cutoff_freq, sample_time),
            output: 0.0,
            initialized: false,
        }
    }

    /// y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    pub fn update(&mut self, input: f32) -> f32 {
        if !self.initialized {
            self.output = input;
            self.initialized = true;
        } else {
            self.output = self.alpha * input + (1.0 - self.alpha) * self.output;
        }
        self.output
    }

    pub fn reset(&mut self) {
        self.output = 0.0;
        self.initialized = false;
    }
}
