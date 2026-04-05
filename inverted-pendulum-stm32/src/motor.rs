use embassy_stm32::peripherals;
use embassy_stm32::timer::simple_pwm::SimplePwm;

pub struct Motors {
    right: SimplePwm<'static, peripherals::TIM1>,
    left_fwd: SimplePwm<'static, peripherals::TIM2>,
    left_rev: SimplePwm<'static, peripherals::TIM3>,
}

impl Motors {
    pub fn new(
        mut right: SimplePwm<'static, peripherals::TIM1>,
        mut left_fwd: SimplePwm<'static, peripherals::TIM2>,
        mut left_rev: SimplePwm<'static, peripherals::TIM3>,
    ) -> Self {
        right.ch3().enable();
        right.ch4().enable();
        left_fwd.ch2().enable();
        left_rev.ch4().enable();

        Self {
            right,
            left_fwd,
            left_rev,
        }
    }

    pub fn set_left(&mut self, duty: f32) {
        let d = duty.clamp(-1.0, 1.0);
        let pct = (d.abs() * 100.0) as u8;

        if d > 0.0 {
            self.left_rev.ch4().set_duty_cycle_percent(0);
            self.left_fwd.ch2().set_duty_cycle_percent(pct);
        } else if d < 0.0 {
            self.left_fwd.ch2().set_duty_cycle_percent(0);
            self.left_rev.ch4().set_duty_cycle_percent(pct);
        } else {
            self.left_fwd.ch2().set_duty_cycle_percent(0);
            self.left_rev.ch4().set_duty_cycle_percent(0);
        }
    }

    pub fn set_right(&mut self, duty: f32) {
        let d = duty.clamp(-1.0, 1.0);
        let pct = (d.abs() * 100.0) as u8;

        if d > 0.0 {
            self.right.ch4().set_duty_cycle_percent(0);
            self.right.ch3().set_duty_cycle_percent(pct);
        } else if d < 0.0 {
            self.right.ch3().set_duty_cycle_percent(0);
            self.right.ch4().set_duty_cycle_percent(pct);
        } else {
            self.right.ch3().set_duty_cycle_percent(0);
            self.right.ch4().set_duty_cycle_percent(0);
        }
    }

    pub fn set_both(&mut self, left: f32, right: f32) {
        self.set_left(left);
        self.set_right(right);
    }

    pub fn stop(&mut self) {
        self.set_both(0.0, 0.0);
    }
}
