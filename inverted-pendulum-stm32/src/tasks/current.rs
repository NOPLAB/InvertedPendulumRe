use core::cell::RefCell;
use core::sync::atomic::Ordering;

use critical_section::Mutex;
use embassy_stm32::pac;
#[allow(unused)]
use embassy_stm32::interrupt;

use crate::controller::{CurrentController, CurrentState};
use crate::driver::adc::get_currents;
use crate::driver::motor::Motors;

struct CurrentControlState {
    controller: CurrentController,
    motors: Motors,
    was_running: bool,
}

static CURRENT_CTL: Mutex<RefCell<Option<CurrentControlState>>> =
    Mutex::new(RefCell::new(None));

/// TIM6を5kHzの周期割り込みとして初期化し、電流制御を開始
pub fn init(_tim6: embassy_stm32::Peri<'static, embassy_stm32::peripherals::TIM6>, motors: Motors) {
    // Motors + CurrentController を static に移動
    critical_section::with(|cs| {
        CURRENT_CTL.borrow_ref_mut(cs).replace(CurrentControlState {
            controller: CurrentController::new(),
            motors,
            was_running: false,
        });
    });

    // TIM6 クロック有効化 (APB1)
    pac::RCC.apb1enr().modify(|w| w.set_tim6en(true));
    // リセット
    pac::RCC.apb1rstr().modify(|w| w.set_tim6rst(true));
    pac::RCC.apb1rstr().modify(|w| w.set_tim6rst(false));

    let tim = pac::TIM6;

    // APB1タイマークロック = 64MHz (APB1 prescaler=2 → timer clock = 2x)
    // 5kHz = 64MHz / (PSC+1) / (ARR+1)
    // PSC=63, ARR=199 → 64MHz / 64 / 200 = 5000Hz
    tim.psc().write_value(63);
    tim.arr().write(|w| w.set_arr(199));

    // UG (update generation) で即座にPSC/ARRをロード
    tim.egr().write(|w| w.set_ug(true));
    // UIF (update interrupt flag) をクリア
    tim.sr().write(|w| w.set_uif(false));

    // Update割り込みを有効化
    tim.dier().write(|w| w.set_uie(true));

    // NVIC で TIM6_DAC1 割り込みを有効化
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM6_DAC1);
    }

    // タイマー開始
    tim.cr1().modify(|w| w.set_cen(true));
}

/// TIM6 割り込みハンドラ — 電流制御を5kHzで実行
#[embassy_stm32::interrupt]
unsafe fn TIM6_DAC1() {
    // Update interrupt flag をクリア
    pac::TIM6.sr().write(|w| w.set_uif(false));

    critical_section::with(|cs| {
        if let Some(state) = CURRENT_CTL.borrow_ref_mut(cs).as_mut() {
            let running = crate::RUNNING.load(Ordering::Relaxed);

            if running {
                if !state.was_running {
                    state.controller.reset();
                }

                let target_current =
                    f32::from_bits(crate::TARGET_CURRENT.load(Ordering::Relaxed));

                let (current_r, current_l) = get_currents();
                let cur_state = CurrentState {
                    current_r,
                    current_l,
                    vin: 12.0,
                };

                let output = state.controller.update(target_current, &cur_state);
                state.motors.set_both(output.left, output.right);
            } else {
                state.motors.stop();
            }

            state.was_running = running;
        }
    });
}
