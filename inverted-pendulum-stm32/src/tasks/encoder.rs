use core::sync::atomic::Ordering;

use crate::driver::encoder::Qei;

#[embassy_executor::task]
pub async fn encoder_r_task(mut qei: Qei<'static>) {
    loop {
        let result = qei.step().await;
        qei.reset();
        let prev = crate::QEI_R.load(Ordering::Relaxed);
        crate::QEI_R.store(prev + result.pulses, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
pub async fn encoder_l_task(mut qei: Qei<'static>) {
    loop {
        let result = qei.step().await;
        qei.reset();
        let prev = crate::QEI_L.load(Ordering::Relaxed);
        crate::QEI_L.store(prev + result.pulses, Ordering::Relaxed);
    }
}
