use crate::config::*;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use embassy_stm32::{
    adc::{Adc, SampleTime},
    peripherals, Peri,
};

pub struct AdcSensors {
    adc1: Adc<'static, peripherals::ADC1>,
    adc2: Adc<'static, peripherals::ADC2>,
}

impl AdcSensors {
    pub fn new(
        adc1: Adc<'static, peripherals::ADC1>,
        adc2: Adc<'static, peripherals::ADC2>,
    ) -> Self {
        Self { adc1, adc2 }
    }

    pub async fn read(
        &mut self,
        theta_pin: &mut Peri<'static, peripherals::PB0>,
        vin_pin: &mut Peri<'static, peripherals::PA0>,
        current_r_pin: &mut Peri<'static, peripherals::PA5>,
        current_l_pin: &mut Peri<'static, peripherals::PA7>,
    ) {
        let theta_raw = self.adc1.read(theta_pin, SampleTime::CYCLES601_5).await;
        let vin_raw = self.adc1.read(vin_pin, SampleTime::CYCLES601_5).await;
        let current_r_raw = self.adc2.read(current_r_pin, SampleTime::CYCLES601_5).await;
        let current_l_raw = self.adc2.read(current_l_pin, SampleTime::CYCLES601_5).await;

        let packed = ((theta_raw as u32) << 16) | (current_r_raw as u32);
        ADC_DATA_HIGH.store(packed, Ordering::Relaxed);
        ADC_DATA_LOW.store(current_l_raw as u32, Ordering::Relaxed);
        ADC_VIN.store(vin_raw, Ordering::Relaxed);
    }
}

// Atomic storage for lock-free ADC access
static ADC_DATA_HIGH: AtomicU32 = AtomicU32::new(0); // theta_raw | current_r_raw
static ADC_DATA_LOW: AtomicU32 = AtomicU32::new(0); // current_l_raw
static ADC_VIN: AtomicU16 = AtomicU16::new(0); // 電源電圧ADC値
static THETA_OFFSET: AtomicU16 = AtomicU16::new(0);
static CURRENT_R_OFFSET: AtomicU16 = AtomicU16::new(0);
static CURRENT_L_OFFSET: AtomicU16 = AtomicU16::new(0);

pub fn calibrate_theta() {
    let high = ADC_DATA_HIGH.load(Ordering::Relaxed);
    THETA_OFFSET.store((high >> 16) as u16, Ordering::Relaxed);
}

#[allow(dead_code)]
pub fn calibrate_current() {
    let high = ADC_DATA_HIGH.load(Ordering::Relaxed);
    let low = ADC_DATA_LOW.load(Ordering::Relaxed);
    CURRENT_R_OFFSET.store(high as u16, Ordering::Relaxed);
    CURRENT_L_OFFSET.store(low as u16, Ordering::Relaxed);
}

pub fn get_theta() -> f32 {
    let high = ADC_DATA_HIGH.load(Ordering::Relaxed);
    let raw = (high >> 16) as u16;
    let offset = THETA_OFFSET.load(Ordering::Relaxed);
    adc_to_radians(raw, offset)
}

pub fn get_vin() -> f32 {
    let raw = ADC_VIN.load(Ordering::Relaxed);
    adc_to_vin(raw)
}

pub fn get_currents() -> (f32, f32) {
    let high = ADC_DATA_HIGH.load(Ordering::Relaxed);
    let low = ADC_DATA_LOW.load(Ordering::Relaxed);
    let r_offset = CURRENT_R_OFFSET.load(Ordering::Relaxed);
    let l_offset = CURRENT_L_OFFSET.load(Ordering::Relaxed);

    let current_r = adc_to_current_with_offset(high as u16, r_offset);
    let current_l = adc_to_current_with_offset(low as u16, l_offset);
    (current_r, current_l)
}

#[embassy_executor::task]
pub async fn adc_task(
    mut sensors: AdcSensors,
    mut theta_pin: Peri<'static, peripherals::PB0>,
    mut vin_pin: Peri<'static, peripherals::PA0>,
    mut current_r_pin: Peri<'static, peripherals::PA5>,
    mut current_l_pin: Peri<'static, peripherals::PA7>,
) -> ! {
    loop {
        sensors
            .read(&mut theta_pin, &mut vin_pin, &mut current_r_pin, &mut current_l_pin)
            .await;
    }
}
