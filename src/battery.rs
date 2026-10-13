//! Supply (coin cell) voltage, measured against the internal reference.

use py32_hal::adc::{Adc, SampleTime};
use py32_hal::peripherals::ADC;
use py32_hal::{adc, bind_interrupts};

bind_interrupts!(struct Irqs {
    ADC_COMP => adc::InterruptHandler<ADC>;
});

const VREFINT_MV: u32 = 1200;

pub async fn measure_vcc_mv(p_adc: &mut ADC) -> u32 {
    let mut adc = Adc::new(p_adc, Irqs);
    adc.set_sample_time(SampleTime::CYCLES71_5);

    let mut vrefint = adc.enable_vref();
    let sample: u32 = adc.read(&mut vrefint).await.into();
    VREFINT_MV * 4095 / sample.max(1)
}

/// Linear interpolation between `at_empty` and `at_full` by where `vcc_mv`
/// sits between `empty_mv` and `full_mv`. LEDs driven without series
/// resistors get much brighter on a fresh cell; use this to even that out.
pub fn scale_for_vcc(vcc_mv: u32, empty_mv: u32, full_mv: u32, at_empty: u16, at_full: u16) -> u16 {
    let vcc_mv = vcc_mv.clamp(empty_mv, full_mv);
    let span = (full_mv - empty_mv) as i32;
    let pos = (vcc_mv - empty_mv) as i32;
    let (empty, full) = (at_empty as i32, at_full as i32);

    (empty + (full - empty) * pos / span) as u16
}
