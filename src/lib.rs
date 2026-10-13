//! Common firmware parts for coin cell LED badges built on PY32 MCUs:
//! a button or two, a lot of LEDs, off via button, Stop mode in between.
//!
//! The board crate owns everything board specific: the chip (py32-hal
//! feature and `memory.x`), the pin tables and the animations.
#![no_std]

use py32_hal::pac;

#[cfg(feature = "battery")]
pub mod battery;
pub mod button;
#[cfg(feature = "charlieplex")]
pub mod charlieplex;
pub mod font;
pub mod power;
#[cfg(feature = "soft-pwm")]
pub mod soft_pwm;

/// GPIO port of a pin given by number rather than by HAL type.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Port {
    A,
    B,
    F,
}

impl Port {
    const COUNT: usize = 3;

    fn regs(self) -> pac::gpio::Gpio {
        match self {
            Port::A => pac::GPIOA,
            Port::B => pac::GPIOB,
            Port::F => pac::GPIOF,
        }
    }
}
