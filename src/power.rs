//! Stop mode ("off") with wake-up on a button.
//!
//! The register sequence is the one measured on a real badge with a current
//! scope. The current of a new board still has to be measured once: it
//! depends on what every pin is doing while the core sleeps, so stop the LED
//! drivers (they float their pins) before calling `stop`.

use core::ptr;

use cortex_m::peripheral::NVIC;
use py32_hal::{interrupt, pac};

use crate::Port;

const PWR_CR1: *mut u32 = 0x4000_7000 as *mut u32;
const PWR_CR1_LPR: u32 = 1 << 14;
const PWR_CR1_VOS_1V0: u32 = 1 << 9;
const PWR_CR1_SRAM_RETV_MASK: u32 = 0b111 << 16;
const PWR_CR1_SRAM_RETV_LOWEST: u32 = 0b011 << 16;

const SCB_SCR: *mut u32 = 0xE000_ED10 as *mut u32;
const SCB_SCR_SLEEPDEEP: u32 = 1 << 2;

/// Pin of the button (to ground, pulled up) that wakes the badge.
///
/// Not every pin can be routed to its EXTI line: lines 0..=4 take port A, B
/// or F, lines 5..=8 only A or B, the rest only A.
#[derive(Clone, Copy)]
pub struct WakePin {
    port: Port,
    pin: u8,
}

impl WakePin {
    pub const fn new(port: Port, pin: u8) -> Self {
        assert!(pin < 16);
        assert!(match port {
            Port::A => true,
            Port::B => pin <= 8,
            Port::F => pin <= 4,
        });
        Self { port, pin }
    }

    fn line(self) -> usize {
        self.pin as usize
    }

    fn irq(self) -> interrupt::Interrupt {
        match self.pin {
            0..=1 => interrupt::EXTI0_1,
            2..=3 => interrupt::EXTI2_3,
            _ => interrupt::EXTI4_15,
        }
    }

    fn is_low(self) -> bool {
        self.port.regs().idr().read().0 & (1 << self.pin) == 0
    }

    fn clear_irq(self) {
        pac::EXTI.pr().write(|w| w.set_line(self.line(), true));
        NVIC::unpend(self.irq());
    }

    /// Falling edge on the pin -> interrupt. The HAL's EXTI handler masks the
    /// line again when it fires, so this is redone before every sleep.
    fn arm(self) {
        cortex_m::interrupt::free(|_| {
            let exti = pac::EXTI;
            let source = match self.port {
                Port::A => 0,
                Port::B => 1,
                Port::F => 2,
            };

            exti.exticr(self.line() / 4)
                .modify(|w| w.set_exti(self.line() % 4, source));
            exti.rtsr().modify(|w| w.set_line(self.line(), false));
            exti.ftsr().modify(|w| w.set_line(self.line(), true));
            exti.pr().write(|w| w.set_line(self.line(), true));
            exti.imr().modify(|w| w.set_line(self.line(), true));

            unsafe { NVIC::unmask(self.irq()) };
        });
    }
}

/// Sleeps in Stop mode until the wake button is pressed.
///
/// Returns with the button still held down; wait for its release before
/// polling it again. SWD is dead while in Stop mode.
pub fn stop(wake: WakePin) {
    loop {
        wake.clear_irq();
        wake.arm();

        cortex_m::interrupt::free(|_| unsafe {
            pac::RCC.apbenr1().modify(|w| w.set_pwren(true));

            let mut pwr_cr1 = ptr::read_volatile(PWR_CR1);
            pwr_cr1 &= !PWR_CR1_SRAM_RETV_MASK;
            pwr_cr1 |= PWR_CR1_LPR | PWR_CR1_VOS_1V0 | PWR_CR1_SRAM_RETV_LOWEST;
            ptr::write_volatile(PWR_CR1, pwr_cr1);

            ptr::write_volatile(SCB_SCR, ptr::read_volatile(SCB_SCR) | SCB_SCR_SLEEPDEEP);
        });

        cortex_m::asm::dsb();
        cortex_m::asm::wfi();
        cortex_m::asm::isb();

        unsafe {
            ptr::write_volatile(SCB_SCR, ptr::read_volatile(SCB_SCR) & !SCB_SCR_SLEEPDEEP);
        }

        wake.clear_irq();

        // Any other interrupt that was still pending ends the WFI as well.
        if wake.is_low() {
            return;
        }
    }
}
