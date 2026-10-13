//! Starter board firmware: 12 charlieplexed LEDs on PA0..PA3, one button on
//! PB0. Copy this directory next to a board's KiCad project and adapt the
//! "Board" section and the animations. See ../AGENTS.md.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use cortex_m::peripheral::SCB;
use cortex_m_rt::{exception, ExceptionFrame};
use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};

use py32_badgekit::button::Button;
use py32_badgekit::charlieplex::{self, Led};
use py32_badgekit::power::{self, WakePin};
use py32_badgekit::Port;
use py32_hal::gpio::{Input, Pull};
use py32_hal::rcc::Sysclk;
use py32_hal::time::Hertz;
use {defmt_rtt as _, panic_halt as _};

// ---- Board -----------------------------------------------------------------

const SYSCLK_HZ: u32 = 8_000_000;

/// 12 slots * 500us = 6ms -> ~167Hz refresh.
const SLOT_US: u32 = 500;

/// Charlieplex pins on port A.
const LED_PINS: [u8; 4] = [0, 1, 2, 3];
const LED_COUNT: usize = 12;

/// Index = position of the LED along the ring.
static LEDS: [Led; LED_COUNT] = {
    let mut leds = [Led::new(Port::A, 0, 1); LED_COUNT];
    let mut i = 0;
    while i < LED_COUNT {
        leds[i] = Led::canonical(Port::A, &LED_PINS, i);
        i += 1;
    }
    leds
};
static LED_GROUPS: [&[Led]; 1] = [&LEDS];

/// Button to ground on PB0.
const WAKE: WakePin = WakePin::new(Port::B, 0);

// ---- Behaviour -------------------------------------------------------------

/// The badge turns itself off after this long without a button press.
const AWAKE_TIMEOUT: Duration = Duration::from_secs(10 * 60);

const CHASE_STEP: Duration = Duration::from_millis(80);

/// Runs until the button is pressed or the badge has been on long enough.
async fn animate(button: &mut Button<'_>) {
    let deadline = Instant::now() + AWAKE_TIMEOUT;

    let mut i = 0;
    while Instant::now() < deadline {
        charlieplex::clear();
        charlieplex::set(i, true);
        i = (i + 1) % LED_COUNT;

        if button.pressed_within(CHASE_STEP).await {
            return;
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Recovery window: stay awake with every pin in its reset state for a
    // second or two (HSI reset default, before any clock setup), so a probe
    // can always attach right after power-up even if the code below ever
    // breaks SWD or sleeps too deep. Keep this first.
    cortex_m::asm::delay(8_000_000);

    let mut cfg: py32_hal::Config = Default::default();
    cfg.rcc.hsi = Some(Hertz::mhz(SYSCLK_HZ / 1_000_000));
    cfg.rcc.sys = Sysclk::HSI;
    let p = py32_hal::init(cfg);

    info!("Hello World!");

    // PA13/PA14 (SWD) and PF2 (nRST) are left alone on purpose.
    let syst = cortex_m::Peripherals::take().unwrap().SYST;
    charlieplex::init(syst, SYSCLK_HZ, SLOT_US, &LED_GROUPS);

    let mut button = Button::new(Input::new(p.PB0, Pull::Up));

    loop {
        animate(&mut button).await;

        info!("off");
        charlieplex::stop();
        button.wait_release().await;
        power::stop(WAKE);
        button.wait_release().await;
        info!("on");
        charlieplex::start();
    }
}

//conserve flash space with the non-handler
#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    SCB::sys_reset()
}
