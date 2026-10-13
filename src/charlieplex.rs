//! Charlieplexed LEDs, scanned from the SysTick exception.
//!
//! The board describes its LEDs as one or more groups. A group is one
//! charlieplex matrix: a list of LEDs, each an (anode pin, cathode pin) pair.
//! Groups must not share pins, which lets the scanner light one LED of every
//! group in the same time slot (e.g. the three dice of an RGB LED wired as
//! three separate matrices).
//!
//! LEDs are addressed by index: the LEDs of the first group in table order,
//! then those of the second group, and so on. Put the tables in whatever
//! order suits the animations.
//!
//! Every LED gets the same on-time no matter how many are lit, and the
//! refresh does not depend on what the executor is doing.

use core::sync::atomic::{AtomicPtr, AtomicU32, AtomicUsize, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use cortex_m_rt::exception;

use crate::Port;

/// Most LEDs (over all groups) one board can have.
pub const MAX_LEDS: usize = 256;

/// One LED: lit by driving `anode` high and `cathode` low.
#[derive(Clone, Copy)]
pub struct Led {
    port: Port,
    anode: u8,
    cathode: u8,
}

impl Led {
    pub const fn new(port: Port, anode: u8, cathode: u8) -> Self {
        assert!(anode < 16 && cathode < 16 && anode != cathode);
        Self {
            port,
            anode,
            cathode,
        }
    }

    /// LED `n` of the usual full matrix on `pins` (all on `port`): each pin
    /// is the anode of `pins.len() - 1` consecutive LEDs, whose cathodes walk
    /// through the other pins in order.
    pub const fn canonical(port: Port, pins: &[u8], n: usize) -> Self {
        let per_anode = pins.len() - 1;
        let anode = n / per_anode;
        let k = n % per_anode;
        let cathode = if k >= anode { k + 1 } else { k };
        Self::new(port, pins[anode], pins[cathode])
    }

    const fn moder_mask(self) -> u32 {
        (0b11 << (2 * self.anode)) | (0b11 << (2 * self.cathode))
    }
}

static GROUPS: AtomicPtr<&'static [Led]> = AtomicPtr::new(core::ptr::null_mut());
static GROUP_COUNT: AtomicUsize = AtomicUsize::new(0);
/// LEDs in the longest group = time slots per refresh.
static SLOTS: AtomicUsize = AtomicUsize::new(0);
/// MODER bits of all matrix pins, per port.
static MODER_MASK: [AtomicU32; Port::COUNT] = [const { AtomicU32::new(0) }; Port::COUNT];

/// One bit per LED index.
static FRAME: [AtomicU32; MAX_LEDS / 32] = [const { AtomicU32::new(0) }; MAX_LEDS / 32];

const PORTS: [Port; Port::COUNT] = [Port::A, Port::B, Port::F];

/// Takes over the matrix pins and SysTick and starts scanning.
///
/// `sysclk_hz` is the core clock SysTick runs from, `slot_us` the time one
/// LED is driven. The refresh period is `slot_us` times the length of the
/// longest group; keep it under ~10ms. The pins in `groups` must not be used
/// for anything else.
pub fn init(mut syst: SYST, sysclk_hz: u32, slot_us: u32, groups: &'static [&'static [Led]]) {
    let mut total = 0;
    let mut slots = 0;
    let mut masks = [0u32; Port::COUNT];
    for group in groups {
        total += group.len();
        slots = slots.max(group.len());
        for led in group.iter() {
            masks[led.port as usize] |= led.moder_mask();
        }
    }
    assert!(total <= MAX_LEDS);

    for (mask, value) in MODER_MASK.iter().zip(masks) {
        mask.store(value, Ordering::Relaxed);
    }
    SLOTS.store(slots, Ordering::Relaxed);
    GROUP_COUNT.store(groups.len(), Ordering::Relaxed);
    GROUPS.store(groups.as_ptr() as *mut _, Ordering::Relaxed);

    blank();

    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(sysclk_hz / 1_000_000 * slot_us - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();
}

/// Stops scanning and floats all matrix pins. Do this before Stop mode.
pub fn stop() {
    // SYST itself was consumed by `init`.
    let syst = unsafe { &*SYST::PTR };
    unsafe { syst.csr.modify(|v| v & !0b11) };
    blank();
}

/// Resumes scanning after `stop`.
pub fn start() {
    let syst = unsafe { &*SYST::PTR };
    unsafe { syst.csr.modify(|v| v | 0b11) };
}

pub fn set(index: usize, on: bool) {
    let (word, bit) = (&FRAME[index / 32], 1 << (index % 32));
    // No atomic read-modify-write on Cortex-M0+.
    cortex_m::interrupt::free(|_| {
        let v = word.load(Ordering::Relaxed);
        word.store(if on { v | bit } else { v & !bit }, Ordering::Relaxed);
    });
}

pub fn clear() {
    for word in FRAME.iter() {
        word.store(0, Ordering::Relaxed);
    }
}

fn is_lit(index: usize) -> bool {
    FRAME[index / 32].load(Ordering::Relaxed) & (1 << (index % 32)) != 0
}

/// All matrix pins to analog mode: hi-Z, and no input buffer to leak current
/// while the pin floats mid-rail.
fn blank() {
    for (port, mask) in PORTS.iter().zip(MODER_MASK.iter()) {
        let mask = mask.load(Ordering::Relaxed);
        if mask != 0 {
            port.regs().moder().modify(|w| w.0 |= mask);
        }
    }
}

#[exception]
fn SysTick() {
    static mut SLOT: usize = 0;

    blank();

    let n = *SLOT;
    *SLOT = if n + 1 < SLOTS.load(Ordering::Relaxed) {
        n + 1
    } else {
        0
    };

    let groups = unsafe {
        core::slice::from_raw_parts(
            GROUPS.load(Ordering::Relaxed) as *const &'static [Led],
            GROUP_COUNT.load(Ordering::Relaxed),
        )
    };

    let mut first_index = 0;
    for group in groups {
        if n < group.len() && is_lit(first_index + n) {
            let led = group[n];
            let gpio = led.port.regs();
            gpio.bsrr()
                .write(|w| w.0 = (1 << led.anode) | (1 << (led.cathode + 16)));
            // analog (0b11) -> output (0b01)
            gpio.moder()
                .modify(|w| w.0 &= !((0b10 << (2 * led.anode)) | (0b10 << (2 * led.cathode))));
        }
        first_index += group.len();
    }
}
