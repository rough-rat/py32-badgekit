//! Charlieplexed LEDs with a brightness per LED, scanned from the SysTick
//! exception.
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
//! Brightness is PWM within the slot: a refresh gives every slot the same
//! window, the slot's LEDs (at most one per group) go on at its start, and
//! each goes off again after its own on-time, to the core cycle. Off-times
//! close to the previous switch are waited for inside the interrupt, the
//! others get an interrupt of their own, so a slot costs 1 + (groups)
//! interrupts at most. The resolution is the window in cycles (~12 bits at
//! 48MHz, 42 slots, 200Hz), linear in light output. `set_level` takes a
//! perceived level instead and maps it through the CIE 1931 lightness curve.
//!
//! (HUB75 panels use bit planes instead because a shift register cannot
//! time single pixels. With a handful of LEDs per slot, timing each one is
//! cheaper and finer.)
//!
//! Drawing goes to a canvas. `present` compiles it into the slot plans the
//! interrupt follows and swaps them in at the end of a refresh, so a refresh
//! never mixes two frames.
//!
//! The scan runs at the highest priority (from RAM with "scan-in-ram"):
//! `init` moves every other interrupt one level down. It switches at a fixed
//! point after each counter wrap, which hides the interrupt entry jitter.
//! The PendSV exception is taken as well, to wake `show` at low priority.
//! Anything that keeps interrupts masked for longer (critical sections, e.g.
//! defmt logging) still delays it and shows as a brightness glitch.
//!
//! The SysTick counter doubles as a cycle counter (`cycles`), and the scan
//! interrupt keeps count of its own cost (`stats`), for profiling.

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU16, AtomicU32, AtomicUsize, Ordering};
use core::task::Poll;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::{NVIC, SCB, SYST};
use cortex_m_rt::exception;
use embassy_sync::waitqueue::AtomicWaker;

use crate::Port;

/// Most groups, i.e. LEDs lit at the same time.
pub const MAX_GROUPS: usize = 4;

/// Cycles from the counter wrap to the switch. Covers the interrupt entry,
/// whose length varies, so every switch lands at the same point.
pub const SWITCH_AT_CYCLES: u32 = 112;

/// Shortest time between two switches done by separate interrupts: the
/// first interrupt has to be done before the second one fires. Switches
/// closer together are waited for within one interrupt. Has to cover
/// `SWITCH_AT_CYCLES`, `Stats::tail_max_cycles` (~400 on a PY32F030 at
/// 48MHz) and a late last switch-off (~150); check `Stats::overruns` on the
/// real board.
pub const MIN_GAP_CYCLES: u32 = 960;

/// One LED: lit by driving `anode` high and `cathode` low. Holds the
/// register bits for that, so the scan does no bit fiddling.
#[derive(Clone, Copy)]
pub struct Led {
    port: Port,
    /// Anode high, cathode low.
    bsrr: u32,
    /// MODER bits to clear to turn both pins from analog (0b11) to output
    /// (0b01).
    outputs: u32,
}

impl Led {
    pub const fn new(port: Port, anode: u8, cathode: u8) -> Self {
        assert!(anode < 16 && cathode < 16 && anode != cathode);
        Self {
            port,
            bsrr: (1 << anode) | (1 << (cathode + 16)),
            outputs: (0b10 << (2 * anode)) | (0b10 << (2 * cathode)),
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

    /// MODER bits of both pins.
    const fn moder_mask(self) -> u32 {
        self.outputs | (self.outputs >> 1)
    }
}

/// What the interrupt does in one slot, compiled by `present`. All zero is
/// a dark slot.
///
/// Wake 0 is the slot start: it switches the `on` groups on (and the last
/// slot's LEDs off). Actions turn groups off at `time` cycles after that,
/// in time order. Each further wake `j` (`1..=extra_wakes`) starts at
/// action `wake_at[j - 1]`; every action up to the next wake's is waited
/// for within the same interrupt.
#[derive(Clone, Copy)]
struct Slot {
    on: u8,
    actions: u8,
    extra_wakes: u8,
    time: [u16; MAX_GROUPS],
    off: [u8; MAX_GROUPS],
    wake_at: [u8; MAX_GROUPS],
}

const DARK: Slot = Slot {
    on: 0,
    actions: 0,
    extra_wakes: 0,
    time: [0; MAX_GROUPS],
    off: [0; MAX_GROUPS],
    wake_at: [0; MAX_GROUPS],
};

impl Slot {
    #[inline(always)]
    fn wakes(&self) -> usize {
        1 + self.extra_wakes as usize
    }

    /// Index of the first action wake `j` does.
    #[inline(always)]
    fn first_action(&self, j: usize) -> usize {
        if j == 0 {
            0
        } else {
            self.wake_at[j - 1] as usize
        }
    }

    /// Cycles from the slot start to wake `j`.
    #[inline(always)]
    fn wake_time(&self, j: usize) -> u32 {
        if j == 0 {
            0
        } else {
            self.time[self.wake_at[j - 1] as usize] as u32
        }
    }
}

/// The scan's memory, sized by the board: `LEDS` at least the LED count,
/// `SLOTS` at least the longest group. Put it in a `static`.
pub struct Buffers<const LEDS: usize, const SLOTS: usize> {
    /// On-time per LED index in cycles, `0..=max_raw()`. Drawn into by the
    /// thread, read by `present`.
    levels: [AtomicU16; LEDS],
    /// Front and back plan. The interrupt reads only the front one,
    /// `prepare` writes only the back one, and they change hands through
    /// SWAP/FRONT.
    plans: UnsafeCell<[[Slot; SLOTS]; 2]>,
}

unsafe impl<const LEDS: usize, const SLOTS: usize> Sync for Buffers<LEDS, SLOTS> {}

impl<const LEDS: usize, const SLOTS: usize> Buffers<LEDS, SLOTS> {
    pub const fn new() -> Self {
        Self {
            levels: [const { AtomicU16::new(0) }; LEDS],
            plans: UnsafeCell::new([[DARK; SLOTS]; 2]),
        }
    }
}

impl<const LEDS: usize, const SLOTS: usize> Default for Buffers<LEDS, SLOTS> {
    fn default() -> Self {
        Self::new()
    }
}

static LEVELS: AtomicPtr<AtomicU16> = AtomicPtr::new(core::ptr::null_mut());
static LED_COUNT: AtomicUsize = AtomicUsize::new(0);
static PLANS: [AtomicPtr<Slot>; 2] = [const { AtomicPtr::new(core::ptr::null_mut()) }; 2];
/// Index of the plan on show.
static FRONT: AtomicUsize = AtomicUsize::new(0);
/// Set by `show`, cleared by the interrupt once it has swapped.
static SWAP: AtomicBool = AtomicBool::new(false);
static SWAPPED: AtomicWaker = AtomicWaker::new();

fn levels() -> &'static [AtomicU16] {
    unsafe {
        core::slice::from_raw_parts(
            LEVELS.load(Ordering::Relaxed),
            LED_COUNT.load(Ordering::Relaxed),
        )
    }
}

/// Plan `i` (0 or 1), one entry per slot.
#[inline(always)]
fn plan(i: usize) -> *mut Slot {
    PLANS[i & 1].load(Ordering::Relaxed)
}

static GROUPS: AtomicPtr<&'static [Led]> = AtomicPtr::new(core::ptr::null_mut());
static GROUP_COUNT: AtomicUsize = AtomicUsize::new(0);
/// LEDs in the longest group = time slots per refresh.
static SLOTS: AtomicUsize = AtomicUsize::new(0);
/// MODER bits of all matrix pins, per port.
static MODER_MASK: [AtomicU32; Port::COUNT] = [const { AtomicU32::new(0) }; Port::COUNT];
/// Slot length in cycles.
static WINDOW: AtomicU32 = AtomicU32::new(0);

// Profiling. Only the interrupt writes these (and `init`, before it runs).
/// Cycles of all SysTick periods that have ended.
static ELAPSED: AtomicU32 = AtomicU32::new(0);
/// Length of the running SysTick period.
static CUR_LEN: AtomicU32 = AtomicU32::new(0);
/// Length of the period the counter reloads with at the next wrap.
static NEXT_LEN: AtomicU32 = AtomicU32::new(0);
static ISR_COUNT: AtomicU32 = AtomicU32::new(0);
static ISR_CYCLES: AtomicU32 = AtomicU32::new(0);
static ISR_MAX: AtomicU32 = AtomicU32::new(0);
static TAIL_MAX: AtomicU32 = AtomicU32::new(0);
static OVERRUNS: AtomicU32 = AtomicU32::new(0);

const PORTS: [Port; Port::COUNT] = [Port::A, Port::B, Port::F];

/// Takes over the matrix pins and SysTick and starts scanning.
///
/// `sysclk_hz` is the core clock SysTick runs from, `refresh_hz` how often
/// every LED gets its turn (keep it over ~100Hz; higher costs resolution
/// and interrupt time). The pins in `groups` must not be used for anything
/// else.
pub fn init<const L: usize, const S: usize>(
    mut syst: SYST,
    sysclk_hz: u32,
    refresh_hz: u32,
    groups: &'static [&'static [Led]],
    buffers: &'static Buffers<L, S>,
) {
    assert!(groups.len() <= MAX_GROUPS);

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
    assert!(total <= L && slots <= S, "buffers too small");

    let window = sysclk_hz / refresh_hz / slots as u32;
    assert!(window >= 2 * MIN_GAP_CYCLES, "refresh too high");
    assert!(window <= u16::MAX as u32, "refresh too low");

    for (mask, value) in MODER_MASK.iter().zip(masks) {
        mask.store(value, Ordering::Relaxed);
    }
    SLOTS.store(slots, Ordering::Relaxed);
    GROUP_COUNT.store(groups.len(), Ordering::Relaxed);
    GROUPS.store(groups.as_ptr() as *mut _, Ordering::Relaxed);
    WINDOW.store(window, Ordering::Relaxed);
    LEVELS.store(buffers.levels.as_ptr() as *mut _, Ordering::Relaxed);
    LED_COUNT.store(total, Ordering::Relaxed);
    let plans = buffers.plans.get() as *mut [Slot; S];
    for (i, ptr) in PLANS.iter().enumerate() {
        ptr.store(unsafe { (*plans.add(i)).as_mut_ptr() }, Ordering::Relaxed);
    }

    // Nothing may delay the scan: everything else one priority level down.
    unsafe {
        let nvic = &*NVIC::PTR;
        for ipr in nvic.ipr.iter() {
            ipr.write(0x4040_4040);
        }
        // SHPR3: SysTick is the top byte, at the top priority. PendSV (`show`'s
        // wake-up) the byte below it, at the bottom one.
        let shpr3 = 0xE000_ED20 as *mut u32;
        shpr3.write_volatile((shpr3.read_volatile() & 0x0000_FFFF) | 0x00C0_0000);
    }

    // The plans start out dark: one wake per slot.
    CUR_LEN.store(window, Ordering::Relaxed);
    NEXT_LEN.store(window, Ordering::Relaxed);

    blank();

    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(window - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();
}

/// What `init` made of the requested refresh.
#[derive(Clone, Copy)]
pub struct Timing {
    /// One slot, in core cycles.
    pub window_cycles: u32,
    /// Longest on-time = `max_raw()`, i.e. the brightness resolution.
    pub max_on_cycles: u32,
    /// One refresh, in core cycles.
    pub refresh_cycles: u32,
}

pub fn timing() -> Timing {
    let window = WINDOW.load(Ordering::Relaxed);
    Timing {
        window_cycles: window,
        max_on_cycles: max_raw() as u32,
        refresh_cycles: window * SLOTS.load(Ordering::Relaxed) as u32,
    }
}

fn syst() -> &'static cortex_m::peripheral::syst::RegisterBlock {
    // SYST itself was consumed by `init`.
    unsafe { &*SYST::PTR }
}

fn scanning() -> bool {
    syst().csr.read() & 1 != 0
}

#[inline(always)]
fn groups() -> &'static [&'static [Led]] {
    unsafe {
        core::slice::from_raw_parts(
            GROUPS.load(Ordering::Relaxed) as *const &'static [Led],
            GROUP_COUNT.load(Ordering::Relaxed),
        )
    }
}

/// Stops scanning and floats all matrix pins. Do this before Stop mode.
pub fn stop() {
    unsafe { syst().csr.modify(|v| v & !0b11) };
    blank();
}

/// Resumes scanning after `stop`.
pub fn start() {
    unsafe { syst().csr.modify(|v| v | 0b11) };
}

/// Longest on-time in cycles: the slot minus the time the last switch-off
/// needs before the next slot starts.
pub fn max_raw() -> u16 {
    WINDOW.load(Ordering::Relaxed).saturating_sub(MIN_GAP_CYCLES) as u16
}

/// Sets the on-time of one LED in cycles (linear brightness), clamped to
/// `max_raw()`.
pub fn set_raw(index: usize, raw: u16) {
    levels()[index].store(raw.min(max_raw()), Ordering::Relaxed);
}

/// Sets the perceived brightness of one LED, 0..=255. Every level above 0
/// stays at least dimly lit.
pub fn set_level(index: usize, level: u8) {
    let max = max_raw() as u32;
    let raw = (LIGHTNESS[level as usize] as u32 * max + 0x8000) >> 16;
    let raw = if level > 0 { raw.max(1) } else { 0 };
    levels()[index].store(raw as u16, Ordering::Relaxed);
}

/// Full brightness or off.
pub fn set(index: usize, on: bool) {
    set_raw(index, if on { u16::MAX } else { 0 });
}

/// Turns every LED off.
pub fn clear() {
    for level in levels().iter() {
        level.store(0, Ordering::Relaxed);
    }
}

/// Shows what was drawn from the next refresh on, and waits for that (at
/// most one refresh). The drawing stays as it is, so it can go on from
/// there. Same as `prepare` then `show`.
pub async fn present() {
    prepare();
    show().await;
}

/// Compiles what was drawn into the plan `show` swaps in. Does not wait.
pub fn prepare() {
    let back = FRONT.load(Ordering::Relaxed) ^ 1;
    let slots = SLOTS.load(Ordering::Relaxed);
    // Only the interrupt reads plans, and only the front one.
    compile(unsafe { core::slice::from_raw_parts_mut(plan(back), slots) });
}

/// Shows the plan from the last `prepare` from the next refresh on, and
/// waits for that.
pub async fn show() {
    let back = FRONT.load(Ordering::Relaxed) ^ 1;
    if scanning() {
        // Release: the plan is written before the interrupt can see SWAP.
        SWAP.store(true, Ordering::Release);
        poll_fn(|cx| {
            SWAPPED.register(cx.waker());
            if SWAP.load(Ordering::Acquire) {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    } else {
        FRONT.store(back, Ordering::Release);
    }
}

/// Works out each slot's switches from LEVELS.
fn compile(plan: &mut [Slot]) {
    let groups = groups();
    let levels = levels();
    for (n, slot) in plan.iter_mut().enumerate() {
        *slot = DARK;

        // (on-time, group bit) of the lit LEDs, sorted by on-time.
        let mut lit = [(0u16, 0u8); MAX_GROUPS];
        let mut count = 0;
        let mut first = 0;
        for (g, group) in groups.iter().enumerate() {
            if n < group.len() {
                let time = levels[first + n].load(Ordering::Relaxed);
                if time > 0 {
                    slot.on |= 1 << g;
                    lit[count] = (time, 1 << g);
                    count += 1;
                }
            }
            first += group.len();
        }
        lit[..count].sort_unstable_by_key(|&(time, _)| time);

        let mut prev = 0;
        for &(time, bit) in &lit[..count] {
            let a = slot.actions as usize;
            if a > 0 && slot.time[a - 1] == time {
                slot.off[a - 1] |= bit;
                continue;
            }
            slot.time[a] = time;
            slot.off[a] = bit;
            slot.actions += 1;
            if (time - prev) as u32 >= MIN_GAP_CYCLES {
                slot.wake_at[slot.extra_wakes as usize] = a as u8;
                slot.extra_wakes += 1;
            }
            prev = time;
        }
    }
}

/// Core cycles counted while scanning, wrapping. Take differences.
pub fn cycles() -> u32 {
    cortex_m::interrupt::free(|_| {
        let syst = syst();
        let elapsed = ELAPSED.load(Ordering::Relaxed);
        let cur = CUR_LEN.load(Ordering::Relaxed);
        let val = syst.cvr.read();
        if SCB::is_pendst_pending() {
            // Wrapped, but the interrupt is held off until we are done.
            let val = syst.cvr.read();
            elapsed
                .wrapping_add(cur)
                .wrapping_add(NEXT_LEN.load(Ordering::Relaxed) - 1 - val)
        } else {
            elapsed.wrapping_add(cur - 1 - val)
        }
    })
}

/// Cost of the scan interrupt so far. Totals wrap; take differences.
#[derive(Clone, Copy, Default)]
pub struct Stats {
    /// Interrupts taken.
    pub count: u32,
    /// Cycles spent in them, from the counter wrap to the end of the handler
    /// (exception entry included, exit not). Overruns are not counted here.
    pub cycles: u32,
    /// Longest single interrupt.
    pub max_cycles: u32,
    /// Longest time from an interrupt's last switch to its end (exit not
    /// included). `MIN_GAP_CYCLES` has to cover this plus
    /// `SWITCH_AT_CYCLES`.
    pub tail_max_cycles: u32,
    /// Interrupts that ran past the next counter wrap.
    pub overruns: u32,
}

pub fn stats() -> Stats {
    cortex_m::interrupt::free(|_| Stats {
        count: ISR_COUNT.load(Ordering::Relaxed),
        cycles: ISR_CYCLES.load(Ordering::Relaxed),
        max_cycles: ISR_MAX.load(Ordering::Relaxed),
        tail_max_cycles: TAIL_MAX.load(Ordering::Relaxed),
        overruns: OVERRUNS.load(Ordering::Relaxed),
    })
}

/// All matrix pins to analog mode: hi-Z, and no input buffer to leak current
/// while the pin floats mid-rail.
#[inline(always)]
fn blank() {
    for (port, mask) in PORTS.iter().zip(MODER_MASK.iter()) {
        let mask = mask.load(Ordering::Relaxed);
        if mask != 0 {
            port.regs().moder().modify(|w| w.0 |= mask);
        }
    }
}

/// Register writes for one switch, worked out ahead so the switch itself is
/// only the writes: per port, BSRR, then MODER with `set` bits set and
/// `clear` bits cleared.
#[derive(Clone, Copy)]
struct Switch {
    bsrr: [u32; Port::COUNT],
    set: [u32; Port::COUNT],
    clear: [u32; Port::COUNT],
}

impl Switch {
    const NONE: Self = Self {
        bsrr: [0; Port::COUNT],
        set: [0; Port::COUNT],
        clear: [0; Port::COUNT],
    };

    // Filled in place, field by field: building one by value and copying it
    // compiles to memcpy/memclr calls, which live in flash.

    /// Turns the LEDs of slot `n` in the groups of `on` on, everything else
    /// off. Old and new LEDs change in the same MODER write. Setting BSRR
    /// before that can only touch the old LEDs' pins, which turns them off
    /// or lights a new LED a few cycles early.
    #[inline(always)]
    fn set_on(&mut self, n: usize, on: u8) {
        for i in 0..Port::COUNT {
            self.bsrr[i] = 0;
            self.set[i] = MODER_MASK[i].load(Ordering::Relaxed);
            self.clear[i] = 0;
        }
        for (g, group) in groups().iter().enumerate() {
            if on & (1 << g) != 0 {
                // `compile` only sets bits of groups that have LED `n`.
                let led = unsafe { group.get_unchecked(n) };
                let p = led.port as usize;
                self.bsrr[p] |= led.bsrr;
                self.clear[p] |= led.outputs;
            }
        }
    }

    /// Turns the LEDs of slot `n` in the groups of `off` off.
    #[inline(always)]
    fn set_off(&mut self, n: usize, off: u8) {
        for i in 0..Port::COUNT {
            // Volatile, or the zeroing is merged into a memclr call.
            unsafe {
                core::ptr::write_volatile(&mut self.bsrr[i], 0);
                core::ptr::write_volatile(&mut self.set[i], 0);
                core::ptr::write_volatile(&mut self.clear[i], 0);
            }
        }
        for (g, group) in groups().iter().enumerate() {
            if off & (1 << g) != 0 {
                let led = unsafe { group.get_unchecked(n) };
                self.set[led.port as usize] |= led.moder_mask();
            }
        }
    }

    #[inline(always)]
    fn apply(&self) {
        for (i, port) in PORTS.iter().enumerate() {
            if MODER_MASK[i].load(Ordering::Relaxed) != 0 {
                let gpio = port.regs();
                gpio.bsrr().write(|w| w.0 = self.bsrr[i]);
                gpio.moder()
                    .modify(|w| w.0 = (w.0 | self.set[i]) & !self.clear[i]);
            }
        }
    }
}

/// The wake the next interrupt does, worked out one interrupt ahead.
struct Scan {
    slot: usize,
    wake: usize,
    /// The wake's first switch. Its moment is the fixed point after the
    /// wrap, so it is worked out ahead; the switch-offs waited for after it
    /// are worked out while waiting, in `then`.
    first: Switch,
    then: Switch,
    /// The last interrupt overran, so this period's length is not the one
    /// planned and its timing says nothing.
    late: bool,
}

#[exception]
fn PendSV() {
    SWAPPED.wake();
}

#[exception]
fn SysTick() {
    // Starts with a dark wake 0 of slot 0, as the plans do.
    static mut SCAN: Scan = Scan {
        slot: 0,
        wake: 0,
        first: Switch::NONE,
        then: Switch::NONE,
        late: false,
    };
    scan(SCAN);
}

/// With the "scan-in-ram" feature, runs from RAM: flash needs a wait state
/// above 24MHz, which makes the scan ~1.6x slower. Costs ~1KB of RAM.
#[inline(never)]
#[cfg_attr(feature = "scan-in-ram", link_section = ".data.charlieplex_scan")]
fn scan(s: &mut Scan) {
    let syst = syst();
    // The period that started with this wrap.
    let cur = NEXT_LEN.load(Ordering::Relaxed);

    // Switch at a fixed point after the wrap, unless already late. The
    // switch-offs that follow are timed from there.
    while (cur - 1).wrapping_sub(syst.cvr.read()) < SWITCH_AT_CYCLES {}
    s.first.apply();
    let start = syst.cvr.read();

    // SAFETY: only `prepare` writes plans, and only the back one. `s.slot`
    // stays below SLOTS.
    let n = s.slot;
    let slot = unsafe { &*plan(FRONT.load(Ordering::Relaxed)).add(n) };
    let at = slot.wake_time(s.wake);
    let mut a = if s.wake == 0 { 0 } else { slot.first_action(s.wake) + 1 };
    let end = if s.wake + 1 < slot.wakes() {
        slot.first_action(s.wake + 1)
    } else {
        slot.actions as usize
    };
    while a < end {
        s.then.set_off(n, slot.off[a]);
        let wait = slot.time[a] as u32 - at;
        while start.wrapping_sub(syst.cvr.read()) < wait {}
        s.then.apply();
        a += 1;
    }
    let done = syst.cvr.read();

    // The counter has just reloaded with NEXT_LEN.
    ELAPSED.store(
        ELAPSED
            .load(Ordering::Relaxed)
            .wrapping_add(CUR_LEN.load(Ordering::Relaxed)),
        Ordering::Relaxed,
    );
    CUR_LEN.store(cur, Ordering::Relaxed);

    s.wake += 1;
    if s.wake >= slot.wakes() {
        s.wake = 0;
        s.slot += 1;
        if s.slot >= SLOTS.load(Ordering::Relaxed) {
            s.slot = 0;
            // A new refresh starts with the next wake.
            if SWAP.load(Ordering::Acquire) {
                FRONT.store(FRONT.load(Ordering::Relaxed) ^ 1, Ordering::Relaxed);
                SWAP.store(false, Ordering::Release);
                // Waking takes a critical section and runs from flash: leave
                // it to PendSV, after the scan.
                SCB::set_pendsv();
            }
        }
    }

    // Loaded at the next wrap: the period from the next wake to the one
    // after it.
    let next = unsafe { &*plan(FRONT.load(Ordering::Relaxed)).add(s.slot) };
    let from = next.wake_time(s.wake);
    let to = if s.wake + 1 < next.wakes() {
        next.wake_time(s.wake + 1)
    } else {
        WINDOW.load(Ordering::Relaxed)
    };
    let len = to - from;
    NEXT_LEN.store(len, Ordering::Relaxed);
    unsafe { syst.rvr.write(len - 1) };

    if s.wake == 0 {
        s.first.set_on(s.slot, next.on);
    } else {
        s.first.set_off(s.slot, next.off[next.first_action(s.wake)]);
    }

    let spent = (cur - 1).wrapping_sub(syst.cvr.read());
    let late = core::mem::replace(&mut s.late, SCB::is_pendst_pending());
    if s.late {
        // The counter has wrapped again, `spent` is garbage.
        OVERRUNS.store(
            OVERRUNS.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
    }
    if s.late || late {
        return;
    }
    ISR_COUNT.store(
        ISR_COUNT.load(Ordering::Relaxed).wrapping_add(1),
        Ordering::Relaxed,
    );
    ISR_CYCLES.store(
        ISR_CYCLES.load(Ordering::Relaxed).wrapping_add(spent),
        Ordering::Relaxed,
    );
    if spent > ISR_MAX.load(Ordering::Relaxed) {
        ISR_MAX.store(spent, Ordering::Relaxed);
    }
    // Over `cur` if the counter wrapped since `done`.
    let tail = done.wrapping_sub(syst.cvr.read());
    if tail < cur && tail > TAIL_MAX.load(Ordering::Relaxed) {
        TAIL_MAX.store(tail, Ordering::Relaxed);
    }
}

/// CIE 1931 lightness: relative luminance (0..=65535) for perceived levels
/// 0..=255.
static LIGHTNESS: [u16; 256] = [
    0, 28, 57, 85, 114, 142, 171, 199, 228, 256, 285, 313,
    341, 370, 398, 427, 455, 484, 512, 541, 569, 598, 627, 658,
    689, 721, 755, 789, 825, 861, 899, 937, 977, 1018, 1060, 1103,
    1147, 1192, 1239, 1287, 1336, 1386, 1437, 1490, 1544, 1599, 1656, 1714,
    1773, 1834, 1896, 1959, 2024, 2090, 2157, 2226, 2297, 2369, 2442, 2517,
    2593, 2671, 2751, 2832, 2914, 2999, 3085, 3172, 3261, 3352, 3444, 3538,
    3634, 3732, 3831, 3932, 4035, 4139, 4245, 4354, 4464, 4575, 4689, 4804,
    4922, 5041, 5162, 5285, 5410, 5537, 5666, 5797, 5930, 6065, 6202, 6341,
    6482, 6626, 6771, 6918, 7068, 7220, 7373, 7529, 7687, 7848, 8010, 8175,
    8342, 8512, 8683, 8857, 9033, 9212, 9393, 9576, 9762, 9949, 10140, 10333,
    10528, 10725, 10926, 11128, 11333, 11541, 11751, 11963, 12179, 12396, 12617, 12840,
    13065, 13293, 13524, 13757, 13993, 14232, 14474, 14718, 14965, 15215, 15467, 15722,
    15980, 16241, 16505, 16771, 17041, 17313, 17588, 17866, 18147, 18431, 18717, 19007,
    19300, 19596, 19894, 20196, 20501, 20809, 21119, 21433, 21750, 22071, 22394, 22720,
    23050, 23383, 23719, 24058, 24400, 24746, 25095, 25447, 25802, 26161, 26523, 26888,
    27257, 27629, 28004, 28383, 28765, 29151, 29540, 29932, 30328, 30728, 31131, 31537,
    31947, 32360, 32777, 33198, 33622, 34050, 34481, 34916, 35355, 35797, 36243, 36693,
    37146, 37603, 38064, 38529, 38997, 39469, 39945, 40425, 40908, 41396, 41887, 42382,
    42881, 43384, 43891, 44401, 44916, 45435, 45957, 46484, 47015, 47549, 48088, 48631,
    49178, 49728, 50283, 50843, 51406, 51973, 52545, 53120, 53700, 54284, 54873, 55465,
    56062, 56663, 57269, 57878, 58492, 59111, 59733, 60360, 60992, 61627, 62268, 62912,
    63561, 64215, 64873, 65535,
];
