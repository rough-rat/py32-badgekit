//! Push button to ground (active low), polled and debounced.

use embassy_time::{Duration, Instant, Timer};
use py32_hal::gpio::Input;

const DEBOUNCE: Duration = Duration::from_millis(40);
const POLL: Duration = Duration::from_millis(10);

pub struct Button<'d> {
    input: Input<'d>,
    low_since: Option<Instant>,
    armed: bool,
}

impl<'d> Button<'d> {
    pub fn new(input: Input<'d>) -> Self {
        Self {
            input,
            low_since: None,
            armed: true,
        }
    }

    /// Raw pin state, not debounced.
    pub fn is_pressed(&self) -> bool {
        self.input.is_low()
    }

    /// Call this regularly (every few ms) from the animation loop.
    /// Returns true once per press, after the button was held for the
    /// debounce time.
    pub fn poll(&mut self) -> bool {
        if !self.is_pressed() {
            self.low_since = None;
            self.armed = true;
            return false;
        }

        let now = Instant::now();
        let low_since = match self.low_since {
            Some(low_since) => low_since,
            None => {
                self.low_since = Some(now);
                return false;
            }
        };

        if self.armed && now >= low_since + DEBOUNCE {
            self.armed = false;
            true
        } else {
            false
        }
    }

    /// Waits `duration`, or returns early with true on a press. This is the
    /// delay to use between animation steps.
    pub async fn pressed_within(&mut self, duration: Duration) -> bool {
        let deadline = Instant::now() + duration;
        loop {
            if self.poll() {
                return true;
            }
            let now = Instant::now();
            if now >= deadline {
                return false;
            }
            Timer::after(POLL.min(deadline - now)).await;
        }
    }

    /// Waits until the button is released and has stopped bouncing.
    /// Use it after a press that changes mode (e.g. after wake-up), so the
    /// same press is not seen again by `poll`.
    pub async fn wait_release(&mut self) {
        while self.is_pressed() {
            Timer::after(POLL).await;
        }
        Timer::after(DEBOUNCE).await;
        self.low_since = None;
        self.armed = true;
    }
}
