# py32-badgekit

Common firmware parts for coin cell LED badges built on PY32 MCUs.

Scope, on purpose narrow: a CR2032, one or two buttons, a lot of LEDs in
some arrangement, off via button, Stop mode while off. Everything specific to
a board (chip, pin tables, animations) lives in that board's own firmware
directory, next to its PCB.

## Modules

| Module | What it does | Claims |
|---|---|---|
| `power` | Stop mode until a button is pressed | the button's EXTI line |
| `button` | Debounced, polled button; delay-or-press helper | - |
| `charlieplex` (feature) | Scans any number of charlieplex matrices, LEDs addressed by index | SysTick |
| `soft_pwm` (feature) | Per-LED brightness for LEDs wired straight to GPIOs | TIM14 |
| `battery` (feature) | Supply voltage via the internal reference | ADC interrupt |
| `font` | 5x7 font and one-line text columns | - |

## Use

```toml
[dependencies]
py32-badgekit = { git = "ssh://git@github.com/rough-rat/py32-badgekit.git", tag = "v0.1.0", features = ["charlieplex"] }
```

A board pins the tag it was written against and never has to follow later
changes; its `Cargo.lock` records the exact commit.

The crate does not select a chip. The board crate depends on `py32-hal`
itself (same version as here, pinned exactly), enables the chip feature and a
time driver, and supplies `memory.x`.

## New board

Copy `starter/` next to the board's KiCad project and follow `AGENTS.md`.

## Check

There is no host build. To check the library, build the starter:

```sh
cd starter
cargo +nightly build --release
cargo +nightly build --release --features py32-badgekit/soft-pwm,py32-badgekit/battery
```

## License

MIT
