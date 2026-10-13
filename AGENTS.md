# Writing firmware for a new board

For AI agents (and people) starting a board firmware on top of this crate.

## Start

1. Copy `starter/` into the board's repository, next to its KiCad project,
   including `Cargo.lock`. The lock file holds a dependency set that is known
   to build; a fresh resolution picks newer, incompatible versions.
2. In `Cargo.toml`: rename the package, switch `py32-badgekit` from `path` to
   the `git` + `tag` line given there, enable only the features the board
   uses.
3. Set the chip in three places: the `py32-hal` feature in `Cargo.toml`,
   `memory.x`, and `CHIP` in `flash.sh` / `.cargo/config.toml`
   (`probe-rs chip list | grep -i PY32`).
4. Rewrite the "Board" section of `src/main.rs` from the schematic, then the
   animations. Keep the main loop: animate, stop LEDs, wait for release,
   `power::stop`, wait for release, start LEDs.

Board specific code stays in the board's repository. Do not add it here, and
do not edit this crate from a board's checkout; if the crate lacks something,
change it here and tag a new version.

## Pin tables

- Take pins and LED order from the board's netlist and footprint positions,
  not from assumptions about the reference designators.
- `charlieplex`: one group per independent matrix. The LED index is the
  position in the table, so build the table in the order the animations want
  (row-major for a grid, along the ring for a ring).
  `Led::canonical` covers the usual wiring where each pin is the anode of
  N-1 consecutive LEDs.
- Never put PA13/PA14 (SWD) in a table.

## Bring-up checklist

- **Recovery delay.** Keep the `asm::delay` at the top of `main`. It is the
  window in which a probe can attach if the rest of the firmware breaks SWD
  or sleeps at once.
- **BOOT0.** PF4 is BOOT0. If the board pulls it high at reset (e.g. a button
  with a pull-up on that pin), the chip starts its ROM bootloader instead of
  the firmware. Check `probe-rs read b32 0x0 2` against `0x08000000`: if they
  differ, the chip is in the bootloader. That is a hardware fix.
- **Stop mode kills SWD.** Once the badge is "off" the probe cannot attach;
  wake it with the button or reset it first.
- **Measure the off current once per board.** The Stop sequence in `power` is
  verified, but the result depends on the state of every pin. Stop the LED
  drivers before `power::stop`; they float their pins.
- **Wake pin.** Not every pin can wake the chip, see `power::WakePin`.
- **LEDs without series resistors** are much brighter on a fresh cell. Use
  `battery` to scale brightness where it matters.

## Limits to know

- `charlieplex` is on/off per LED, no brightness levels.
- `soft_pwm` handles 8 LEDs, all on one port.
- Embassy time does not advance while in Stop mode.
