# Project status

Last updated: 5 August 2026

This file records what actually exists in this repository, so that nothing in the
README or on a CV overstates it. Update it as items move between sections.

## Complete

**Flight controller PCB, `FCSTMV1.05`**

- 6-layer board: `F_Cu`, `In1_Cu`, `In2_Cu`, `In3_Cu`, `In4_Cu`, `B_Cu`
- Schematic across 7 sheets, top level plus `Part2` to `Part7`
- Controlled-impedance stackup specified, see
  `hardware/flight-controller/docs/Required_impedance_control-10.18.xlsx`
- Full Gerber set with PTH and NPTH drill files, revision 10.19
- Assembly position files for top and bottom
- BOM and interactive BOM

**Supporting subsystem design, schematic level**

- ESC, two parts, `hardware/esc/docs/`
- Control link RX and TX, `hardware/rf/docs/`
- Video link RX and TX, `hardware/rf/docs/`

## In progress

**Firmware, portable core.** Written and passing host-side tests. Nothing has
run on hardware. Build and test with `firmware/run_tests.sh`; counts below come
from `firmware/test_summary.sh`.

- IMU driver for the ICM-42688-P over an abstract SPI bus
- Bit-banged SPI mode 3 master, required by the pin routing described below,
  verified against a bit-level slave model that decodes the actual waveform
- Attitude estimator, Mahony complementary filter with gyro-bias estimation
- PID attitude loop, angle and rate modes, with quad-X mixer
- DShot frame, CRC and timing encoding for 150/300/600/1200
- CRSF receiver parsing and telemetry frame construction
- Rate-scheduled UART telemetry output, non-blocking, resumes partial writes
- Battery monitoring: divider scaling, cell-count detection, mAh integration
- Arming and failsafe state machine
- On-board diagnostics: SPI self-benchmark and control-loop timing statistics,
  printed as plain text so bring-up needs a serial adapter rather than an
  oscilloscope
- 221 tests, 183,846 assertions, 0 failures, all running on a host with no
  board attached

Constraints the design files imposed, all documented in
`firmware/docs/PINOUT.md`:

- `FCSTMV1.05` is one KiCad project holding several electrically separate
  sub-circuits, joined by solder pads at assembly rather than by copper. Only
  three peripherals share a net with the F405: the ICM-42688-P IMU, the BMP390
  barometer and the W25Q128JV flash. The ESCs, GPS, magnetometer, OSD, VTX, BLE
  and receiver all terminate on headers.
- The IMU cannot be driven by the F405 hardware SPI1 peripheral as routed. PA6
  carries the IMU's data-in and PA7 its data-out, but the F405 fixes PA6 to
  SPI1_MISO and PA7 to SPI1_MOSI and has no swap bit. A bit-banged GPIO master
  is required on this board revision.
- The same transposition applies to **SPI2 (flash) and SPI3 (OSD)**, verified
  from device pin direction. Any blackbox or OSD driver will need the same
  bit-banged master. Neither driver is written, so nothing is broken today.
- The IMU's INT1 is not routed to the F405, so there is no data-ready
  interrupt and sampling must be paced by a timer. The barometer's INT is not
  routed either.
- The magnetometer is not on the F405's bus, so yaw is gyro-only and will drift.
- The barometer's CSB strap (`R128`) is **not fitted**, so its interface mode is
  undefined until CSB is strapped high at `J152.1`. Its I2C address is `0x77`.

**Firmware, STM32 platform layer.** Written and cross-compiling. Build with
`firmware/build_firmware.sh`; check the linked image with
`firmware/verify_image.sh`. Includes startup and vector table, linker script,
168 MHz PLL from the 8 MHz crystal, TIM2 as a 32-bit microsecond clock, GPIO,
the bit-banged SPI master, TIM1/TIM8 PWM ESC output, interrupt-driven UART and
a non-blocking ADC. No vendor HAL or CMSIS dependency; the register definitions
are written by hand.

Image size, from `arm-none-eabi-size`: 24,160 B flash (2.30 % of 1 MB) and
5,752 B RAM (4.39 % of 128 KB).

A written bring-up procedure exists at `firmware/docs/BRING_UP.md`. It has not
been performed.

None of it has been executed on silicon.

**Continuous integration.** `.github/workflows/ci.yml` builds and runs the host
test suite, cross-compiles the firmware and verifies the linked image.

## Not started

**Hardware execution.** The firmware compiles, links and passes structural
checks on the linked image, but no line of it has run on a real board. The PID
gains in the source are structurally sensible placeholders, not tuned values.

**DShot ESC output.** The encoder is written and unit-tested, but the timer DMA
is deliberately not wired up: doing so needs the DMA2 request mapping from
RM0090 Table 43, which could not be confirmed against a primary source. PWM
output is implemented instead as the first bring-up path, because it needs no
DMA.

**ESC firmware.** The board carries four `STM32F051C8T6` ESC MCUs. They have no
firmware anywhere in this repository, and their signal inputs are not connected
to the F405 on copper — `/S1`–`/S4` and each ESC's `PA2` land on separate solder
pads. Before any motor can turn, either both gaps have to be closed (a wiring map
plus BLDC firmware for four F051s) or the onboard ESCs have to be bypassed in
favour of external ones. **This is the riskiest open item on the project**, and it
is a prerequisite for tethered hover, not a detail of it.

**Barometer and GPS drivers.** Not written. Note also that the onboard SAM-M10Q
is not wired to USART6; the GPS port reaches `J129` pads only.

**Calibration constants.** The battery divider ratio and the bit-banged SPI
half-bit delay are both defaults rather than measurements, and the PID gains
have never driven a motor. All three need an assembled board.

**Loop timing budget.** Unmeasured. The bit-banged SPI cost needs a scope before
the 1 kHz loop rate can be trusted.

**Board bring-up.** The boards have not been assembled or powered. No bring-up
has been performed, so nothing about the design is yet validated in hardware.

**Test results.** `test-results/` is empty. No step responses, no scope captures,
no thermal measurements, no efficiency data.

**Flight.** The aircraft has never flown. There are no flight logs.

**Power distribution board.** Referenced in the README as part of the intended
scope. Not designed.

## Honest one-line summary

Use this wording, or something equally accurate, wherever the project is described:

> A 6-layer impedance-controlled quadrotor flight controller taken from schematic
> through to fabrication-ready Gerbers and assembly files, with ESC and RF link
> subsystems designed at schematic level. The portable half of the C++ firmware
> (attitude estimation, PID control, ESC and radio protocol codecs, arming and
> failsafe logic) is written and covered by host-side unit tests; the STM32
> platform layer and hardware bring-up are still to do.

Do not describe the aircraft as built, tested or flown until the corresponding
sections above have moved to Complete.
