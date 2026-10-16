# Project status

Last updated: 16 October 2026

This file records what actually exists in this repository, so that nothing in the
README or elsewhere overstates it. Update it as items move between sections.

Three kinds of evidence appear below and are never mixed up: **host tests**
(portable code against mocks), **simulation** (software-in-the-loop against an
assumed plant) and **emulation** (the real firmware image on Renode). None of
them is a hardware measurement.

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

**Verification without hardware, in CI** (`.github/workflows/ci.yml`)

- Host tests: 17 suites, 259 tests, 186,608 assertions, 0 failures, from
  `firmware/test_summary.sh`.
- Software-in-the-loop (simulated): `firmware/sim/sil` closes the unmodified
  `FlightController`, IMU driver, PID, mixer and arming logic around a
  rigid-body quad-X with first-order motor lag, 2 ms output latency, sensor
  noise and a constant disturbance torque, through the existing HAL mocks. Five
  step scenarios (roll, pitch and yaw rate; roll and pitch angle) must meet
  overshoot, settling and steady-error limits on a nominal and a perturbed
  plant. Results, plots and the tuning record are in `test-results/sil/`; CI
  regenerates them and fails if they drift from the committed copy.
- Emulation: the real `fc_firmware.elf` runs on Renode 1.16.1's STM32F4
  platform. `firmware/sim/renode/Icm42688BitBang.cs` is a GPIO-level
  ICM-42688-P that decodes the firmware's own bit-banged SPI, so the image
  boots, brings up the IMU, calibrates the gyro and runs its 1 kHz loop. Four
  Robot tests check the UART boot text, clean SPI framing, at least 5000 IMU
  burst reads from the control loop, that the attitude in the CRSF telemetry
  follows a simulated 10 deg tilt and that a wrong `WHO_AM_I` stops the boot.
- Cross build with the Arm GNU Toolchain 13.3.rel1 (pinned and checksummed)
  and CMake, plus `firmware/verify_image.sh` structural checks on the image.

## In progress

**Firmware, portable core.** Written, host-tested and flown in simulation.
Nothing has run on hardware.

- IMU driver for the ICM-42688-P over an abstract SPI bus
- Bit-banged SPI mode 3 master, required by the pin routing described below,
  verified against a bit-level slave model on the host and against the Renode
  model on the emulated MCU
- Attitude estimator, Mahony complementary filter with gyro-bias estimation
- PID attitude loop, angle and rate modes, with quad-X mixer. Default gains are
  simulation-tuned (`tools/pid_tuner.py`); they have never driven a motor
- DShot frame, CRC and timing encoding for 150/300/600/1200
- CRSF receiver parsing and telemetry frame construction
- Rate-scheduled UART telemetry output, non-blocking, resumes partial writes
- Battery monitoring: divider scaling, cell-count detection, mAh integration
- Arming and failsafe state machine
- On-board diagnostics: SPI self-benchmark and control-loop timing statistics,
  printed as plain text so bring-up needs a serial adapter rather than an
  oscilloscope
- I2C bus interface, with drivers tested against simulated devices in
  `firmware/sim/sensors/`: BMP390 barometer (datasheet compensation, ISA
  altitude), MMC5983MA magnetometer (SET/RESET offset removal,
  tilt-compensated heading) and a UBX NAV-PVT parser for the SAM-M10Q GPS

**Firmware, STM32 platform layer.** Written, cross-compiling and running under
emulation. Startup and vector table, linker script, 168 MHz PLL from the 8 MHz
crystal, TIM2 as a 32-bit microsecond clock, GPIO, the bit-banged SPI master,
TIM1/TIM8 PWM ESC output, interrupt-driven UART and a non-blocking ADC. No
vendor HAL or CMSIS dependency; the register definitions are written by hand.

Image size, from `arm-none-eabi-size` with the Arm GNU Toolchain 13.3.rel1:
22,300 B flash (2.13 % of 1 MB) and 5,720 B RAM (4.36 % of 128 KB).

A written bring-up procedure exists at `firmware/docs/BRING_UP.md`. It has not
been performed.

Constraints the design files imposed, all documented in
`firmware/docs/PINOUT.md`:

- `FCSTMV1.05` is one KiCad project holding several electrically separate
  sub-circuits, joined by solder pads at assembly rather than by copper. Only
  three peripherals share a net with the F405: the ICM-42688-P IMU, the BMP390
  barometer and the W25Q128JV flash. The ESCs, GPS, magnetometer, OSD, VTX, BLE
  and receiver all terminate on headers.
- The IMU cannot be driven by the F405 hardware SPI1 peripheral as routed. PA6
  carries the IMU's data-in and PA7 its data-out; the F405 fixes PA6 to
  SPI1_MISO and PA7 to SPI1_MOSI and has no swap bit. A bit-banged GPIO master
  is required on this board revision.
- The same transposition applies to **SPI2 (flash) and SPI3 (OSD)**, verified
  from device pin direction. Neither driver is written.
- The IMU's INT1 is not routed to the F405, so there is no data-ready
  interrupt and sampling must be paced by a timer. The barometer's INT is not
  routed either.
- The magnetometer is not on the F405's bus, so yaw is gyro-only and will drift.
- The barometer's CSB strap (`R128`) is **not fitted**, so its interface mode is
  undefined until CSB is strapped high at `J152.1`. Its I2C address is `0x77`.

## Not started

**Hardware execution.** No line of the firmware has run on a real board.

**STM32 I2C driver.** The `I2cBus` interface has no STM32 implementation, so
the BMP390 driver is not instantiated in `main.cpp`. The magnetometer and GPS
cannot be wired in on this board revision at all (see above and below).

**Velocity-aided attitude estimation.** Simulation shows that in a held
angle-mode tilt the accelerometer correction pulls the estimate towards level:
in the roll angle step the estimate holds 17.5 deg while the true angle reaches
36.4 deg two seconds later. Nothing in the firmware corrects this yet.

**DShot ESC output.** The encoder is written and unit-tested but the timer DMA
is deliberately not wired up: doing so needs the DMA2 request mapping from
RM0090 Table 43, which could not be confirmed against a primary source. PWM
output is implemented instead as the first bring-up path, because it needs no
DMA.

**ESC firmware.** The board carries four `STM32F051C8T6` ESC MCUs. They have no
firmware anywhere in this repository and their signal inputs are not connected
to the F405 on copper: `/S1` to `/S4` and each ESC's `PA2` land on separate
solder pads. Before any motor can turn, either both gaps have to be closed (a
wiring map plus BLDC firmware for four F051s) or the onboard ESCs have to be
bypassed in favour of external ones. **This is the riskiest open item on the
project** and it is a prerequisite for tethered hover, not a detail of it.

**Ground-station tool.** Telemetry is sent as CRSF frames; nothing in the
repository plots it yet.

**Board bring-up, test results and flight.** The boards have not been
assembled or powered. `test-results/` holds only simulated results: no scope
captures, thermal measurements, efficiency data, step responses from a rig or
flight logs.

**Power distribution board.** Not designed.

## Limitations

- **Simulated means simulated.** The SIL plant parameters are assumptions for
  a generic 5-inch 4S quad, not measurements of this airframe. The tuned gains
  are a starting point for bench tuning. The BMP390 model's trimming
  coefficients are synthetic.
- **Emulated timing is not real timing.** Renode is not cycle accurate and
  the CI run sets the emulated core to 25 MIPS to keep it short, so the SPI
  bitrate and loop timing the firmware prints under emulation say nothing about
  silicon. The bit-banged SPI half-bit delay and the loop budget still need a
  board and a serial adapter (or a scope).
- **The Renode model is not the whole board.** The stock STM32F4 description
  has no ADC, so the battery reads 0 V under emulation and the arming and
  motor output path is not exercised there; that path is covered by host tests
  and SIL instead. Renode 1.16.1's GPIO port rewrites every pin on a BSRR
  write, which the IMU model works around (see the comment in
  `Icm42688BitBang.cs`).
- **Calibration constants are defaults.** The battery divider ratio and the
  bit-banged SPI half-bit delay are not measured; both need an assembled board.
- **On-board peripherals not on the F405's nets** (GPS, magnetometer, ESCs,
  OSD, VTX, BLE, receiver) need hand wiring before the firmware can reach them.

## One-line summary

Use this wording, or something equally accurate, wherever the project is described:

> A 6-layer impedance-controlled quadrotor flight controller taken from schematic
> through to fabrication-ready Gerbers and assembly files, with ESC and RF link
> subsystems designed at schematic level. The C++ firmware (attitude estimation,
> PID control, sensor drivers, ESC and radio protocol codecs, arming and
> failsafe logic, plus a bare-metal STM32F405 platform layer) is covered by host
> tests, tuned and gated in software-in-the-loop simulation and boots and runs
> its control loop on an emulated STM32F4 in CI. Hardware bring-up and flight
> are still to do.

Do not describe the aircraft as built, tested or flown until the corresponding
sections above have moved to Complete.
