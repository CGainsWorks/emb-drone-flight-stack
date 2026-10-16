# Firmware

C++20 flight-control firmware for the `FCSTMV1.05` board (STM32F405ZGT6),
written from scratch rather than forked from Betaflight.

> **Status: passes 259 host tests, flies a simulated quadrotor in
> software-in-the-loop and boots its real image on an emulated STM32F4 in CI.
> Nothing has run on hardware.** The boards are unassembled, so the firmware has
> never executed on silicon. See [What is not done](#what-is-not-done) for the
> precise boundary.

## The constraint that shapes the design

No board exists yet, so **every piece of control logic must be testable on a
laptop with nothing attached**. That drives a hard split:

```
        portable, host-tested                    hardware
  ┌────────────────────────────────┐   ┌──────────────────────┐
  │ maths, estimator, PID, mixer,  │   │ STM32 registers,     │
  │ protocol codecs, sensor        │──▶│ timers, GPIO, UART,  │
  │ drivers, state machine         │   │ ADC (src/platform/)  │
  │      (no hardware headers)     │   │  written, emulated   │
  └────────────────────────────────┘   └──────────────────────┘
                     │                            │
                     └────── HAL interfaces ──────┘
               SpiBus, I2cBus, Uart, MotorOutput, Clock
```

Everything above the HAL line is compiled and tested natively. The HAL is a set
of small pure-virtual interfaces (`include/fc/hal/`), so tests substitute mocks
and simulated devices: a fake ICM-42688 that implements the real SPI wire
protocol, a fake clock that makes every timeout deterministic, a motor output
that records commands, plus register-level models of the barometer and
magnetometer (`sim/sensors/`).

Three layers of verification sit on top, none of which needs a board:

| Layer | Where | What it shows |
|---|---|---|
| Host unit tests | `test/` | Each module behaves to specification against mocks |
| Software in the loop (simulated) | `sim/sil/`, `../tools/pid_tuner.py` | The control code, closed around an assumed quadrotor, is stable and meets step-response limits |
| Emulation | `sim/renode/` | The real cross-compiled image boots, talks to its IMU over bit-banged SPI and runs its control loop on an emulated STM32F4 |

## Running the tests

Requires only a C++20 compiler and CMake. No board, no cross-toolchain.

```bash
./run_tests.sh
./test_summary.sh
```

Verified on Ubuntu 22.04 with g++ 11.4 and CMake 3.22. Every count quoted here
comes from `test_summary.sh`:

| Suite | Tests | Assertions |
|---|---|---|
| `test_arming` | 21 | 895 |
| `test_attitude` | 18 | 36 |
| `test_baro` | 9 | 689 |
| `test_battery` | 19 | 536 |
| `test_bitbang_spi` | 17 | 6,555 |
| `test_crsf` | 23 | 124 |
| `test_diagnostics` | 14 | 42 |
| `test_dshot` | 17 | 15,148 |
| `test_flight_controller` | 22 | 24,074 |
| `test_gps` | 10 | 149 |
| `test_imu` | 18 | 78 |
| `test_mag` | 8 | 1,827 |
| `test_math` | 14 | 36 |
| `test_mixer` | 11 | 131,267 |
| `test_pid` | 14 | 5,028 |
| `test_sil` | 11 | 94 |
| `test_telemetry` | 13 | 30 |
| **Total** | **259** | **186,608** |

Zero failures. The large assertion counts are exhaustive sweeps: every DShot
value from 0 to 2047 has its CRC verified the way an ESC would, every byte value
is round-tripped through the bit-banged SPI waveform and the mixer is swept
across its whole command space checking nothing ever escapes the 0..1 range.

## Software in the loop (simulated)

`sim/sil/` closes the unmodified `FlightController`, ICM-42688 driver, PID,
mixer and arming logic around a rigid-body quad-X:

- Euler rigid-body dynamics with a diagonal inertia, integrated at 4 kHz
- first-order motor lag, thrust and rotor drag torque proportional to the square
  of normalised motor speed, 2 ms output latency
- linear translational drag, a constant disturbance torque standing in for a
  centre-of-mass offset and white noise on the gyro and accelerometer
- the IMU fed through `MockIcm42688`, so the driver's register reads, scaling
  and bias subtraction are all in the loop

The plant parameters are assumptions for a generic 5-inch 4S quad, recorded in
`sim/sil/quad_plant.hpp`; none was measured. The firmware boots, calibrates and
arms exactly as on a board, then flies five step scenarios (roll, pitch and yaw
rate; roll and pitch angle).

```bash
./build/fc_sil                                  # table of results
./build/fc_sil --out ../test-results/sil        # CSVs and metrics.json
./build/fc_sil --check --motor-tau 0.045 --inertia-scale 1.3   # perturbed plant
python3 ../tools/pid_tuner.py tune --start ../tools/placeholder_gains.json
```

`test_sil` is the CI stability gate: with the default gains every scenario must
stay bounded and meet its overshoot, settling and steady-error limits on the
nominal plant and on a slower, heavier one. It also checks the plant's sign
conventions against the mixer independently, which is how an inverted pitch
column in the mixer was found and fixed. Results and plots are in
`../test-results/sil/`; see the top-level README for the numbers and for the
angle-mode estimator drift the simulation exposed.

The default gains in `FlightControllerConfig` were tuned this way. They are
simulation-tuned starting values for bench tuning and have never driven a
motor.

## Emulating the real image on Renode

`sim/renode/` runs the actual `build-arm/fc_firmware.elf` on Renode's stock
STM32F4 platform, extended in `fcstmv105.repl` with the F405's FPU core type,
64 KB CCM RAM and an 84 MHz TIM2 clock so `micros()` ticks at 1 MHz.

The IMU is the awkward part: on this board it is bit-banged over GPIO, so
Renode's SPI models never see a byte. `Icm42688BitBang.cs` is a C# peripheral
that sits on PA4 to PA7, decodes the SPI mode 3 waveform the emulated CPU
produces and drives SDO back, with the same register file as the host model.

```bash
renode-test sim/renode/fc_firmware.robot -r build-arm/renode
renode sim/renode/fcstmv105.resc                # interactive, from sim/renode
```

`fc_firmware.robot` checks that the image prints its boot banner on UART5,
brings up the IMU (one soft reset, no SPI framing errors), reaches the main
loop and makes at least 5000 IMU burst reads, that the attitude in its CRSF
telemetry converges on a simulated 10 deg tilt and that a wrong `WHO_AM_I`
stops it before the loop. CI runs it on Renode 1.16.1.

Timing printed under emulation (SPI bitrate, loop times) is an artefact of the
emulator and its configured instruction rate, not a property of the firmware on
silicon. The stock platform has no ADC, so battery voltage reads 0 V under
emulation and arming is not exercised there.

## Building the firmware image

Requires `arm-none-eabi-gcc`. CI uses the Arm GNU Toolchain 13.3.rel1, pinned
and checksummed, with CMake.

```bash
./build_firmware.sh
```

Current image (from `arm-none-eabi-size`, reproduced by the script):

| Region | Used | Available | Usage |
|---|---|---|---|
| FLASH | 22,300 B | 1 MB | 2.13 % |
| RAM | 5,720 B | 128 KB | 4.36 % |
| CCM | 0 B | 64 KB | 0 % |

`text 22,128, data 172, bss 5,552`. Outputs are `fc_firmware.elf`, `.bin` and
`.hex` in `build-arm/`. The barometer, magnetometer and GPS drivers compile into
the core library but are not referenced from `main.cpp`, so the linker drops
them.

### Verifying the image

```bash
./verify_image.sh
```

Linking successfully is a weak claim, so this checks the image is well formed:
vector table at `0x08000000`, initial stack pointer at the top of the 128 KB
SRAM, thumb-encoded reset vector matching `Reset_Handler`, the hard-float ABI in
use and (the easy one to get silently wrong) that the USART1 handler really is
in vector slot 53 rather than falling through to the default trap. It also
asserts the neighbouring unused slot *is* the trap, which catches an off-by-one
in the table.

## What is implemented

| Area | Files | Notes |
|---|---|---|
| Vectors, quaternions, filters | `math/` | Explicit NED and body-frame convention, asserted in tests |
| Attitude estimator | `control/attitude_estimator.cpp` | Mahony complementary filter, gyro-bias estimation, accelerometer trust gating |
| PID | `control/pid.cpp` | D-on-measurement, filtered derivative, conditional-integration anti-windup |
| Mixer | `control/mixer.cpp` | Quad-X, desaturation preserves attitude authority over throttle |
| IMU driver | `drivers/icm42688.cpp` | Burst read, config read-back verification, bias subtraction |
| Bit-banged SPI | `drivers/bitbang_spi.cpp` | SPI mode 3 software master; waveform verified against a bit-level slave model |
| Barometer | `drivers/bmp390.cpp` | BMP390 over `I2cBus`: reset, identify, NVM trimming, config read-back, datasheet compensation, ISA altitude |
| Magnetometer | `drivers/mmc5983.cpp` | MMC5983MA over `I2cBus`: 18-bit single shot, SET/RESET offset removal, tilt-compensated heading |
| GPS | `drivers/ubx.cpp` | UBX frame parser and NAV-PVT decoder, `GpsReceiver` over `Uart` |
| DShot | `drivers/dshot.cpp` | Frame and CRC, 150/300/600/1200 timing, DMA buffer encoding |
| CRSF | `drivers/crsf.cpp` | RC parsing, telemetry frames, idle-gap resync |
| Telemetry | `telemetry/telemetry.cpp` | Rate-scheduled CRSF output, non-blocking, resumes partial writes |
| Battery monitor | `sensors/battery.cpp` | Divider scaling, cell-count detection, current integration to mAh |
| Diagnostics | `diagnostics/` | Loop timing statistics and SPI self-benchmark, reported as plain text on boot |
| Arming and failsafe | `state/arming.cpp` | See safety properties below |
| Loop integration | `control/flight_controller.cpp` | Angle and Rate modes, tested end to end against mocks and in simulation |

### The bit-banged SPI is verified without a scope

`test_bitbang_spi.cpp` drives the software master against a slave model that
sees **only pin transitions** and reconstructs the protocol from the waveform:
it presents MISO on falling clock edges and samples MOSI on rising edges, as a
real mode-3 slave does. That checks clock polarity, clock phase, bit order and
chip-select framing, including that the bus is never clocked while deselected
and that no transaction ends mid-byte.

The final test in that file runs the **real IMU driver over the real bit-banged
master over the waveform-level slave**. The Renode suite then repeats the check
with the cross-compiled master toggling emulated STM32 GPIO registers.

### Safety properties enforced and tested

- Motors are only ever enabled in the `Armed` state.
- A board powered up with the arm switch **already on** will not arm; the switch
  must be cycled off first.
- Failsafe **never** transitions straight back to `Armed`. A link that flickers
  back cannot silently restart the motors; an explicit disarm is required.
- The RX-loss timeout is correct **across the 32-bit `micros()` wrap** (about
  71 min).
- A brief voltage sag under a throttle punch does not trigger failsafe; a
  sustained low voltage does (2 s debounce).
- PID integrators reset on disarm, so no lurch on the next arm.
- NaN or a failed sensor read cannot poison the attitude estimate.

## What the design files forced

**1. No SPI bus on this board can use the hardware peripheral.** On this board
`PA6` carries the IMU's data-*in* and `PA7` its data-*out*, while the F405 fixes
PA6 = `SPI1_MISO` and PA7 = `SPI1_MOSI` and has no swap bit. Hardware SPI1 would
drive MOSI into the IMU's output. The IMU therefore needs a bit-banged GPIO
master, which is why `SpiBus` is an interface. **SPI2 (flash) and SPI3 (OSD) are
transposed the same way**, so a blackbox or OSD driver will need the same
treatment. Full detail in [docs/PINOUT.md](docs/PINOUT.md).

**2. There is no magnetometer on the F405's bus,** so yaw is gyro-only and will
drift. The MMC5983MA is on a separate I2C bus in the GPS power island; its driver
exists for a later revision or an external compass.

**3. Most of the board is not wired to the MCU.** `FCSTMV1.05` is one KiCad
project holding several electrically separate sub-circuits joined by solder pads
at assembly. Only the IMU, the BMP390 and the SPI flash share a net with the
F405. The ESCs, GPS, magnetometer, OSD, VTX, BLE and receiver all terminate on
headers, including the four onboard `STM32F051C8T6` ESC MCUs, which are
unconnected *and* have no firmware in this repository.

## The STM32 platform layer

`src/platform/stm32/` implements the HAL against the bare metal with no vendor
HAL or CMSIS dependency: the register definitions are written out by hand in
`registers.hpp`, so the build is fully self-contained.

| Piece | What it does |
|---|---|
| `startup.cpp` | Vector table, reset handler, `.data`/`.bss` init, FPU enable, C++ static constructors |
| `stm32f405zg.ld` | 1 MB flash, 128 KB SRAM, 64 KB CCM layout |
| `system.cpp` | 8 MHz HSE to 168 MHz PLL, flash wait states, bus prescalers; TIM2 as a 32-bit microsecond clock |
| `gpio.cpp` | Pin configuration; `BSRR`-based atomic pin writes |
| `bitbang_spi_stm32.hpp` | The IMU's software SPI master (inter-edge delay only; the logic is in the tested portable class) |
| `pwm_motors.cpp` | TIM1/TIM8 PWM ESC output on `/S1` to `/S4` |
| `uart_stm32.cpp` | Interrupt-driven RX (CRSF at 420 kbaud), non-blocking polled TX |
| `adc.cpp` | Round-robin non-blocking sampling of `/ADC_VBAT`, `/ADC_RSSI`, `/ADC_CURR` |

Two decisions worth flagging:

- **TIM2, not SysTick or DWT, provides `micros()`.** TIM2 is one of the F4's
  32-bit timers, so it maps exactly onto the 32-bit `micros()` contract the
  portable code already handles wrap for. A 16-bit timer would wrap every 65 ms
  and quietly break the failsafe timeouts.
- **Receive is interrupt-driven, transmit is polled.** CRSF at 420 kbaud is a
  byte roughly every 24 µs; polling once per 1 ms control cycle would lose most
  of a frame. Telemetry TX is best-effort and must never stall the loop.

There is no STM32 implementation of `I2cBus` yet, so the barometer is not
brought up in `main.cpp`.

## What is not done

Stated plainly rather than stubbed out:

- **Nothing has run on hardware.** No bring-up, no scope captures, no timing
  measurements, no PID tuning on a rig. The gains in `FlightControllerConfig`
  were tuned in simulation against an assumed plant and **have never driven a
  motor**.
- **Angle mode drifts under a held tilt.** Simulation shows the accelerometer
  correction pulling the estimate towards level while a tilt is held, because
  the accelerometer sees thrust rather than gravity. There is no velocity aiding
  to correct it.
- **ESC output is PWM, not DShot.** The DShot encoder is written and
  exhaustively unit-tested (every value from 0 to 2047, CRC checked the way an
  ESC would) while the **timer DMA plumbing is deliberately not written**. Wiring
  it needs the DMA2 stream and channel request mapping from RM0090 Table 43,
  which could not be confirmed against a primary source offline. Writing
  plausible-looking DMA constants into a flight-critical output path, on a board
  that cannot be tested, would produce code that looks finished and fails
  mysteriously. PWM needs no DMA and is the safer bring-up path. To finish
  DShot: confirm `TIM1_UP` and `TIM8_UP` stream and channel against RM0090, then
  drive the CCR registers in burst mode from the encoder output.
- **No STM32 I2C driver.** The BMP390 driver is tested against a simulated
  device only. On the board it is on I2C1 at address `0x77` and its CSB strap
  (`R128`) is not fitted, so CSB must be tied high at `J152.1` before it will
  respond at all.
- **The magnetometer and GPS do not reach the F405 on this revision.** Their
  drivers are tested against simulated devices; the onboard SAM-M10Q is not
  wired to USART6 and the MMC5983MA is on an isolated bus.
- **The battery divider ratio is a default, not a measurement.**
  `BatteryConfig::voltage_divider_ratio` defaults to 11.0; the real value must
  be measured on an assembled board because divider tolerance dominates the
  error. Until then reported voltages will be wrong and the low-voltage
  failsafe thresholds with them.
- **There is no ESC firmware and the ESCs are not connected.** Four bare
  `STM32F051C8T6`s, with `/S1` to `/S4` and each ESC's `PA2` on separate solder
  pads. Nothing can drive a motor until this is resolved one way or the other.
- **The bit-banged SPI half-bit delay is a guess.** `kNopsPerHalfBit = 8` is a
  starting point; the real half-period depends on flash wait states and bus
  contention and must be checked against the ICM-42688-P's 24 MHz
  register-access limit. From instruction timing it should land around 5.5 to
  7 MHz, putting a 15-byte burst read at roughly 20 to 25 µs (about 2 % of a
  1 kHz loop), though that is an estimate and not a measurement. The emulator
  cannot settle it.
- **Loop timing budget is unmeasured.** Emulated timing does not count.

## Next steps

Everything remaining needs a physical board. The full procedure, with expected
values and failure modes, is in [docs/BRING_UP.md](docs/BRING_UP.md).

**The board measures itself, so most of it needs no oscilloscope.** On boot the
firmware benchmarks its own SPI bus against the microsecond counter and prints
the effective bitrate and burst time; while disarmed it prints loop timing
statistics including a budget-overrun count. Both go out as plain text on UART5
at 115200 baud, so a USB-serial adapter is enough to calibrate
`kNopsPerHalfBit` and confirm the 1 kHz budget. A scope is only genuinely
required for signal integrity (step 9).

Summary of the order: SWD, clock, LED heartbeat, serial diagnostics, IMU
`WHO_AM_I`, calibrate SPI rate, confirm loop budget, first motor spin (props
off, current limited), scope for signal integrity.

## Continuous integration

`.github/workflows/ci.yml` has two jobs. The host job builds and runs the test
suite, regenerates the simulated step responses, gates them against their
limits on the nominal and perturbed plants and checks them against the
committed copy in `test-results/sil/`. The image job cross-compiles with the
pinned Arm GNU Toolchain, runs `verify_image.sh`, then runs the Renode suite.
The firmware `.elf`, `.bin`, `.hex` and `.map`, the Renode logs and the
simulated results are uploaded as artifacts.
