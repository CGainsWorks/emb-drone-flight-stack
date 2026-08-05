# Firmware

C++20 flight-control firmware for the `FCSTMV1.05` board (STM32F405ZGT6),
written from scratch — not a Betaflight fork.

> **Status: builds for the target and passes 188 host tests. Nothing has run on
> hardware.** The boards are unassembled, so the firmware has been compiled,
> linked and structurally verified, but never executed on silicon. See
> [What is not done](#what-is-not-done) for the precise boundary.

## The constraint that shapes the design

No board exists yet, so **every piece of control logic must be testable on a
laptop with nothing attached**. That drives a hard split:

```
        portable, host-tested                    hardware
  ┌────────────────────────────────┐   ┌──────────────────────┐
  │ maths, estimator, PID, mixer,  │   │ STM32 registers,     │
  │ protocol codecs, state machine │──▶│ DMA, timers, GPIO    │
  │                                │   │                      │
  │      (no hardware headers)     │   │  (not yet written)   │
  └────────────────────────────────┘   └──────────────────────┘
                     │                            │
                     └────── HAL interfaces ──────┘
                       SpiBus, Uart, MotorOutput, Clock
```

Everything above the HAL line is compiled and tested natively. The HAL is four
small pure-virtual interfaces (`include/fc/hal/`), so tests substitute mocks: a
fake ICM-42688 that implements the real SPI wire protocol, a fake clock that
makes every timeout deterministic, and a motor output that records commands.

## Running the tests

Requires only a C++20 compiler and CMake. No board, no cross-toolchain.

```bash
./run_tests.sh
```

Verified on Ubuntu 22.04 with g++ 11.4 and CMake 3.22.

```bash
./test_summary.sh
```

Current result — every number quoted in this file comes from that script:

| Suite | Tests | Assertions |
|---|---|---|
| `test_arming` | 21 | 895 |
| `test_attitude` | 18 | 36 |
| `test_battery` | 19 | 536 |
| `test_bitbang_spi` | 17 | 6,555 |
| `test_crsf` | 23 | 124 |
| `test_diagnostics` | 14 | 42 |
| `test_dshot` | 17 | 15,148 |
| `test_flight_controller` | 22 | 24,074 |
| `test_imu` | 18 | 78 |
| `test_math` | 14 | 36 |
| `test_mixer` | 11 | 131,264 |
| `test_pid` | 14 | 5,028 |
| `test_telemetry` | 13 | 30 |
| **Total** | **221** | **183,846** |

Zero failures. The large assertion counts are exhaustive sweeps: every DShot
value 0–2047 has its CRC verified the way an ESC would, every byte value is
round-tripped through the bit-banged SPI waveform, and the mixer is swept across
its whole command space checking nothing ever escapes the 0..1 output range.

## Building the firmware image

Requires `arm-none-eabi-gcc`:

```bash
./build_firmware.sh
```

Current image (from `arm-none-eabi-size`, reproduced by the script):

| Region | Used | Available | Usage |
|---|---|---|---|
| FLASH | 24,160 B | 1 MB | 2.30 % |
| RAM | 5,752 B | 128 KB | 4.39 % |
| CCM | 0 B | 64 KB | 0 % |

`text 23,964 · data 196 · bss 5,560`. Outputs are `fc_firmware.elf`, `.bin` and
`.hex` in `build-arm/`.

### Verifying the image without a board

```bash
./verify_image.sh
```

Linking successfully is a weak claim, so this checks the image is actually
well formed: vector table at `0x08000000`, initial stack pointer at the top of
the 128 KB SRAM, thumb-encoded reset vector matching `Reset_Handler`, the
hard-float ABI in use, and — the easy one to get silently wrong — that the
USART1 handler really is in vector slot 53 rather than falling through to the
default trap. It also asserts the neighbouring unused slot *is* the trap, which
catches an off-by-one in the table.

All checks currently pass. This still does not mean the firmware runs.

## What is implemented

| Area | Files | Notes |
|---|---|---|
| Vectors, quaternions, filters | `math/` | Explicit NED / body-frame convention, asserted in tests |
| Attitude estimator | `control/attitude_estimator.cpp` | Mahony complementary filter, gyro-bias estimation, accelerometer trust gating |
| PID | `control/pid.cpp` | D-on-measurement, filtered derivative, conditional-integration anti-windup |
| Mixer | `control/mixer.cpp` | Quad-X, desaturation preserves attitude authority over throttle |
| IMU driver | `drivers/icm42688.cpp` | Burst read, config read-back verification, bias subtraction |
| Bit-banged SPI | `drivers/bitbang_spi.cpp` | SPI mode 3 software master; waveform verified against a bit-level slave model |
| DShot | `drivers/dshot.cpp` | Frame + CRC, 150/300/600/1200 timing, DMA buffer encoding |
| CRSF | `drivers/crsf.cpp` | RC parsing, telemetry frames, idle-gap resync |
| Telemetry | `telemetry/telemetry.cpp` | Rate-scheduled CRSF output, non-blocking, resumes partial writes |
| Battery monitor | `sensors/battery.cpp` | Divider scaling, cell-count detection, current integration to mAh |
| Diagnostics | `diagnostics/` | Loop timing statistics and SPI self-benchmark, reported as plain text on boot |
| Arming / failsafe | `state/arming.cpp` | See safety properties below |
| Loop integration | `control/flight_controller.cpp` | Angle and Rate modes, tested end to end against mocks |

### The bit-banged SPI is verified without a scope

`test_bitbang_spi.cpp` drives the software master against a slave model that
sees **only pin transitions** and reconstructs the protocol from the waveform —
presenting MISO on falling clock edges and sampling MOSI on rising edges, as a
real mode-3 slave does. That checks clock polarity, clock phase, bit order and
chip-select framing, including that the bus is never clocked while deselected
and that no transaction ends mid-byte.

The final test in that file runs the **real IMU driver over the real bit-banged
master over the waveform-level slave**, so the whole path is proven consistent
before any hardware exists.

### Safety properties enforced and tested

- Motors are only ever enabled in the `Armed` state.
- A board powered up with the arm switch **already on** will not arm; the switch
  must be cycled off first.
- Failsafe **never** transitions straight back to `Armed`. A link that flickers
  back cannot silently restart the motors — an explicit disarm is required.
- The RX-loss timeout is correct **across the 32-bit `micros()` wrap** (~71 min).
- A brief voltage sag under a throttle punch does not trigger failsafe; a
  sustained low voltage does (2 s debounce).
- PID integrators reset on disarm, so no lurch on the next arm.
- NaN or a failed sensor read cannot poison the attitude estimate.

## Two things the design files forced

**1. The IMU cannot use hardware SPI1.** On this board `PA6` carries the IMU's
data-*in* and `PA7` its data-*out*, but the F405 fixes PA6 = `SPI1_MISO` and
PA7 = `SPI1_MOSI` and has no swap bit. Hardware SPI1 would drive MOSI into the
IMU's output. The IMU therefore needs a bit-banged GPIO master. This is why
`SpiBus` is an interface. Full detail in [docs/PINOUT.md](docs/PINOUT.md).

**2. There is no magnetometer on the F405's bus,** so yaw is gyro-only and will
drift. The MMC5983MA is on a separate I2C bus in the GPS power island.

## The STM32 platform layer

`src/platform/stm32/` implements the HAL against the bare metal, with no vendor
HAL or CMSIS dependency — the register definitions are written out by hand in
`registers.hpp`, so the build is fully self-contained.

| Piece | What it does |
|---|---|
| `startup.cpp` | Vector table, reset handler, `.data`/`.bss` init, FPU enable, C++ static constructors |
| `stm32f405zg.ld` | 1 MB flash / 128 KB SRAM / 64 KB CCM layout |
| `system.cpp` | 8 MHz HSE → 168 MHz PLL, flash wait states, bus prescalers; TIM2 as a 32-bit microsecond clock |
| `gpio.cpp` | Pin configuration; `BSRR`-based atomic pin writes |
| `bitbang_spi_stm32.hpp` | The IMU's software SPI master (inter-edge delay only — the logic is in the tested portable class) |
| `pwm_motors.cpp` | TIM1/TIM8 PWM ESC output on `/S1`–`/S4` |
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

## What is not done

Stated plainly rather than stubbed out:

- **Nothing has run on hardware.** No bring-up, no scope captures, no timing
  measurements, no PID tuning. The gains in `FlightControllerConfig` are
  structurally sensible placeholders, **not tuned values** — they have never
  driven a motor.
- **ESC output is PWM, not DShot.** The DShot encoder is written and
  exhaustively unit-tested (every value 0–2047, CRC checked the way an ESC
  would), but the **timer DMA plumbing is deliberately not written**. Wiring it
  needs the DMA2 stream/channel request mapping from RM0090 Table 43, which I
  could not confirm against a primary source offline. Writing plausible-looking
  DMA constants into a flight-critical output path, on a board that cannot be
  tested, would produce code that looks finished and fails mysteriously. PWM
  needs no DMA and is the safer bring-up path. To finish DShot: confirm
  `TIM1_UP` and `TIM8_UP` stream/channel against RM0090, then drive the CCR
  registers in burst mode from the encoder output.
- **The battery divider ratio is a default, not a measurement.**
  `BatteryConfig::voltage_divider_ratio` defaults to 11.0; the real value must
  be measured on an assembled board, because divider tolerance dominates the
  error. Until then reported voltages will be wrong, and the low-voltage
  failsafe thresholds with them.
- **Barometer and GPS drivers are not written.**
- **The bit-banged SPI half-bit delay is a guess.** `kNopsPerHalfBit = 8` is a
  starting point; the real half-period depends on flash wait states and bus
  contention and must be checked with a scope against the ICM-42688-P's 24 MHz
  register-access limit.
- **Loop timing budget is unmeasured.** The bit-banged SPI cost in particular
  needs a scope before the 1 kHz loop rate can be trusted.

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

Summary of the order: SWD → clock → LED heartbeat → serial diagnostics → IMU
`WHO_AM_I` → calibrate SPI rate → confirm loop budget → first motor spin (props
off, current limited) → scope for signal integrity.

## Continuous integration

`.github/workflows/ci.yml` runs both builds on every push touching `firmware/`:
the host test suite, and the cross-compile plus `verify_image.sh`. The firmware
`.elf`/`.bin`/`.hex`/`.map` are uploaded as build artifacts.
