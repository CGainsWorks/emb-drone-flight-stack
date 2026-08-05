# Bring-up procedure, FCSTMV1.05

Nothing in this document has been performed. It is the procedure to follow once
a board exists, written while the design is fresh so the expected values are
recorded rather than reconstructed later.

**Safety, before anything else:** propellers off for every step here. There is
no step in this document that requires a propeller fitted. Motors are only
commanded in step 8, and only on a current-limited supply.

---

## Equipment

| Step | Needs |
|---|---|
| 1–3 | ST-Link or equivalent SWD probe, bench supply |
| 4–7 | USB-serial adapter (3.3 V) on UART5 — **no oscilloscope required** |
| 8 | Current-limited supply, motors without props |
| 9 | Oscilloscope (signal integrity only) |

The firmware measures its own SPI rate and loop timing and prints them as plain
text, so steps 4–7 need a serial terminal rather than an instrument. A scope is
still the authority on edge quality and is used in step 9, but it is not needed
to answer "is this fast enough and within the IMU's limits".

---

## 1. Power and SWD

1. Board on a current-limited bench supply, limit set low (100 mA) initially.
2. Confirm 3V3 rails present before connecting anything else.
3. Attach SWD: **SWDIO = PA13, SWCLK = PA14**, plus NRST and ground.
   There is no JTAG on this board and no SWO — PB3 is used as `LED1`.
4. Confirm the debugger can halt the core and read the CPU ID.

**If SWD does not connect:** check NRST is not held low, and that BOOT0
(`/MCU_BOOT`) is not strapped high.

## 2. Clock

The firmware refuses to continue if the PLL does not lock — `configure_clocks()`
returns false and `main()` parks in a tight loop before any peripheral is set up.

1. Flash `fc_firmware.elf`.
2. If the board appears completely dead with no LED activity at all, suspect the
   8 MHz crystal (`Y`, FY0800027). Halt in the debugger and check `RCC->CR`:
   bit 17 (`HSERDY`) and bit 25 (`PLLRDY`) should both be set.

Expected: SYSCLK 168 MHz, AHB 168 MHz, APB1 42 MHz, APB2 84 MHz.

## 3. LED heartbeat

`LED1` is on **PB3**.

| Pattern | Meaning |
|---|---|
| Steady on, then blinking slowly (~1 Hz) | Running, disarmed |
| Fast blink (~5 Hz) | Armed |
| Continuous ~3 Hz blink from `fault_loop()` | IMU init or gyro calibration failed |

A board that reaches the slow blink has completed clock setup, GPIO, PWM init,
UART init, IMU detection and gyro calibration.

## 4. Serial diagnostics

Connect a 3.3 V USB-serial adapter to **UART5: TX = PC12, RX = PD2**, at
**115200 baud, 8N1**. On boot the firmware prints:

```
=== FCSTMV1.05 boot ===
IMU: ok
SPI effective bitrate: <n> Hz
SPI burst (15 bytes):  <n> us
Battery cells detected: <n>S
Pack voltage (mV):      <n>
Loop budget:            1000 us
=== running ===
```

`IMU: NO RESPONSE (WHO_AM_I mismatch)` is the single most likely first failure
on this board — see step 5.

## 5. IMU over bit-banged SPI

**This is the step most likely to fail, and the reason is a known design issue.**

The ICM-42688-P cannot be driven by the F405's hardware SPI1 peripheral on this
board: `PA6` carries the IMU's data-*in* and `PA7` its data-*out*, but the F405
fixes PA6 to `SPI1_MISO` and PA7 to `SPI1_MOSI` and has no swap bit. The
firmware therefore bit-bangs the bus over GPIO. See [PINOUT.md](PINOUT.md).

If `WHO_AM_I` does not read back `0x47`:

1. Confirm the IMU is powered — `3V3_IMU` is a separate net from `3V3_2A_FC`.
2. Confirm CS (PA4) idles high and SCK (PA5) idles high. The driver parks the
   clock high before asserting CS because it runs in SPI mode 3.
3. Slow the bus down: raise `kNopsPerHalfBit` in
   `src/platform/stm32/bitbang_spi_stm32.hpp` and re-flash. The default is 8,
   which is a **guess** — this is the value step 6 exists to calibrate.
4. If it still fails, verify on a scope that PA6 is driven by the MCU and PA7 is
   not. Getting these the wrong way round is exactly the failure the routing
   invites.

## 6. Calibrate the SPI bit rate

Read `SPI effective bitrate` from the boot output.

| Result | Action |
|---|---|
| Above ~10 MHz | Increase `kNopsPerHalfBit`. The ICM-42688-P's limit is 24 MHz for register access; leave margin. |
| 1–8 MHz | Good working range. |
| Below ~500 kHz | Too slow — see step 7, the loop budget will not close. |

`SPI burst (15 bytes)` is the number that feeds the loop budget: it is the cost
of one full IMU sample read including chip-select overhead.

## 7. Loop timing

While **disarmed**, the firmware prints loop statistics every 5000 cycles:

```
loop us  min=<n>
         max=<n>
        mean=<n>
    overruns=<n>
```

The budget is 1000 µs (1 kHz).

- **`overruns` must be 0.** A good mean with occasional overruns still means the
  loop is missing its deadline, which is why it is counted separately.
- `max` is the figure that matters, not `mean`.
- If `max` approaches 1000 µs, the largest single contributor will be the
  bit-banged SPI burst from step 6. Options in order of preference: speed up the
  SPI (fewer NOPs, if step 6 allows), drop the loop to 500 Hz in
  `board_v105.hpp`, or move the IMU read to DMA on a board revision that fixes
  the PA6/PA7 routing.

Statistics are only printed while disarmed, so this never competes with a
flying aircraft for loop time.

## 8. First motor spin

**Propellers off. Current-limited supply. Motors clamped.**

ESC output is conventional PWM (1000–2000 µs at 400 Hz) on:

| Motor index | Net | Pin | Timer |
|---|---|---|---|
| 0 | `/S1` | PA9 | TIM1_CH2 |
| 1 | `/S2` | PA8 | TIM1_CH1 |
| 2 | `/S3` | PC9 | TIM8_CH4 |
| 3 | `/S4` | PC8 | TIM8_CH3 |

1. Confirm all four outputs sit at 1000 µs while disarmed. The firmware parks
   them there before enabling the timer outputs, so no ESC should ever see a
   high pulse during initialisation.
2. **Verify the motor-to-arm mapping before anything else.** The mixer assumes
   index 0 = front-right (CCW), 1 = rear-right (CW), 2 = rear-left (CCW),
   3 = front-left (CW). If the wiring does not match, the aircraft will flip on
   takeoff. Spin each motor individually and confirm both position and
   direction.
3. Arm and confirm the motors respond to stick input in the correct sense: roll
   right should increase the *left* motors.

## 9. Signal integrity (oscilloscope)

Only now is a scope genuinely required:

- SPI clock edge quality and any ringing at the IMU.
- PWM pulse width accuracy against the commanded value.
- Supply rail noise under motor load.

## 10. Calibration values to record

Three constants in the source are placeholders and must be replaced with
measurements. Record them here once known:

| Constant | Where | Current value | Measured |
|---|---|---|---|
| `kNopsPerHalfBit` | `bitbang_spi_stm32.hpp` | 8 | |
| `voltage_divider_ratio` | `BatteryConfig` | 11.0 | |
| PID gains | `FlightControllerConfig` | untuned defaults | |

The battery divider in particular must be measured rather than calculated from
the schematic values: divider tolerance dominates the error, and every
low-voltage failsafe threshold depends on it.

---

## Still not implemented

- **DShot.** The encoder is written and tested, but the timer DMA is not wired
  up because the DMA2 request mapping could not be confirmed against RM0090.
  PWM is the bring-up path.
- **Barometer and GPS drivers.**
- **Magnetometer** is not reachable from the F405 on this board, so yaw is
  gyro-only and will drift.
