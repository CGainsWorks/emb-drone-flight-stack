# FCSTMV1.05 pinout, as extracted from the design files

Every entry here was read out of the fabricated design rather than assumed:

- pad → net assignments from `hardware/flight-controller/kicad/FCSTMV1.05.kicad_pcb`
  (the PCB netlist is what is actually on copper)
- placed-part list from `hardware/flight-controller/outputs/bom/FCSTMV1.05-10.19.csv`

Cross-checked against the schematic sheets. Note that only the root sheet is
registered in `FCSTMV1.05.kicad_pro`; `Part2`–`Part7` are not linked as
hierarchical sub-sheets, so the PCB netlist was treated as authoritative.

**Nothing here has been verified on hardware.** The boards are unassembled. These
are the connections the design specifies, not measured behaviour.

---

## Main MCU

**STM32F405ZGT6**, LQFP-144, 20×20 mm, 0.5 mm pitch (ref `U5`).
Cortex-M4F, 168 MHz, 1 MB flash, 192 KB SRAM.

## What actually reaches the F405 on copper

`FCSTMV1.05` is **one KiCad project holding several electrically separate
sub-circuits**, not one integrated board: the flight controller, four ESCs
(4× STM32F051C8T6 + DRV8300 + MOSFETs + INA240 current sense), a GPS island,
OSD, VTX, video RX, two SX1280 ELRS radios, two ESP32s and a BLE module. They
are joined by **solder pads fitted by hand at assembly**, not by copper.

Only **three** peripherals share a net with the F405:

| Peripheral | Ref | Interface |
|---|---|---|
| ICM-42688-P IMU | `U7` | SPI1 pins, bit-banged |
| BMP390 barometer | `IC9` | I2C1, PB8/PB9 |
| W25Q128JV flash | `U6` | SPI2 pins, bit-banged |

Everything else — **ESCs, GPS, magnetometer, OSD, VTX, BLE, receiver** —
terminates on a header or solder pad. This is the single most important fact
about the board for firmware purposes, and it is easy to miss because the
schematic looks like one integrated design.

## Clock and debug

| Function | Net | Pin |
|---|---|---|
| HSE in / out | `/OSC_IN`, `/OSC_OUT` | PH0 / PH1 (pads 23, 24) |
| SWDIO | `/SWD` | PA13 (pad 105) |
| SWCLK | `/SWC` | PA14 (pad 109) |
| NRST | `/MCU_NRST` | pad 25 |
| BOOT0 | `/MCU_BOOT` | pad 138 |
| USB D− / D+ | `/DRes_N`, `/DRes_P` | PA11 / PA12 (pads 103, 104) |

- **Crystal: 8.000 MHz** (FY0800027, 18 pF load, 20 ppm) → PLL to 168 MHz.
- **Debug is 2-wire SWD only.** No JTAG. No SWO — PB3, which would carry it, is
  used as `/LED1_MCU`.
- USB FS is present, so DFU flashing is available as well as SWD.

## IMU — ICM-42688-P (`U7`)

| Signal | Net | F405 pin | IMU pad |
|---|---|---|---|
| CS | `/GYRO_CS` | PA4 (pad 40) | 12 |
| SCLK | `/GYRO_SPI_SCK1` | PA5 (pad 41) | 13 |
| MCU → IMU (SDI) | `/GYRO_SPI_MOSI1` | **PA6** (pad 42) | 14 |
| IMU → MCU (SDO) | `/GYRO_SPI_MISO1` | **PA7** (pad 43) | 1 |
| INT1 | `/GYRO_EXTI1` | **not routed to the F405** | 4 |

### ⚠ Hardware SPI1 cannot drive this IMU as routed

On copper, `PA6` shares a net with the IMU's **SDI** (data *in*) and `PA7` shares
a net with the IMU's **SDO** (data *out*).

On the STM32F405 the alternate-function map is fixed in silicon:

- PA6 can only ever be `SPI1_MISO`
- PA7 can only ever be `SPI1_MOSI`

The F405 has **no MISO/MOSI swap bit** (that exists on F3/G4/H7, not on F4). So
enabling the hardware SPI1 peripheral would drive MOSI out of PA7 into the IMU's
output pin — bus contention — and the IMU would never receive a command byte.

**Consequence:** the IMU must be driven by a **bit-banged GPIO master**, which is
free to use PA6 as an output and PA7 as an input. The firmware is written against
a `SpiBus` interface (`include/fc/hal/spi_bus.hpp`) so the driver above it is
unaffected either way, and a hardware-SPI implementation can be dropped in if a
future board revision swaps the pins.

### ⚠ …and it is not just the IMU: all three SPI buses are transposed

| Bus | Device | SCK | MOSI | MISO |
|---|---|---|---|---|
| SPI1 | IMU `U7` | PA5 ✅ | PA6 ❌ | PA7 ❌ |
| SPI2 | flash `U6` | PB13 ✅ | PB14 ❌ | PB15 ❌ |
| SPI3 | OSD `U8` | PC10 ✅ | PC11 ❌ | PB5 ❌ |

Verified from **device pin direction**, not from net names. `W25Q128JV` pad 5 is
`DI` — an input the MCU must drive — and it sits on PB14, which hardware SPI2 can
only ever make an input. Exactly the same contention as the IMU. `AT7456E` pad 9
is `SDIN` and reaches PC11 through the `TXS0104E` level shifter (`IC7`); PC11 is
`SPI3_MISO` in silicon.

SPI3 has no escape route either: its real MOSI (PC12) is consumed by `/UART5_TX`,
and the alternate PB3/PB4/PB5 mapping is transposed in the same way (PB5 is
`SPI3_MOSI`, but the board uses it as MISO).

**Consequence:** a blackbox-logging driver for the flash, and any OSD driver, will
both need the same bit-banged master. Neither is written yet, so nothing is broken
today — but do not reach for hardware SPI on this board.

Secondary consequence: **there is no data-ready interrupt.** `/GYRO_EXTI1` does
not reach the F405, so sampling must be paced by a timer and the driver polled.
(`PC4` / `/GYRO_INIT` is a separate net and is not the INT line.)

## Barometer, magnetometer, GPS

- **BMP390** (`IC9`) — on the F405's **I2C1**: SCL = PB8 (`/I2C_SCL`, pad 139),
  SDA = PB9 (`/I2C_SDA`, pad 140). Strapping now resolved from the placed-part
  list rather than guessed:

  | Strap | Net | Fitted? | Effect |
  |---|---|---|---|
  | `R126` 0 Ω | `/SPI_MISO2` → 3V3 | **yes** | SDO high → **7-bit address `0x77`** |
  | `R127` 0 Ω | `/SPI_MISO2` → GND | no | would have given `0x76` |
  | `R128` 0 Ω | `/BARO_CS` → 3V3 | **no** | CSB left floating |
  | `R103`/`R104` 2k7 | SCL/SDA pull-ups | **yes** | `R124`/`R129` 4k7 not fitted |

  **⚠ `R128` is not fitted, so CSB floats and the part has no defined interface
  mode.** CSB is brought out on `J152.1` and must be strapped high before the
  BMP390 will answer on I2C. Check this at bring-up before condemning a silent
  sensor. `/BARO_EXTI1` does not reach the F405 either, so the baro is
  polling-only.

- **MMC5983MA magnetometer** (`IC10`) — sits on a **separate `/I2C_SCL_2` /
  `/I2C_SDA_2` bus inside the GPS power island** (`3V3_2A_GPS` / `GND_GPS`). It is
  **not wired to the F405**. There is therefore **no heading reference available
  to this MCU**, so yaw is gyro-only and will drift. (`R131` 0 Ω is fitted, tying
  `/MAG_CS` high for I2C mode; its bus pull-ups `R132`/`R133` are *not* fitted.)

- **GPS — `/GPS_UART*` on USART6 does *not* reach the onboard receiver.**
  PC6 (pad 96) and PC7 (pad 97) carry `/GPS_UARTTX` and `/GPS_UARTRX`, and those
  nets terminate on **`J129` solder pads only**.

  The onboard **u-blox SAM-M10Q-00B** (`ANT2`) lives in the GPS power island and
  drives the *separate* nets `/GPS_UARTTX_2` and `/GPS_UARTRX_2` (pads 13/14),
  plus a DDC/I2C port on `/GPS_I2C_SCL` / `/GPS_I2C_SDA` (pads 12/9). Those land
  on `J164`/`J166`. Its timepulse `/GPS_EXTI1` (pad 19) lands on `J160`.

  **None of them reach the F405.** To use the onboard receiver, `J129` must be
  wired across to `J164` by hand. Otherwise USART6 is simply a spare port on a
  header.

## Motor outputs — 4 channels

| Ch | Net | F405 pin | Timer |
|---|---|---|---|
| M1 | `/S1` | PA9 (pad 101) | TIM1_CH2 (AF1) |
| M2 | `/S2` | PA8 (pad 100) | TIM1_CH1 (AF1) |
| M3 | `/S3` | PC9 (pad 99) | TIM8_CH4 (AF3) |
| M4 | `/S4` | PC8 (pad 98) | TIM8_CH3 (AF3) |

All four are DMA-capable (TIM1 and TIM8 via DMA2), so DShot is available on every
channel.

### ⚠ The flight controller is not connected to the onboard ESCs

`/S1`–`/S4` terminate on solder pads: `S1`,`S2` → `J122.6`/`J122.7`;
`S3`,`S4` → `J129.1`/`J129.2`.

The four onboard ESC MCUs (`U1`, `U10`, `U11`, `U12` — all `STM32F051C8T6`) each
take their signal input on their **own** `PA2`, which lands on a *separate* pad
(`J39.1`, `J205.1`, `J248.1`, `J291.1`). Every other ESC signal (`/BEMFA`,
`/BEMF_COMMON`, `/ADC_CURRENT`, `/CLOW`…) is likewise local to its own ESC block.

**No net joins the two sides.** On top of that, **the four F051s have no firmware
anywhere in this repository** — `hardware/esc/` holds design files only.

So there are two prerequisites before any motor can turn, and neither is done:

1. a hand-wiring map from `J122`/`J129` to `J39`/`J205`/`J248`/`J291`, and
2. BLDC firmware for four F051s (commutation, BEMF sensing, current limiting)

…**or** bypass the onboard ESCs entirely and wire `/S1`–`/S4` to external ESCs.
That decision gates everything downstream and should be made before any further
firmware work.

ESC telemetry returns on **USART2** (`/ESC_UART2_*`, PA2/PA3) — also via pads.

## UART map

| UART | TX / RX | Net | Purpose |
|---|---|---|---|
| USART1 | PB6 / PB7 | `/CRX_UART1_*` | **Control receiver (CRSF)** |
| USART2 | PA2 / PA3 | `/ESC_UART2_*` | ESC telemetry |
| USART3 | PB10 / PB11 | `/BLE_UART*` | NINA-B416 BLE |
| UART4 | PA0 / PA1 | `/VTX_UART4_*` | VTX control (SmartAudio / MSP) |
| UART5 | PC12 / PD2 | `/UART5_*` | spare → **used here for ground telemetry** |
| USART6 | PC6 / PC7 | `/GPS_UART*` | → `J129` pads only, **not** the onboard GPS |

Each of these lands on the correct STM32F405 alternate-function pin for its
peripheral, which is a useful independent check that the pad→pin numbering was
read correctly.

Every one of these six ports reaches a header or solder pad rather than a device
on the same board. `/UART5_RX` additionally passes an inverter transistor (`Q8`,
BC817W, with `R94` 10 k), so it can accept an inverted signal such as SBUS or
inverted CRSF.

**Note:** no net on the F405 is literally named "telemetry". Downlink telemetry is
not broken out as its own dedicated F405 UART — in a normal build it rides the
CRSF link back up USART1. UART5 is the only undesignated port, so this firmware
defaults the ground-station telemetry port to **UART5**; it is a single constant
in `include/fc/config/board_v105.hpp` and trivial to repoint.

## Analogue sensing

| Signal | Net | Pin |
|---|---|---|
| Battery voltage | `/ADC_VBAT` | PC1 (pad 27) |
| RSSI | `/ADC_RSSI` | PC2 (pad 28) |
| Current | `/ADC_CURR` | PC3 (pad 29) |

All on ADC123. Used by the low-voltage failsafe.

## Indicators

| Signal | Pin | Pad |
|---|---|---|
| `/LED1_MCU` | PB3 | 133 |
| `/LED2_MCU` | PC14 | 8 |
| `/LED3_MCU` | PC15 | 9 |
| `/BUZZER` | **PC13** | 7 |

**Corrected:** `board_v105.hpp` previously declared `kBuzzer` as **PA15**. Pad 7 is
PC13; PA15 carries `/OSD_CS_Translated`, so driving the buzzer from PA15 would
have toggled the OSD chip select. The constant was unused, so nothing downstream
was affected, but it is now `PC13`.

Note that PC13/PC14/PC15 are the F405's low-drive RTC-domain pins. A schematic
review note embedded in the design file (visible in the KiCad text blocks) flags
that these should not sink or source more than ~20 mW, so the buzzer must be
driven through its MOSFET (`Q7`, IRLML0030) rather than directly — which is how
the board is drawn.

---

## Resolved since the first pass

These were open questions; all four are now answered from the design files.

| Was | Now |
|---|---|
| Confirm the BMP390's `/BARO_CS` and `/SPI_MISO2` strapping | **Resolved.** `R126` fitted → address `0x77`. `R128` **not fitted** → CSB floats and must be strapped at `J152.1`. |
| Can the magnetometer be reached from the F405? | **No.** `IC10` is entirely inside the GPS power island. Yaw is gyro-only. |
| Confirm the `/S1`–`/S4` jumpering into the onboard ESCs | **There is none.** Separate nets on both sides; needs hand wiring *and* ESC firmware that does not exist. |
| Is the SPI transposition limited to the IMU? | **No.** SPI2 and SPI3 are transposed the same way. |

## Open questions to resolve at bring-up

1. **The ESC strategy** — write BLDC firmware for four `STM32F051C8T6`s and define
   the FC→ESC wiring, or bypass the onboard ESCs and use external ones. Nothing
   downstream can be validated end-to-end until this is decided. **This is now the
   riskiest unknown on the project.**
2. **Strap the BMP390's CSB high** at `J152.1` (`R128` is not fitted), or the
   barometer will not answer on I2C at all.
3. **The PA6/PA7 SPI swap** — accept bit-banged SPI on v1.05, or fix the pin
   assignment in a v1.06. If a respin happens, fix SPI2 and SPI3 at the same time.
4. Decide whether the onboard SAM-M10Q is wired to USART6 (`J129` ↔ `J164`) or the
   GPS port is left for an external module.
5. Confirm the bit-banged SPI half-period with a scope. Current setting is
   `kNopsPerHalfBit = 8`; at 168 MHz that is roughly **5.5–7 MHz**, comfortably
   inside the ICM-42688-P's 24 MHz limit, and a 15-byte burst read should cost on
   the order of **20–25 µs**, about 2 % of a 1 kHz loop. *Estimated from
   instruction timing, not measured.*
6. Decide the final ground-telemetry UART (currently UART5).
