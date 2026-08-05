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

The board is a heavily integrated design that also carries an onboard 4-in-1 ESC
(4× STM32F051C8T + 4× DRV8300 + 24× MOSFETs + 4× INA240 current sense), GPS, OSD,
VTX, video RX, two SX1280 ELRS radios, two ESP32s and a BLE module. Only the
F405-facing subset matters for this firmware.

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

Secondary consequence: **there is no data-ready interrupt.** `/GYRO_EXTI1` does
not reach the F405, so sampling must be paced by a timer and the driver polled.
(`PC4` / `/GYRO_INIT` is a separate net and is not the INT line.)

## Barometer, magnetometer, GPS

- **BMP390** (`IC9`) — on the F405's **I2C1**: SCL = PB8 (`/I2C_SCL`, pad 139),
  SDA = PB9 (`/I2C_SDA`, pad 140). It also has `/BARO_CS` and `/SPI_MISO2` nets
  that do **not** reach the F405; these most likely strap it into I2C mode and
  set its address. *Unconfirmed — worth checking on the schematic sheet.*
- **MMC5983MA magnetometer** (`IC10`) — sits on a **separate `/I2C_SCL_2` /
  `/I2C_SDA_2` bus inside the GPS power island** (`3V3_2A_GPS` / `GND_GPS`). It is
  **not wired to the F405**. There is therefore **no heading reference available
  to this MCU**, so yaw is gyro-only and will drift.
- **u-blox SAM-M10Q** (`ANT2`) — **USART6**: TX = PC6 (pad 96), RX = PC7 (pad 97).
  Timepulse on `/GPS_EXTI1`.

## Motor outputs — 4 channels

| Ch | Net | F405 pin | Timer |
|---|---|---|---|
| M1 | `/S1` | PA9 (pad 101) | TIM1_CH2 (AF1) |
| M2 | `/S2` | PA8 (pad 100) | TIM1_CH1 (AF1) |
| M3 | `/S3` | PC9 (pad 99) | TIM8_CH4 (AF3) |
| M4 | `/S4` | PC8 (pad 98) | TIM8_CH3 (AF3) |

All four are DMA-capable (TIM1 and TIM8 via DMA2), so DShot is available on every
channel. On copper `/S1`–`/S4` land on header/solder pads (e.g. `/S1` → `J122`),
which is how they reach the onboard ESC F051s — worth confirming the intended
jumpering during bring-up, though the firmware simply sees four outputs.

ESC telemetry returns on **USART2** (`/ESC_UART2_*`, PA2/PA3).

## UART map

| UART | TX / RX | Net | Purpose |
|---|---|---|---|
| USART1 | PB6 / PB7 | `/CRX_UART1_*` | **Control receiver (CRSF)** |
| USART2 | PA2 / PA3 | `/ESC_UART2_*` | ESC telemetry |
| USART3 | PB10 / PB11 | `/BLE_UART*` | NINA-B416 BLE |
| UART4 | PA0 / PA1 | `/VTX_UART4_*` | VTX control (SmartAudio / MSP) |
| UART5 | PC12 / PD2 | `/UART5_*` | spare → **used here for ground telemetry** |
| USART6 | PC6 / PC7 | `/GPS_UART*` | GPS |

Each of these lands on the correct STM32F405 alternate-function pin for its
peripheral, which is a useful independent check that the pad→pin numbering was
read correctly.

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

`/LED1_MCU` on PB3 (pad 133), `/LED2_MCU` pad 8, `/LED3_MCU` pad 9,
`/BUZZER` pad 7.

---

## Open questions to resolve at bring-up

1. **The PA6/PA7 SPI swap** — the single most important item. Either accept
   bit-banged SPI on v1.05, or swap the pins (or move the IMU to SPI3) in a v1.06.
2. Confirm the BMP390's `/BARO_CS` and `/SPI_MISO2` strapping.
3. Confirm whether the magnetometer can be reached at all from the F405, or only
   via the GPS module.
4. Confirm the `/S1`–`/S4` jumpering into the onboard ESC MCUs.
5. Decide the final ground-telemetry UART.
