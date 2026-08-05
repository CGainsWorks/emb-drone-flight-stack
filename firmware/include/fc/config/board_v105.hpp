#pragma once

#include <cstdint>

// ===========================================================================
// Board definition: FCSTMV1.05
//
// Every value here was read out of the fabricated design, not assumed:
// pad -> net assignments come from the PCB netlist
// (hardware/flight-controller/kicad/FCSTMV1.05.kicad_pcb) and the placed-part
// list from outputs/bom/FCSTMV1.05-10.19.csv.
//
// MCU:     STM32F405ZGT6, LQFP-144 (ref U5)
// Clock:   8.000 MHz HSE crystal (FY0800027, 18 pF, 20 ppm) -> PLL to 168 MHz
// Debug:   SWD only (SWDIO PA13, SWCLK PA14). No JTAG, no SWO.
// IMU:     ICM-42688-P (ref U7) on SPI1 pins
// Baro:    BMP390 (ref IC9) on I2C1, 7-bit address 0x77
// GPS:     USART6 (PC6/PC7) reaches solder pads on J129 only. The onboard
//          u-blox SAM-M10Q (ref ANT2) is NOT wired to the F405 -- see
//          "What actually reaches the F405 on copper" below.
//
// FCSTMV1.05 is one KiCad project holding several electrically separate
// sub-circuits (flight controller, 4x ESC, GPS island, OSD, VTX, video RX,
// two ELRS radios). They are joined by solder pads at assembly, not by
// copper. Only three peripherals share a net with the F405:
//
//   ICM-42688-P IMU (U7)      SPI1 pins, bit-banged (see below)
//   BMP390 barometer (IC9)    I2C1, PB8/PB9
//   W25Q128JV flash (U6)      SPI2 pins, bit-banged (see below)
//
// Everything else -- ESCs, GPS, magnetometer, OSD, VTX, BLE, receiver --
// terminates on a header or solder pad and needs a wire fitted by hand.
//
// NOTE (unassembled board): none of this has been verified against hardware.
// The pin numbers are what the netlist says; the electrical behaviour is
// unproven until a board is built and brought up.
// ===========================================================================

namespace fc {
namespace board {

enum class Port : uint8_t { A = 0, B, C, D, E, F, G, H };

struct Pin {
    Port port;
    uint8_t number;
};

// --- Clock -------------------------------------------------------------
inline constexpr uint32_t kHseFrequencyHz = 8000000u;
inline constexpr uint32_t kSysclkHz = 168000000u;
// TIM1/TIM8 sit on APB2; with APB2 at 84 MHz the timer clock doubles to 168.
inline constexpr uint32_t kMotorTimerClockHz = 168000000u;

// --- Debug -------------------------------------------------------------
inline constexpr Pin kSwdio{Port::A, 13};  // net /SWD,  pad 105
inline constexpr Pin kSwclk{Port::A, 14};  // net /SWC,  pad 109

// --- IMU: ICM-42688-P (U7) --------------------------------------------
//
// WARNING - READ BEFORE WRITING AN SPI DRIVER FOR THIS BOARD.
//
// These four pins are wired to the IMU, but they CANNOT be driven by the F405
// hardware SPI1 peripheral:
//
//   net /GYRO_SPI_MOSI1 is on PA6, and connects to the IMU's SDI (data in)
//   net /GYRO_SPI_MISO1 is on PA7, and connects to the IMU's SDO (data out)
//
// On the STM32F405 the alternate-function map is fixed: PA6 can only ever be
// SPI1_MISO and PA7 can only ever be SPI1_MOSI. The F405 has no MISO/MOSI swap
// bit (that exists on F3/G4/H7, not here). Enabling hardware SPI1 would drive
// MOSI into the IMU's output pin and the IMU would never receive a command.
//
// Consequence: the IMU must be driven by a bit-banged GPIO master, which is
// free to use PA6 as an output and PA7 as an input. See hal/spi_bus.hpp.
inline constexpr Pin kImuCs{Port::A, 4};    // net /GYRO_CS,        pad 40
inline constexpr Pin kImuSck{Port::A, 5};   // net /GYRO_SPI_SCK1,  pad 41
inline constexpr Pin kImuSdi{Port::A, 6};   // net /GYRO_SPI_MOSI1, pad 42 (MCU output)
inline constexpr Pin kImuSdo{Port::A, 7};   // net /GYRO_SPI_MISO1, pad 43 (MCU input)

// The IMU's INT1 (/GYRO_EXTI1) is NOT routed to the F405 on this board, so
// there is no data-ready interrupt. Sampling must be paced by a timer.
// (PC4 / /GYRO_INIT is a different net and is not the INT line.)
inline constexpr bool kImuInterruptAvailable = false;

// --- The MOSI/MISO transposition is not limited to the IMU --------------
//
// All three SPI buses on this board are wired the same way round as SPI1,
// i.e. transposed relative to the F405's fixed alternate-function map.
// Verified from device pin direction, not from net names:
//
//   SPI1  IMU (U7)     SCK PA5  ok | MOSI PA6  clash | MISO PA7  clash
//   SPI2  flash (U6)   SCK PB13 ok | MOSI PB14 clash | MISO PB15 clash
//   SPI3  OSD  (U8)    SCK PC10 ok | MOSI PC11 clash | MISO PB5  clash
//
// e.g. W25Q128JV pad 5 is DI, an input the MCU must drive, and it sits on
// PB14 -- which hardware SPI2 can only ever make an input. Same contention
// as the IMU. SPI3 has no escape either: its real MOSI (PC12) is taken by
// UART5_TX, and the PB3/PB4/PB5 alternate mapping is transposed too.
//
// Consequence: a blackbox-logging driver for the flash, and any OSD driver,
// both need the same bit-banged master the IMU uses. Neither is written yet,
// so nothing is broken today -- but do not reach for hardware SPI.
inline constexpr bool kHardwareSpiUsable = false;

// --- Barometer: BMP390 (IC9) on I2C1 -----------------------------------
inline constexpr Pin kI2cScl{Port::B, 8};  // net /I2C_SCL, pad 139
inline constexpr Pin kI2cSda{Port::B, 9};  // net /I2C_SDA, pad 140

// Strapping, read from the placed-part list (outputs/assembly):
//   R126 (0R, FITTED)      SDO -> 3V3  => 7-bit I2C address 0x77
//   R127 (0R, not fitted)  SDO -> GND  would have given 0x76
//   R128 (0R, NOT FITTED)  CSB -> 3V3
//   Bus pull-ups: R103/R104 (2k7, fitted). R124/R129 (4k7) not fitted.
//
// CAUTION: R128 is not fitted, so the BMP390's CSB pin is left floating and
// the part has no defined interface mode. CSB must be strapped high (it is
// brought out on J152.1) before the barometer will answer on I2C. Confirm
// this at bring-up before writing off a non-responding sensor.
inline constexpr uint8_t kBaroI2cAddress = 0x77;
inline constexpr bool kBaroCsbStrapFitted = false;

// The magnetometer (MMC5983MA, IC10) is on a separate /I2C_*_2 bus inside the
// GPS power island and is NOT connected to the F405. There is therefore no
// heading reference available to this MCU; yaw is gyro-only and will drift.
inline constexpr bool kMagnetometerOnMcuBus = false;

// --- Motor outputs, nets /S1../S4 --------------------------------------
// All four are DMA-capable, so DShot is available on every channel.
//   S1 PA9  TIM1_CH2 (AF1)
//   S2 PA8  TIM1_CH1 (AF1)
//   S3 PC9  TIM8_CH4 (AF3)
//   S4 PC8  TIM8_CH3 (AF3)
//
// /S1../S4 terminate on solder pads (S1,S2 -> J122.6/7; S3,S4 -> J129.1/2).
// The four onboard ESC MCUs (U1, U10, U11, U12 -- STM32F051C8T6) each take
// their signal input on their own PA2, which lands on a separate pad
// (J39.1, J205.1, J248.1, J291.1). No net joins the two sides: the flight
// controller is NOT connected to the onboard ESCs on copper, and the four
// F051s have no firmware anywhere in this repository. Commanding a motor
// therefore needs either hand wiring plus ESC firmware, or external ESCs.
inline constexpr Pin kMotor1{Port::A, 9};   // pad 101
inline constexpr Pin kMotor2{Port::A, 8};   // pad 100
inline constexpr Pin kMotor3{Port::C, 9};   // pad 99
inline constexpr Pin kMotor4{Port::C, 8};   // pad 98

// --- UARTs -------------------------------------------------------------
// USART1 PB6/PB7   /CRX_UART1_*    control receiver (CRSF)
// USART2 PA2/PA3   /ESC_UART2_*    ESC telemetry
// USART3 PB10/PB11 /BLE_UART*      NINA-B416 BLE module
// UART4  PA0/PA1   /VTX_UART4_*    VTX control
// UART5  PC12/PD2  /UART5_*        spare -> used here for ground telemetry
// USART6 PC6/PC7   /GPS_UART*      -> J129 pads only, NOT the onboard GPS
//
// The onboard SAM-M10Q (ANT2) sits in the GPS power island and drives the
// separate nets /GPS_UARTTX_2 and /GPS_UARTRX_2 (plus a DDC/I2C port on
// /GPS_I2C_*), all of which terminate on J164/J166. Its timepulse
// (/GPS_EXTI1) lands on J160. None of these reach the F405. To use the
// onboard receiver, J129 must be wired across to J164 by hand; otherwise
// USART6 is simply a spare port on a header.
inline constexpr Pin kRxUartTx{Port::B, 6};
inline constexpr Pin kRxUartRx{Port::B, 7};

inline constexpr Pin kTelemetryUartTx{Port::C, 12};
inline constexpr Pin kTelemetryUartRx{Port::D, 2};

inline constexpr Pin kGpsUartTx{Port::C, 6};
inline constexpr Pin kGpsUartRx{Port::C, 7};

inline constexpr Pin kEscTelemetryTx{Port::A, 2};
inline constexpr Pin kEscTelemetryRx{Port::A, 3};

// --- Analogue sensing --------------------------------------------------
inline constexpr Pin kAdcVbat{Port::C, 1};  // net /ADC_VBAT, pad 27
inline constexpr Pin kAdcRssi{Port::C, 2};  // net /ADC_RSSI, pad 28
inline constexpr Pin kAdcCurrent{Port::C, 3};  // net /ADC_CURR, pad 29

// --- Indicators --------------------------------------------------------
inline constexpr Pin kLed1{Port::B, 3};   // net /LED1_MCU, pad 133
// pad 7 is PC13, not PA15. PA15 carries /OSD_CS_Translated, so driving the
// buzzer from PA15 would toggle the OSD chip select instead.
inline constexpr Pin kBuzzer{Port::C, 13};  // net /BUZZER, pad 7
inline constexpr Pin kLed2{Port::C, 14};    // net /LED2_MCU, pad 8
inline constexpr Pin kLed3{Port::C, 15};    // net /LED3_MCU, pad 9

// --- Loop timing -------------------------------------------------------
inline constexpr uint32_t kControlLoopHz = 1000u;
inline constexpr uint32_t kControlLoopPeriodUs = 1000000u / kControlLoopHz;

}  // namespace board
}  // namespace fc
