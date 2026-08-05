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
// Baro:    BMP390 (ref IC9) on I2C1
// GPS:     u-blox SAM-M10Q (ref ANT2) on USART6
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
inline constexpr bool kImuInterruptAvailable = false;

// --- Barometer: BMP390 (IC9) on I2C1 -----------------------------------
inline constexpr Pin kI2cScl{Port::B, 8};  // net /I2C_SCL, pad 139
inline constexpr Pin kI2cSda{Port::B, 9};  // net /I2C_SDA, pad 140

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
// USART6 PC6/PC7   /GPS_UART*      SAM-M10Q GPS
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
inline constexpr Pin kBuzzer{Port::A, 15};  // net /BUZZER, pad 7

// --- Loop timing -------------------------------------------------------
inline constexpr uint32_t kControlLoopHz = 1000u;
inline constexpr uint32_t kControlLoopPeriodUs = 1000000u / kControlLoopHz;

}  // namespace board
}  // namespace fc
