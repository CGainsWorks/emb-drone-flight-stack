// Firmware entry point for FCSTMV1.05.
//
// This file is the only place where the portable control code meets the STM32.
// Everything it constructs below the HAL line is board-specific; everything
// above it is the same code the host test suite exercises.
//
// NOT YET RUN ON HARDWARE. The boards are unassembled. This compiles and links
// for the target, but no line of it has been executed on silicon.

#include "fc/config/board_v105.hpp"
#include "fc/control/flight_controller.hpp"
#include "fc/drivers/crsf.hpp"
#include "fc/drivers/icm42688.hpp"
#include "fc/sensors/battery.hpp"
#include "fc/telemetry/telemetry.hpp"
#include "platform/stm32/adc.hpp"
#include "platform/stm32/bitbang_spi_stm32.hpp"
#include "platform/stm32/gpio.hpp"
#include "platform/stm32/pwm_motors.hpp"
#include "platform/stm32/registers.hpp"
#include "platform/stm32/system.hpp"
#include "platform/stm32/uart_stm32.hpp"

namespace {

using namespace fc;

// --- board wiring -------------------------------------------------------
// Pin numbers come from config/board_v105.hpp, which was derived from the PCB
// netlist. See docs/PINOUT.md.

stm32::Stm32GpioPin g_imu_sck{stm32::gpio(stm32::kGpioABase), 5};   // PA5
stm32::Stm32GpioPin g_imu_sdi{stm32::gpio(stm32::kGpioABase), 6};   // PA6 -> IMU SDI
stm32::Stm32GpioPin g_imu_sdo{stm32::gpio(stm32::kGpioABase), 7};   // PA7 <- IMU SDO
stm32::Stm32GpioPin g_imu_cs{stm32::gpio(stm32::kGpioABase), 4};    // PA4
stm32::Stm32GpioPin g_led{stm32::gpio(stm32::kGpioBBase), 3};       // PB3

// PA6 is an OUTPUT and PA7 an INPUT here, which is exactly the arrangement the
// hardware SPI1 peripheral cannot produce on this part.
stm32::BitBangSpiStm32 g_spi{g_imu_sck, g_imu_sdi, g_imu_sdo, g_imu_cs};

stm32::Stm32Clock g_clock;
stm32::PwmMotorOutput g_motors;

// USART1 on APB2 (84 MHz) for the CRSF receiver; UART5 on APB1 (42 MHz) for
// ground telemetry.
stm32::Stm32Uart g_rx_uart{stm32::kUsart1Base, 84000000u};
stm32::Stm32Uart g_telemetry_uart{stm32::kUart5Base, 42000000u};

stm32::Stm32Adc g_adc;
BatteryMonitor g_battery;

Icm42688 g_imu{g_spi, g_clock};
FlightController g_fc{g_imu, g_motors, g_clock};
crsf::Parser g_crsf;
TelemetryTransmitter g_telemetry{g_telemetry_uart};

constexpr uint32_t kCrsfBaud = 420000u;
constexpr uint32_t kTelemetryBaud = 115200u;

void configure_pins() {
    stm32::enable_gpio_clocks();

    // IMU software SPI.
    stm32::configure_output(stm32::gpio(stm32::kGpioABase), 4);  // CS
    stm32::configure_output(stm32::gpio(stm32::kGpioABase), 5);  // SCK
    stm32::configure_output(stm32::gpio(stm32::kGpioABase), 6);  // SDI (MCU out)
    stm32::configure_input(stm32::gpio(stm32::kGpioABase), 7,
                           stm32::Pull::Up);                     // SDO (MCU in)

    // Status LED.
    stm32::configure_output(stm32::gpio(stm32::kGpioBBase), 3);

    // USART1 (PB6 TX, PB7 RX) is AF7.
    stm32::configure_alternate(stm32::gpio(stm32::kGpioBBase), 6, 7);
    stm32::configure_alternate(stm32::gpio(stm32::kGpioBBase), 7, 7);

    // UART5 is PC12 TX / PD2 RX, AF8.
    stm32::configure_alternate(stm32::gpio(stm32::kGpioCBase), 12, 8);
    stm32::configure_alternate(stm32::gpio(stm32::kGpioDBase), 2, 8);
}

void enable_uart_clocks() {
    stm32::set_bits(stm32::rcc()->APB2ENR, stm32::rcc_apb2enr::kUsart1);
    stm32::set_bits(stm32::rcc()->APB1ENR, stm32::rcc_apb1enr::kUart5);
}

void set_led(bool on) { g_led.write(on); }

// Slow blink while something is wrong. Never returns: the motors have not been
// started at this point, so parking here is the safe outcome.
[[noreturn]] void fault_loop() {
    g_motors.disarm();
    while (true) {
        set_led(true);
        g_clock.delay_us(150000);
        set_led(false);
        g_clock.delay_us(150000);
    }
}

void pump_receiver() {
    uint8_t buf[64];
    std::size_t n = g_rx_uart.read(buf, sizeof(buf));
    while (n > 0) {
        for (std::size_t i = 0; i < n; ++i) {
            if (g_crsf.push(buf[i])) {
                crsf::RcChannels channels;
                if (g_crsf.take(channels)) {
                    g_fc.on_rc_frame(channels, g_clock.micros());
                }
            }
        }
        n = g_rx_uart.read(buf, sizeof(buf));
    }

    // The idle gap between bursts bounds the damage from a truncated frame.
    if (g_rx_uart.take_idle_flag()) {
        g_crsf.notify_idle();
    }
}

TelemetrySnapshot build_snapshot() {
    TelemetrySnapshot s;
    const EulerAngles att = g_fc.estimator().euler();
    s.roll_rad = att.roll;
    s.pitch_rad = att.pitch;
    s.yaw_rad = att.yaw;
    s.battery_voltage = g_battery.voltage();
    s.battery_current = g_battery.current();
    s.used_mah = static_cast<uint32_t>(g_battery.used_mah());
    s.battery_remaining_pct = g_battery.remaining_percent();
    s.arm_state = g_fc.arming().state();
    return s;
}

}  // namespace

int main() {
    if (!stm32::configure_clocks()) {
        // Without the PLL every timing constant below is wrong, so do not run.
        while (true) {
        }
    }
    stm32::start_microsecond_timer();

    configure_pins();
    enable_uart_clocks();

    g_motors.begin();
    g_motors.disarm();

    g_adc.begin();

    g_rx_uart.begin(kCrsfBaud, /*enable_rx_interrupt=*/true);
    g_telemetry_uart.begin(kTelemetryBaud, /*enable_rx_interrupt=*/false);
    stm32::set_usart1_instance(&g_rx_uart);
    stm32::nvic_enable_irq(stm32::kIrqUsart1);

    if (!g_fc.init()) {
        // The IMU did not answer. Most likely causes on this board: the
        // bit-banged SPI timing, or the PA6/PA7 routing described in
        // docs/PINOUT.md.
        fault_loop();
    }

    if (!g_fc.calibrate_gyro()) {
        // Calibration requires the airframe to be still.
        fault_loop();
    }

    // Let the ADC fill all three channels, then latch the cell count once while
    // the motors are stopped and the pack is unloaded.
    for (int i = 0; i < 64; ++i) {
        g_adc.poll();
        g_battery.update(g_adc.vbat_counts(), g_adc.current_counts(), 0.001f);
        g_clock.delay_us(1000);
    }
    g_battery.detect_cell_count();

    set_led(true);

    uint32_t next_cycle = g_clock.micros();
    uint32_t blink_timer = next_cycle;

    while (true) {
        // Fixed-rate loop. Unsigned arithmetic keeps this correct across the
        // 32-bit microsecond wrap.
        const uint32_t now = g_clock.micros();
        if (static_cast<uint32_t>(now - next_cycle) < board::kControlLoopPeriodUs) {
            continue;
        }
        next_cycle += board::kControlLoopPeriodUs;

        g_adc.poll();
        g_battery.update(g_adc.vbat_counts(), g_adc.current_counts(),
                         static_cast<float>(board::kControlLoopPeriodUs) * 1e-6f);
        g_fc.set_battery_voltage(g_battery.voltage());

        pump_receiver();
        g_fc.update(now);
        g_telemetry.update(now, build_snapshot());

        // Heartbeat: fast when armed, slow when disarmed.
        const uint32_t blink_period =
            (g_fc.arming().state() == ArmState::Armed) ? 100000u : 500000u;
        if (static_cast<uint32_t>(now - blink_timer) >= blink_period) {
            blink_timer = now;
            static bool led_state = false;
            led_state = !led_state;
            set_led(led_state);
        }
    }
}
