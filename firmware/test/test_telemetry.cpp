#include <vector>

#include "fc/drivers/crsf.hpp"
#include "fc/telemetry/telemetry.hpp"
#include "mocks/mock_spi.hpp"
#include "test_framework.hpp"

using namespace fc;
using fc::testing_mocks::MockUart;

namespace {

TelemetrySnapshot snapshot() {
    TelemetrySnapshot s;
    s.roll_rad = 0.1f;
    s.pitch_rad = -0.2f;
    s.yaw_rad = 1.0f;
    s.battery_voltage = 16.8f;
    s.battery_current = 12.5f;
    s.used_mah = 450;
    s.battery_remaining_pct = 78;
    return s;
}

// Walk a byte stream and check every frame in it is well formed. Returns the
// number of frames found, or -1 if the stream is malformed.
int validate_stream(const std::vector<uint8_t>& tx) {
    std::size_t i = 0;
    int frames = 0;
    while (i < tx.size()) {
        if (i + 2 > tx.size()) {
            return -1;
        }
        const uint8_t len = tx[i + 1];
        if (len < 2 || len > crsf::kMaxFrameLength) {
            return -1;
        }
        const std::size_t total = static_cast<std::size_t>(len) + 2u;
        if (i + total > tx.size()) {
            return -1;  // truncated frame in the stream
        }
        const uint8_t expected = crsf::crc8(&tx[i + 2], static_cast<std::size_t>(len) - 1u);
        if (tx[i + total - 1] != expected) {
            return -1;
        }
        ++frames;
        i += total;
    }
    return frames;
}

int count_frames_of_type(const std::vector<uint8_t>& tx, uint8_t type) {
    std::size_t i = 0;
    int n = 0;
    while (i + 2 <= tx.size()) {
        const uint8_t len = tx[i + 1];
        const std::size_t total = static_cast<std::size_t>(len) + 2u;
        if (i + total > tx.size()) {
            break;
        }
        if (tx[i + 2] == type) {
            ++n;
        }
        i += total;
    }
    return n;
}

}  // namespace

TEST(telemetry_sends_something_on_the_first_update) {
    MockUart uart;
    TelemetryTransmitter tx{uart};
    tx.update(0, snapshot());
    CHECK(!uart.tx.empty());
    CHECK(validate_stream(uart.tx) >= 1);
}

TEST(telemetry_frames_are_always_well_formed) {
    MockUart uart;
    TelemetryTransmitter tx{uart};
    for (uint32_t t = 0; t < 5000000u; t += 1000u) {
        tx.update(t, snapshot());
    }
    const int frames = validate_stream(uart.tx);
    CHECK(frames > 0);
    // -1 would mean a torn or corrupt frame reached the wire.
    CHECK(frames != -1);
}

TEST(telemetry_respects_the_attitude_rate) {
    MockUart uart;
    TelemetryTransmitter tx{uart};
    TelemetryConfig cfg;
    cfg.attitude_period_us = 100000;  // 10 Hz
    cfg.battery_period_us = 10000000; // effectively off for this test
    tx.set_config(cfg);

    // Run for one second at 1 kHz.
    for (uint32_t t = 0; t < 1000000u; t += 1000u) {
        tx.update(t, snapshot());
    }

    const int attitude_frames =
        count_frames_of_type(uart.tx, crsf::kFrameTypeAttitude);
    // 10 Hz for 1 s, allowing for the initial frame and boundary effects.
    CHECK(attitude_frames >= 9);
    CHECK(attitude_frames <= 12);
}

TEST(telemetry_respects_the_battery_rate) {
    MockUart uart;
    TelemetryTransmitter tx{uart};
    TelemetryConfig cfg;
    cfg.attitude_period_us = 100000;
    cfg.battery_period_us = 500000;  // 2 Hz
    tx.set_config(cfg);

    for (uint32_t t = 0; t < 1000000u; t += 1000u) {
        tx.update(t, snapshot());
    }

    const int battery_frames =
        count_frames_of_type(uart.tx, crsf::kFrameTypeBatterySensor);
    CHECK(battery_frames >= 2);
    CHECK(battery_frames <= 4);
}

TEST(telemetry_sends_both_frame_types) {
    MockUart uart;
    TelemetryTransmitter tx{uart};
    for (uint32_t t = 0; t < 2000000u; t += 1000u) {
        tx.update(t, snapshot());
    }
    CHECK(count_frames_of_type(uart.tx, crsf::kFrameTypeAttitude) > 0);
    CHECK(count_frames_of_type(uart.tx, crsf::kFrameTypeBatterySensor) > 0);
}

TEST(telemetry_encodes_the_snapshot_values) {
    MockUart uart;
    TelemetryTransmitter tx{uart};
    TelemetryConfig cfg;
    cfg.attitude_period_us = 1000000;
    cfg.battery_period_us = 1000;
    tx.set_config(cfg);

    TelemetrySnapshot s = snapshot();
    s.battery_voltage = 15.4f;
    s.battery_remaining_pct = 63;
    tx.update(0, s);

    // First frame out should be the battery frame.
    CHECK(uart.tx.size() >= 12u);
    CHECK_EQ(static_cast<int>(uart.tx[2]), static_cast<int>(crsf::kFrameTypeBatterySensor));
    const uint16_t volts = static_cast<uint16_t>((uart.tx[3] << 8) | uart.tx[4]);
    CHECK_EQ(static_cast<int>(volts), 154);
    CHECK_EQ(static_cast<int>(uart.tx[10]), 63);
}

TEST(telemetry_never_blocks_when_the_port_is_full) {
    // A UART that accepts nothing at all must not hang the caller or corrupt
    // internal state.
    MockUart uart;
    uart.accept_limit = 0;
    TelemetryTransmitter tx{uart};

    for (uint32_t t = 0; t < 1000000u; t += 1000u) {
        tx.update(t, snapshot());
    }
    CHECK(uart.tx.empty());
    CHECK_EQ(static_cast<int>(tx.frames_sent()), 0);
    // It should still be holding the frame, ready to resume.
    CHECK(tx.busy());
}

TEST(telemetry_resumes_after_a_partial_write) {
    // The realistic case: the port drains a few bytes at a time.
    MockUart uart;
    uart.accept_limit = 3;
    TelemetryTransmitter tx{uart};

    for (uint32_t t = 0; t < 2000000u; t += 1000u) {
        tx.update(t, snapshot());
    }

    CHECK(tx.frames_sent() > 0u);
    // Despite being dribbled out three bytes at a time, every frame on the wire
    // must still be intact and in order.
    CHECK(validate_stream(uart.tx) > 0);
}

TEST(telemetry_recovers_when_a_blocked_port_frees_up) {
    MockUart uart;
    uart.accept_limit = 0;
    TelemetryTransmitter tx{uart};

    for (uint32_t t = 0; t < 100000u; t += 1000u) {
        tx.update(t, snapshot());
    }
    CHECK(uart.tx.empty());

    uart.accept_limit = -1;  // port frees up
    for (uint32_t t = 100000u; t < 300000u; t += 1000u) {
        tx.update(t, snapshot());
    }

    CHECK(tx.frames_sent() > 0u);
    CHECK(validate_stream(uart.tx) > 0);
}

TEST(telemetry_drops_rather_than_tearing_when_congested) {
    // Under congestion frames must be dropped whole. A partially written frame
    // followed by a different frame would desynchronise the receiver.
    MockUart uart;
    uart.accept_limit = 1;
    TelemetryTransmitter tx{uart};

    for (uint32_t t = 0; t < 3000000u; t += 1000u) {
        tx.update(t, snapshot());
    }

    // Whatever made it out is well formed.
    CHECK(validate_stream(uart.tx) != -1);
}

TEST(telemetry_handles_a_timer_wrap) {
    // Same 32-bit wrap hazard as the failsafe timeout.
    MockUart uart;
    TelemetryTransmitter tx{uart};

    const uint32_t start = 0xFFFF0000u;
    for (uint32_t i = 0; i < 2000u; ++i) {
        tx.update(start + i * 1000u, snapshot());
    }
    const std::size_t before_wrap = uart.tx.size();

    // Cross the wrap point.
    for (uint32_t i = 0; i < 2000u; ++i) {
        tx.update(static_cast<uint32_t>(start + 2000000u + i * 1000u), snapshot());
    }

    // Telemetry must keep flowing across the wrap, not stall for 71 minutes.
    CHECK(uart.tx.size() > before_wrap);
    CHECK(validate_stream(uart.tx) != -1);
}

TEST(telemetry_reset_clears_state) {
    MockUart uart;
    TelemetryTransmitter tx{uart};
    for (uint32_t t = 0; t < 100000u; t += 1000u) {
        tx.update(t, snapshot());
    }
    CHECK(tx.frames_sent() > 0u);

    tx.reset();
    CHECK_EQ(static_cast<int>(tx.frames_sent()), 0);
    CHECK(!tx.busy());
}

TEST(telemetry_output_parses_with_the_crsf_parser) {
    // Closing the loop: feed our own transmitted stream into the receiver-side
    // parser and confirm it accepts every frame's CRC.
    MockUart uart;
    TelemetryTransmitter tx{uart};
    for (uint32_t t = 0; t < 2000000u; t += 1000u) {
        tx.update(t, snapshot());
    }

    crsf::Parser parser;
    parser.push(uart.tx.data(), uart.tx.size());
    CHECK(parser.frames_decoded() > 0u);
    CHECK_EQ(static_cast<int>(parser.crc_errors()), 0);
}

TEST_MAIN()
