#include "fc/drivers/dshot.hpp"
#include "test_framework.hpp"

using namespace fc;
using namespace fc::dshot;

TEST(dshot_crc_matches_hand_computation) {
    // payload = throttle 48, no telemetry -> 48 << 1 = 0x060
    // crc = (0x060 ^ 0x006 ^ 0x000) & 0xF = 0x66 & 0xF = 6
    CHECK_EQ(static_cast<int>(crc4(0x060)), 6);

    // payload = 0 -> crc 0
    CHECK_EQ(static_cast<int>(crc4(0x000)), 0);
}

TEST(dshot_crc_only_uses_twelve_bits) {
    // Bits above 12 must be ignored, not corrupt the CRC.
    CHECK_EQ(static_cast<int>(crc4(0x060)), static_cast<int>(crc4(0xF060)));
}

TEST(dshot_frame_layout_is_value_telemetry_crc) {
    const uint16_t frame = make_frame(48, false);
    // Top 11 bits are the value.
    CHECK_EQ(static_cast<int>(frame >> 5), 48);
    // Bit 4 is the telemetry request.
    CHECK_EQ(static_cast<int>((frame >> 4) & 0x1u), 0);
    // Bottom 4 bits are the CRC.
    CHECK_EQ(static_cast<int>(frame & 0x0Fu), 6);
    CHECK_EQ(static_cast<int>(frame), 0x0606);
}

TEST(dshot_frame_sets_telemetry_bit) {
    const uint16_t frame = make_frame(48, true);
    CHECK_EQ(static_cast<int>((frame >> 4) & 0x1u), 1);
    // Setting the telemetry bit changes the payload, so the CRC must change too.
    CHECK(( frame & 0x0Fu) != (make_frame(48, false) & 0x0Fu));
}

TEST(dshot_frame_crc_is_self_consistent_across_range) {
    // Recompute the CRC from the transmitted payload for every legal value and
    // confirm it matches the transmitted CRC. This is exactly the check the ESC
    // performs, so a mismatch here means the ESC would reject the frame.
    for (uint16_t v = 0; v <= kMaxThrottle; ++v) {
        for (int telem = 0; telem < 2; ++telem) {
            const uint16_t frame = make_frame(v, telem != 0);
            const uint16_t payload = static_cast<uint16_t>(frame >> 4);
            const uint8_t transmitted_crc = static_cast<uint8_t>(frame & 0x0Fu);
            CHECK_EQ(static_cast<int>(crc4(payload)), static_cast<int>(transmitted_crc));
        }
    }
}

TEST(dshot_frame_masks_out_of_range_values) {
    // 2048 does not fit in 11 bits; it must wrap to 0 rather than corrupt the
    // telemetry bit or the CRC field.
    const uint16_t frame = make_frame(2048, false);
    CHECK_EQ(static_cast<int>(frame >> 5), 0);
}

TEST(dshot_zero_throttle_is_a_genuine_motor_stop) {
    CHECK_EQ(static_cast<int>(throttle_to_value(0.0f)), static_cast<int>(kCmdMotorStop));
    CHECK_EQ(static_cast<int>(throttle_to_value(-0.5f)), static_cast<int>(kCmdMotorStop));
    // And the resulting frame must be the all-zero stop frame.
    CHECK_EQ(static_cast<int>(make_frame(throttle_to_value(0.0f), false)), 0);
}

TEST(dshot_throttle_maps_onto_the_valid_range) {
    // Anything above zero must land at or above the minimum spin value; a
    // command between 1 and 47 would be interpreted as an ESC command instead.
    CHECK_EQ(static_cast<int>(throttle_to_value(0.0001f)), static_cast<int>(kMinThrottle));
    CHECK_EQ(static_cast<int>(throttle_to_value(1.0f)), static_cast<int>(kMaxThrottle));
    CHECK_EQ(static_cast<int>(throttle_to_value(2.0f)), static_cast<int>(kMaxThrottle));

    const uint16_t mid = throttle_to_value(0.5f);
    CHECK(mid > kMinThrottle);
    CHECK(mid < kMaxThrottle);
    CHECK_NEAR(static_cast<float>(mid), 1047.5f, 1.5f);
}

TEST(dshot_throttle_is_monotonic) {
    uint16_t prev = throttle_to_value(0.0f);
    for (int i = 1; i <= 1000; ++i) {
        const uint16_t v = throttle_to_value(static_cast<float>(i) / 1000.0f);
        CHECK(v >= prev);
        prev = v;
    }
}

TEST(dshot_no_throttle_value_lands_in_the_command_region) {
    // Values 1..47 are reserved ESC commands. A throttle command must never
    // accidentally beep the ESC or, worse, trigger direction reversal.
    for (int i = 0; i <= 10000; ++i) {
        const uint16_t v = throttle_to_value(static_cast<float>(i) / 10000.0f);
        CHECK(v == kCmdMotorStop || v >= kMinThrottle);
    }
}

TEST(dshot_timing_for_600k_on_168mhz) {
    // 168 MHz / 600 kbit = 280 counts per bit.
    const Timing t = compute_timing(168000000u, Rate::Dshot600);
    CHECK_EQ(static_cast<int>(t.period_counts), 280);
    CHECK_EQ(static_cast<int>(t.t0h_counts), 105);  // 37.5%
    CHECK_EQ(static_cast<int>(t.t1h_counts), 210);  // 75%
}

TEST(dshot_timing_scales_with_rate) {
    const Timing t150 = compute_timing(168000000u, Rate::Dshot150);
    const Timing t300 = compute_timing(168000000u, Rate::Dshot300);
    const Timing t600 = compute_timing(168000000u, Rate::Dshot600);
    CHECK_EQ(static_cast<int>(t150.period_counts), 1120);
    CHECK_EQ(static_cast<int>(t300.period_counts), 560);
    CHECK_EQ(static_cast<int>(t600.period_counts), 280);
    // A 1 must always be longer than a 0, at every rate.
    CHECK(t150.t1h_counts > t150.t0h_counts);
    CHECK(t300.t1h_counts > t300.t0h_counts);
    CHECK(t600.t1h_counts > t600.t0h_counts);
}

TEST(dshot_timing_handles_degenerate_input) {
    const Timing t = compute_timing(0u, Rate::Dshot600);
    CHECK_EQ(static_cast<int>(t.period_counts), 0);
}

TEST(dshot_encode_writes_msb_first) {
    const Timing t = compute_timing(168000000u, Rate::Dshot600);
    uint16_t buf[kFrameBits] = {0};

    // 0x8000 -> only the most significant bit set.
    const std::size_t n = encode_frame(0x8000, t, buf, kFrameBits);
    CHECK_EQ(static_cast<int>(n), 16);
    CHECK_EQ(static_cast<int>(buf[0]), static_cast<int>(t.t1h_counts));
    for (std::size_t i = 1; i < kFrameBits; ++i) {
        CHECK_EQ(static_cast<int>(buf[i]), static_cast<int>(t.t0h_counts));
    }
}

TEST(dshot_encode_lsb_lands_last) {
    const Timing t = compute_timing(168000000u, Rate::Dshot600);
    uint16_t buf[kFrameBits] = {0};
    encode_frame(0x0001, t, buf, kFrameBits);
    CHECK_EQ(static_cast<int>(buf[15]), static_cast<int>(t.t1h_counts));
    CHECK_EQ(static_cast<int>(buf[0]), static_cast<int>(t.t0h_counts));
}

TEST(dshot_encode_round_trips_a_real_frame) {
    const Timing t = compute_timing(168000000u, Rate::Dshot600);
    uint16_t buf[kFrameBits] = {0};
    const uint16_t frame = make_frame(1046, false);
    encode_frame(frame, t, buf, kFrameBits);

    // Reconstruct the frame from the pulse widths.
    uint16_t decoded = 0;
    for (std::size_t i = 0; i < kFrameBits; ++i) {
        decoded = static_cast<uint16_t>(decoded << 1);
        if (buf[i] == t.t1h_counts) {
            decoded = static_cast<uint16_t>(decoded | 1u);
        }
    }
    CHECK_EQ(static_cast<int>(decoded), static_cast<int>(frame));
}

TEST(dshot_encode_rejects_undersized_buffer) {
    const Timing t = compute_timing(168000000u, Rate::Dshot600);
    uint16_t buf[8] = {0};
    CHECK_EQ(static_cast<int>(encode_frame(0xFFFF, t, buf, 8)), 0);
    CHECK_EQ(static_cast<int>(encode_frame(0xFFFF, t, nullptr, 16)), 0);
}

TEST_MAIN()
