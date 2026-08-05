#include <vector>

#include "fc/drivers/crsf.hpp"
#include "test_framework.hpp"

using namespace fc;
using namespace fc::crsf;

namespace {

// Build a well-formed RC channel frame around the given channel set.
std::vector<uint8_t> make_rc_frame(const RcChannels& ch) {
    std::vector<uint8_t> frame;
    frame.push_back(kAddressFlightController);
    frame.push_back(static_cast<uint8_t>(kRcPayloadBytes + 2));
    frame.push_back(kFrameTypeRcChannelsPacked);

    uint8_t payload[kRcPayloadBytes];
    pack_channels(ch, payload);
    for (std::size_t i = 0; i < kRcPayloadBytes; ++i) {
        frame.push_back(payload[i]);
    }

    // CRC covers type + payload.
    std::vector<uint8_t> crc_region;
    crc_region.push_back(kFrameTypeRcChannelsPacked);
    for (std::size_t i = 0; i < kRcPayloadBytes; ++i) {
        crc_region.push_back(payload[i]);
    }
    frame.push_back(crc8(crc_region.data(), crc_region.size()));
    return frame;
}

RcChannels ramp_channels() {
    RcChannels ch;
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        ch.channel[i] = static_cast<uint16_t>(kChannelMin + i * 100);
    }
    return ch;
}

}  // namespace

TEST(crsf_crc_is_zero_for_empty_input) { CHECK_EQ(static_cast<int>(crc8(nullptr, 0)), 0); }

TEST(crsf_crc_detects_single_bit_changes) {
    const uint8_t a[] = {0x16, 0x01, 0x02, 0x03};
    uint8_t b[] = {0x16, 0x01, 0x02, 0x03};
    b[2] ^= 0x01;
    CHECK(crc8(a, sizeof(a)) != crc8(b, sizeof(b)));
}

TEST(crsf_channel_pack_unpack_round_trips) {
    const RcChannels in = ramp_channels();
    uint8_t payload[kRcPayloadBytes];
    pack_channels(in, payload);

    RcChannels out;
    unpack_channels(payload, out);

    for (std::size_t i = 0; i < kChannelCount; ++i) {
        CHECK_EQ(static_cast<int>(out.channel[i]), static_cast<int>(in.channel[i]));
    }
}

TEST(crsf_channel_pack_handles_full_range) {
    RcChannels in;
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        in.channel[i] = (i % 2 == 0) ? 0x7FF : 0x000;
    }
    uint8_t payload[kRcPayloadBytes];
    pack_channels(in, payload);
    RcChannels out;
    unpack_channels(payload, out);
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        CHECK_EQ(static_cast<int>(out.channel[i]), static_cast<int>(in.channel[i]));
    }
}

TEST(crsf_parser_decodes_a_valid_frame) {
    const RcChannels expected = ramp_channels();
    const std::vector<uint8_t> frame = make_rc_frame(expected);

    Parser parser;
    const std::size_t frames = parser.push(frame.data(), frame.size());
    CHECK_EQ(static_cast<int>(frames), 1);
    CHECK(parser.has_channels());
    CHECK_EQ(static_cast<int>(parser.crc_errors()), 0);

    RcChannels got;
    CHECK(parser.take(got));
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        CHECK_EQ(static_cast<int>(got.channel[i]), static_cast<int>(expected.channel[i]));
    }
    // take() consumes.
    CHECK(!parser.take(got));
}

TEST(crsf_parser_rejects_a_corrupt_crc) {
    std::vector<uint8_t> frame = make_rc_frame(ramp_channels());
    frame.back() ^= 0xFF;  // break the CRC

    Parser parser;
    CHECK_EQ(static_cast<int>(parser.push(frame.data(), frame.size())), 0);
    CHECK(!parser.has_channels());
    CHECK_EQ(static_cast<int>(parser.crc_errors()), 1);
}

TEST(crsf_parser_rejects_a_corrupt_payload) {
    std::vector<uint8_t> frame = make_rc_frame(ramp_channels());
    frame[5] ^= 0x20;  // flip a payload bit, CRC no longer matches

    Parser parser;
    CHECK_EQ(static_cast<int>(parser.push(frame.data(), frame.size())), 0);
    CHECK_EQ(static_cast<int>(parser.crc_errors()), 1);
}

TEST(crsf_parser_resynchronises_after_garbage) {
    // Noise on the wire followed by a good frame: the good frame must still be
    // decoded. A parser that wedged here would mean permanent loss of control.
    std::vector<uint8_t> stream = {0x00, 0xFF, 0x13, 0xC8, 0x01, 0x77, 0xAB};
    const RcChannels expected = ramp_channels();
    const std::vector<uint8_t> good = make_rc_frame(expected);
    stream.insert(stream.end(), good.begin(), good.end());

    Parser parser;
    parser.push(stream.data(), stream.size());
    CHECK(parser.has_channels());

    RcChannels got;
    CHECK(parser.take(got));
    CHECK_EQ(static_cast<int>(got.channel[0]), static_cast<int>(expected.channel[0]));
}

TEST(crsf_truncated_frame_swallows_the_next_one_without_idle_resync) {
    // Documents the failure mode that motivates notify_idle(). A pure
    // byte-stream parser keeps counting through the gap, so the frame after a
    // truncated one is consumed as its tail and lost.
    const RcChannels expected = ramp_channels();
    const std::vector<uint8_t> good = make_rc_frame(expected);

    std::vector<uint8_t> stream(good.begin(), good.begin() + 10);
    stream.insert(stream.end(), good.begin(), good.end());

    Parser parser;
    parser.push(stream.data(), stream.size());
    CHECK(!parser.has_channels());
}

TEST(crsf_parser_recovers_after_a_truncated_frame_with_idle_resync) {
    // With the idle-gap notification the damage is bounded to the corrupted
    // frame: the very next frame decodes normally.
    const RcChannels expected = ramp_channels();
    const std::vector<uint8_t> good = make_rc_frame(expected);

    Parser parser;
    // A frame cut short by noise.
    parser.push(good.data(), 10);
    // The line goes quiet between bursts.
    parser.notify_idle();
    CHECK_EQ(static_cast<int>(parser.truncated_frames()), 1);

    // Next burst arrives intact.
    parser.push(good.data(), good.size());
    CHECK(parser.has_channels());

    RcChannels got;
    CHECK(parser.take(got));
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        CHECK_EQ(static_cast<int>(got.channel[i]), static_cast<int>(expected.channel[i]));
    }
    CHECK_EQ(static_cast<int>(parser.crc_errors()), 0);
}

TEST(crsf_idle_on_a_clean_boundary_is_harmless) {
    // Idle between well-formed frames must not count as truncation or disturb
    // decoding.
    const std::vector<uint8_t> good = make_rc_frame(ramp_channels());
    Parser parser;
    parser.push(good.data(), good.size());
    parser.notify_idle();
    CHECK_EQ(static_cast<int>(parser.truncated_frames()), 0);

    parser.push(good.data(), good.size());
    CHECK_EQ(static_cast<int>(parser.frames_decoded()), 2);
}

TEST(crsf_parser_survives_a_byte_at_a_time_delivery) {
    // UART interrupts deliver one byte at a time; that must work identically.
    const RcChannels expected = ramp_channels();
    const std::vector<uint8_t> frame = make_rc_frame(expected);

    Parser parser;
    bool got_frame = false;
    for (uint8_t b : frame) {
        if (parser.push(b)) {
            got_frame = true;
        }
    }
    CHECK(got_frame);
    RcChannels got;
    CHECK(parser.take(got));
    CHECK_EQ(static_cast<int>(got.channel[3]), static_cast<int>(expected.channel[3]));
}

TEST(crsf_parser_rejects_absurd_length_bytes) {
    // 0xC8 followed by an impossible length must not lock the parser up
    // waiting for bytes that will never come.
    Parser parser;
    parser.push(kAddressFlightController);
    parser.push(0xFF);  // illegal length
    CHECK(parser.length_errors() > 0u);

    const std::vector<uint8_t> good = make_rc_frame(ramp_channels());
    parser.push(good.data(), good.size());
    CHECK(parser.has_channels());
}

TEST(crsf_parser_handles_back_to_back_frames) {
    const RcChannels a = ramp_channels();
    RcChannels b;
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        b.channel[i] = kChannelMid;
    }

    std::vector<uint8_t> stream = make_rc_frame(a);
    const std::vector<uint8_t> second = make_rc_frame(b);
    stream.insert(stream.end(), second.begin(), second.end());

    Parser parser;
    CHECK_EQ(static_cast<int>(parser.push(stream.data(), stream.size())), 2);
    CHECK_EQ(static_cast<int>(parser.frames_decoded()), 2);

    // Latest wins.
    RcChannels got;
    CHECK(parser.take(got));
    CHECK_EQ(static_cast<int>(got.channel[0]), static_cast<int>(kChannelMid));
}

TEST(crsf_parser_never_overflows_on_random_noise) {
    // Fuzz: a long pseudo-random stream must not corrupt memory, hang or
    // produce a bogus "valid" frame count that outruns the input.
    Parser parser;
    uint32_t seed = 12345u;
    for (int i = 0; i < 200000; ++i) {
        seed = seed * 1103515245u + 12345u;
        parser.push(static_cast<uint8_t>((seed >> 16) & 0xFFu));
    }
    // Chance CRC hits are possible but must be rare, and nothing may crash.
    CHECK(parser.frames_decoded() < 2000u);
}

TEST(crsf_scaling_maps_the_stick_range) {
    CHECK_NEAR(to_bipolar(kChannelMid), 0.0f, 1e-6);
    CHECK_NEAR(to_bipolar(kChannelMax), 1.0f, 0.01f);
    CHECK_NEAR(to_bipolar(kChannelMin), -1.0f, 0.01f);

    CHECK_NEAR(to_unipolar(kChannelMin), 0.0f, 1e-6);
    CHECK_NEAR(to_unipolar(kChannelMax), 1.0f, 1e-6);
    CHECK_NEAR(to_unipolar(kChannelMid), 0.5f, 0.01f);
}

TEST(crsf_scaling_clamps_out_of_range_values) {
    // A receiver can emit slightly out-of-spec values; they must not become
    // out-of-range control demands.
    CHECK(to_bipolar(0) >= -1.0f);
    CHECK(to_bipolar(2047) <= 1.0f);
    CHECK(to_unipolar(0) >= 0.0f);
    CHECK(to_unipolar(2047) <= 1.0f);
}

TEST(crsf_switch_detection) {
    CHECK(!is_switch_high(kChannelMin));
    CHECK(is_switch_high(kChannelMax));
}

TEST(crsf_attitude_telemetry_frame_is_well_formed) {
    uint8_t buf[32];
    const std::size_t n = build_attitude_frame(0.1f, -0.2f, 1.5f, buf, sizeof(buf));
    CHECK_EQ(static_cast<int>(n), 10);  // addr + len + type + 6 + crc
    CHECK_EQ(static_cast<int>(buf[1]), 8);
    CHECK_EQ(static_cast<int>(buf[2]), static_cast<int>(kFrameTypeAttitude));
    // CRC must validate over type + payload.
    CHECK_EQ(static_cast<int>(buf[n - 1]), static_cast<int>(crc8(&buf[2], 7)));
}

TEST(crsf_attitude_telemetry_encodes_values_correctly) {
    uint8_t buf[32];
    build_attitude_frame(0.1f, -0.2f, 0.0f, buf, sizeof(buf));
    const int16_t pitch = static_cast<int16_t>((buf[3] << 8) | buf[4]);
    const int16_t roll = static_cast<int16_t>((buf[5] << 8) | buf[6]);
    CHECK_EQ(static_cast<int>(pitch), 1000);
    CHECK_EQ(static_cast<int>(roll), -2000);
}

TEST(crsf_battery_telemetry_frame_is_well_formed) {
    uint8_t buf[32];
    const std::size_t n = build_battery_frame(16.8f, 12.5f, 1234u, 87u, buf, sizeof(buf));
    CHECK_EQ(static_cast<int>(n), 12);
    CHECK_EQ(static_cast<int>(buf[2]), static_cast<int>(kFrameTypeBatterySensor));

    const uint16_t volts = static_cast<uint16_t>((buf[3] << 8) | buf[4]);
    CHECK_EQ(static_cast<int>(volts), 168);
    const uint32_t mah = (static_cast<uint32_t>(buf[7]) << 16) |
                         (static_cast<uint32_t>(buf[8]) << 8) | buf[9];
    CHECK_EQ(static_cast<int>(mah), 1234);
    CHECK_EQ(static_cast<int>(buf[10]), 87);
    CHECK_EQ(static_cast<int>(buf[n - 1]), static_cast<int>(crc8(&buf[2], 9)));
}

TEST(crsf_telemetry_refuses_undersized_buffers) {
    uint8_t small[4];
    CHECK_EQ(static_cast<int>(build_attitude_frame(0, 0, 0, small, sizeof(small))), 0);
    CHECK_EQ(static_cast<int>(build_battery_frame(0, 0, 0, 0, small, sizeof(small))), 0);
    CHECK_EQ(static_cast<int>(build_attitude_frame(0, 0, 0, nullptr, 32)), 0);
}

TEST(crsf_telemetry_frames_parse_back_with_valid_crc) {
    // Anything we transmit must satisfy the same CRC check a receiver applies.
    uint8_t buf[32];
    const std::size_t n = build_attitude_frame(0.3f, 0.4f, -0.5f, buf, sizeof(buf));
    const uint8_t len = buf[1];
    CHECK_EQ(static_cast<int>(crc8(&buf[2], len - 1u)), static_cast<int>(buf[n - 1]));
}

TEST_MAIN()
