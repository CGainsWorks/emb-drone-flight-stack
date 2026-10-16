#include "fc/drivers/mmc5983.hpp"

#include <cmath>

namespace fc {
namespace mmc5983 {

void decode(const uint8_t r[7], int32_t out[3]) {
    // r[6] holds the two least significant bits of each axis: X in 7:6,
    // Y in 5:4, Z in 3:2.
    for (int axis = 0; axis < 3; ++axis) {
        const uint32_t msb = r[2 * axis];
        const uint32_t mid = r[2 * axis + 1];
        const uint32_t lsb = (static_cast<uint32_t>(r[6]) >> (6 - 2 * axis)) & 0x3u;
        out[axis] = static_cast<int32_t>((msb << 10) | (mid << 2) | lsb);
    }
}

}  // namespace mmc5983

float tilt_compensated_heading(const Vec3& m, float roll, float pitch) {
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    // Rotate the body field back to a level frame (undo roll, then pitch).
    const float xh = m.x * cp + (m.y * sr + m.z * cr) * sp;
    const float yh = m.y * cr - m.z * sr;
    return std::atan2(-yh, xh);
}

bool Mmc5983::init() {
    initialised_ = false;

    if (!bus_.write_register(address_, mmc5983::kRegControl1, mmc5983::kCtrl1SoftReset)) {
        return false;
    }
    clock_.delay_us(10000);

    uint8_t id = 0;
    if (!bus_.read_registers(address_, mmc5983::kRegProductId, &id, 1) ||
        id != mmc5983::kProductId) {
        return false;
    }
    if (!bus_.write_register(address_, mmc5983::kRegControl1, mmc5983::kCtrl1Bandwidth100Hz)) {
        return false;
    }

    if (!calibrate_offset()) {
        return false;
    }
    initialised_ = true;
    return true;
}

bool Mmc5983::measure(int32_t out[3]) {
    if (!bus_.write_register(address_, mmc5983::kRegControl0, mmc5983::kCtrl0TakeMeasurement)) {
        return false;
    }
    // 8 ms conversion at the 100 Hz bandwidth setting; poll rather than
    // sleep the whole time and give up after a generous bound.
    bool done = false;
    for (int i = 0; i < 20 && !done; ++i) {
        clock_.delay_us(1000);
        uint8_t status = 0;
        if (!bus_.read_registers(address_, mmc5983::kRegStatus, &status, 1)) {
            return false;
        }
        done = (status & mmc5983::kStatusMeasDone) != 0u;
    }
    if (!done) {
        return false;
    }
    uint8_t raw[7] = {};
    if (!bus_.read_registers(address_, mmc5983::kRegXout0, raw, sizeof(raw))) {
        return false;
    }
    int32_t counts[3];
    mmc5983::decode(raw, counts);
    for (int i = 0; i < 3; ++i) {
        out[i] = counts[i] - mmc5983::kNullCounts;
    }
    return true;
}

bool Mmc5983::calibrate_offset() {
    int32_t set_counts[3];
    int32_t reset_counts[3];

    if (!bus_.write_register(address_, mmc5983::kRegControl0, mmc5983::kCtrl0Reset)) {
        return false;
    }
    clock_.delay_us(1000);
    if (!measure(reset_counts)) {
        return false;
    }

    // Finish in the SET state, which is the polarity read() assumes.
    if (!bus_.write_register(address_, mmc5983::kRegControl0, mmc5983::kCtrl0Set)) {
        return false;
    }
    clock_.delay_us(1000);
    if (!measure(set_counts)) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        offset_[i] = (set_counts[i] + reset_counts[i]) / 2;
    }
    return true;
}

bool Mmc5983::read(Vec3& gauss) {
    if (!initialised_) {
        return false;
    }
    int32_t c[3];
    if (!measure(c)) {
        return false;
    }
    gauss = Vec3{static_cast<float>(c[0] - offset_[0]) / mmc5983::kCountsPerGauss,
                 static_cast<float>(c[1] - offset_[1]) / mmc5983::kCountsPerGauss,
                 static_cast<float>(c[2] - offset_[2]) / mmc5983::kCountsPerGauss};
    return true;
}

}  // namespace fc
