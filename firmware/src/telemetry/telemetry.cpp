#include "fc/telemetry/telemetry.hpp"

#include <cstring>

#include "fc/drivers/crsf.hpp"
#include "fc/hal/clock.hpp"

namespace fc {

void TelemetryTransmitter::reset() {
    pending_len_ = 0;
    pending_pos_ = 0;
    have_sent_attitude_ = false;
    have_sent_battery_ = false;
    frames_sent_ = 0;
    frames_dropped_ = 0;
}

void TelemetryTransmitter::flush() {
    if (pending_pos_ >= pending_len_) {
        return;
    }
    const std::size_t remaining = pending_len_ - pending_pos_;
    const std::size_t written = uart_.write(&pending_[pending_pos_], remaining);
    pending_pos_ += written;

    if (pending_pos_ >= pending_len_) {
        // Whole frame is out.
        pending_len_ = 0;
        pending_pos_ = 0;
        ++frames_sent_;
    }
}

bool TelemetryTransmitter::queue(const uint8_t* data, std::size_t len) {
    if (busy() || len == 0 || len > kBufferSize) {
        // Still draining the previous frame. Drop this one rather than
        // corrupting the stream; telemetry is best-effort and must never
        // interfere with the control loop.
        ++frames_dropped_;
        return false;
    }
    std::memcpy(pending_, data, len);
    pending_len_ = len;
    pending_pos_ = 0;
    return true;
}

void TelemetryTransmitter::update(uint32_t now_us, const TelemetrySnapshot& snapshot) {
    // Always try to make progress on whatever is already queued first.
    flush();

    if (busy()) {
        return;
    }

    const bool attitude_due =
        !have_sent_attitude_ ||
        elapsed_us(now_us, last_attitude_us_) >= cfg_.attitude_period_us;
    const bool battery_due =
        !have_sent_battery_ ||
        elapsed_us(now_us, last_battery_us_) >= cfg_.battery_period_us;

    uint8_t frame[kBufferSize];

    // Battery first when both are due: it is the slower stream, so letting it
    // win avoids it being starved by the faster attitude updates.
    if (battery_due) {
        const std::size_t n = crsf::build_battery_frame(
            snapshot.battery_voltage, snapshot.battery_current, snapshot.used_mah,
            snapshot.battery_remaining_pct, frame, sizeof(frame));
        if (n > 0 && queue(frame, n)) {
            last_battery_us_ = now_us;
            have_sent_battery_ = true;
            flush();
        }
        return;
    }

    if (attitude_due) {
        const std::size_t n = crsf::build_attitude_frame(
            snapshot.pitch_rad, snapshot.roll_rad, snapshot.yaw_rad, frame,
            sizeof(frame));
        if (n > 0 && queue(frame, n)) {
            last_attitude_us_ = now_us;
            have_sent_attitude_ = true;
            flush();
        }
    }
}

}  // namespace fc
