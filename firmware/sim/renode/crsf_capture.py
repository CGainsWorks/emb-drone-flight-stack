"""Decode CRSF attitude frames from a raw UART capture.

Used by fc_firmware.robot. The emulated firmware sends CRSF telemetry on UART5
(interleaved with its plain-text diagnostics); Renode writes every byte to a
file backend and this module pulls the attitude frames back out.
"""

import math

# Frames start with an address byte: 0xC8 towards a flight controller, 0xEA
# towards the radio, which is what this firmware sends.
CRSF_ADDRESSES = (0xC8, 0xEA)
CRSF_TYPE_ATTITUDE = 0x1E


def _crc8_dvb_s2(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _be16(hi, lo):
    value = (hi << 8) | lo
    return value - 0x10000 if value & 0x8000 else value


def attitude_frames(path):
    """Return every CRC-valid attitude frame as (pitch, roll, yaw) in degrees."""
    with open(path, "rb") as handle:
        data = handle.read()
    frames = []
    i = 0
    while i + 4 <= len(data):
        if data[i] not in CRSF_ADDRESSES:
            i += 1
            continue
        length = data[i + 1]
        end = i + 2 + length
        if length < 2 or end > len(data):
            i += 1
            continue
        body = data[i + 2:end]
        if body[0] == CRSF_TYPE_ATTITUDE and len(body) == 8 and _crc8_dvb_s2(body[:-1]) == body[-1]:
            pitch, roll, yaw = (_be16(body[k], body[k + 1]) * 1e-4 for k in (1, 3, 5))
            frames.append(tuple(math.degrees(v) for v in (pitch, roll, yaw)))
            i = end
            continue
        i += 1
    return frames


def latest_roll_degrees(path):
    """Roll from the newest attitude frame in the capture."""
    frames = attitude_frames(path)
    if not frames:
        raise AssertionError("no CRC-valid CRSF attitude frame in " + path)
    return frames[-1][1]


def count_attitude_frames(path):
    return len(attitude_frames(path))
