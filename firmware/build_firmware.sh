#!/usr/bin/env bash
# Cross-compile the STM32F405 firmware image.
#
#   ./build_firmware.sh          configure and build
#   ./build_firmware.sh clean    wipe the build directory first
#
# Requires: arm-none-eabi-gcc / g++ / objcopy / size
set -euo pipefail

cd "$(dirname "$0")"

if [ "${1:-}" = "clean" ]; then
  rm -rf build-arm
fi

cmake -S . -B build-arm \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build-arm -j"$(nproc)"

echo
echo "Artifacts:"
ls -l build-arm/fc_firmware.elf build-arm/fc_firmware.bin build-arm/fc_firmware.hex 2>/dev/null || true
