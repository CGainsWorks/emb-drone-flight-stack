#!/usr/bin/env bash
# Build and run the host-side unit tests. No hardware required.
#
#   ./run_tests.sh          configure, build and run everything
#   ./run_tests.sh clean    wipe the build directory first
set -euo pipefail

cd "$(dirname "$0")"

if [ "${1:-}" = "clean" ]; then
  rm -rf build
fi

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j"$(nproc)"

cd build
ctest --output-on-failure
