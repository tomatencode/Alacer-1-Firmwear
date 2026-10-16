#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
deps="$root/.pio/libdeps/custom_f411ce"
build="$(mktemp -d)"
trap 'rm -rf "$build"' EXIT

g++ -std=c++20 -Wall -Wextra -Werror -Wno-unused-parameter \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$root/tests/host/pyro" -I"$root/tests/host" -I"$root/src" \
    -isystem "$deps/Embedded Template Library/include" \
    -isystem "$deps/ArduinoEigen/ArduinoEigen" \
    "$root/tests/ClockTimingTest.cpp" \
    "$root/src/rotationEstimation/RotationAccumulator.cpp" \
    "$root/src/helpers/coordinates/IMURocketCoordinateConverter.cpp" \
    "$root/src/hardwareIO/pyro/PyroArmManager.cpp" \
    "$root/src/hardwareIO/pyro/PyroChannel.cpp" \
    "$root/src/logManagement/StorageManager.cpp" \
    "$root/src/logManagement/LogManager.cpp" \
    "$root/src/logManagement/LogProtocol.cpp" \
    -o "$build/clock-tests"
"$build/clock-tests"

# Arduino time primitives belong only in the Clock adapter.
if grep -RInE '\b(millis|micros)[[:space:]]*\(' "$root/src" --exclude=Clock.hpp; then
    echo "Direct Arduino time call outside Clock.hpp" >&2
    exit 1
fi