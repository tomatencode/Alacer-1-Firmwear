#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
build="$(mktemp -d)"
trap 'rm -rf "$build"' EXIT
deps="$root/.pio/libdeps/custom_f411ce"
if [[ ! -f "$deps/ArduinoEigen/ArduinoEigen.h" ]]; then
    echo "Install PlatformIO dependencies first (pio pkg install)." >&2
    exit 1
fi

# Servo declarations include this MCU header but these tests use no timer API.
printf '#pragma once\n' > "$build/HardwareTimer.h"
g++ -std=c++20 -Wall -Wextra -Werror -Wno-error=reorder \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$build" -I"$root/tests/host" -I"$root/src" \
    -isystem "$deps/ArduinoEigen/ArduinoEigen" \
    "$root/tests/SimRocketAerodynamicsTest.cpp" \
    "$root/src/simMode/SimRocket.cpp" "$root/src/simMode/physics/RigidBody.cpp" \
    -o "$build/sim-rocket-tests"
"$build/sim-rocket-tests"