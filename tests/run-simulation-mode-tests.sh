#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
deps="$root/.pio/libdeps/custom_f411ce"
build="$(mktemp -d)"
trap 'rm -rf "$build"' EXIT

g++ -std=c++20 -Wall -Wextra -Werror \
    -DALACER_SIMULATION -DALACER_SIMULATION_SPEED=2 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$root/tests/host" -I"$root/src" \
    -isystem "$deps/ArduinoEigen/ArduinoEigen" \
    "$root/tests/SimulationModeTest.cpp" \
    "$root/src/simMode/SimRocket.cpp" \
    "$root/src/simMode/physics/RigidBody.cpp" \
    "$root/src/helpers/coordinates/IMURocketCoordinateConverter.cpp" \
    -o "$build/simulation-mode-tests"
"$build/simulation-mode-tests"