#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
build="$(mktemp -d)"
deps="$root/.pio/libdeps/custom_f411ce"
trap 'rm -rf "$build"' EXIT

g++ -std=c++20 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$root/tests/host/pyro" -I"$root/tests/host" -I"$root/src" \
    -isystem "$deps/ArduinoEigen/ArduinoEigen" \
    "$root/tests/HardwareInterfacesTest.cpp" \
    "$root/src/hardwareIO/pyro/PyroArmManager.cpp" \
    "$root/src/hardwareIO/pyro/PyroChannel.cpp" \
    -o "$build/hardware-interface-tests"
"$build/hardware-interface-tests"