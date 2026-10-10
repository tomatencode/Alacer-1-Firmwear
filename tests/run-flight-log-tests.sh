#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
deps="$root/.pio/libdeps/custom_f411ce"
build="$(mktemp -d)"
trap 'rm -rf "$build"' EXIT
g++ -std=c++20 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$root/tests/host" -I"$root/src" \
    -isystem "$deps/Embedded Template Library/include" \
    -isystem "$deps/ArduinoEigen/ArduinoEigen" \
    "$root/tests/FlightLogEventsTest.cpp" \
    "$root/src/logManagement/StorageManager.cpp" \
    "$root/src/logManagement/LogManager.cpp" \
    "$root/src/logManagement/LogProtocol.cpp" \
    "$root/src/logManagement/TelemetryRecorder.cpp" \
    "$root/src/hardwareIO/pyro/PyroArmManager.cpp" \
    "$root/src/hardwareIO/pyro/PyroChannel.cpp" \
    "$root/src/radioLink/Protocol.cpp" \
    "$root/src/radioLink/MessageScheduler.cpp" \
    "$root/src/hardwareIO/imu/ICM45686.cpp" \
    "$root/src/hardwareIO/barometer/MS5611.cpp" \
    "$root/src/barometricHeightCalculation/BarometricHeightCalculator.cpp" \
    "$root/src/ascentTracking/VerticalMovementTracker.cpp" \
    "$root/src/ascentTracking/HorizontalMovementTracker.cpp" \
    "$root/src/rotationEstimation/RotationAccumulator.cpp" \
    "$root/src/helpers/coordinates/IMURocketCoordinateConverter.cpp" \
    "$root/src/hardwareComponents/gimbal/Gimbal.cpp" \
    "$root/src/controlPID/ControlPID.cpp" \
    "$root/src/flightStateManagement/FlightSequenceExecutor.cpp" \
    "$root/src/flightStateManagement/FlightStateManager.cpp" \
    -o "$build/flight-log-tests"
"$build/flight-log-tests"