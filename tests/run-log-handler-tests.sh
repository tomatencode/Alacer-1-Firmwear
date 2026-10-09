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
    "$root/tests/LogRequestHandlersTest.cpp" \
    "$root/src/logManagement/StorageManager.cpp" \
    "$root/src/logManagement/LogManager.cpp" \
    "$root/src/logManagement/LogProtocol.cpp" \
    "$root/src/radioLink/Protocol.cpp" \
    "$root/src/radioLink/MessageScheduler.cpp" \
    -o "$build/log-handler-tests"
"$build/log-handler-tests"