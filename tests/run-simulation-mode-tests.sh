#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
deps="$root/.pio/libdeps/custom_f411ce"
build="$(mktemp -d)"
trap 'rm -rf "$build"' EXIT

compiler="$(command -v g++)"
flags=(
    -std=c++20 -Wall -Wextra -Werror
    -DALACER_SIMULATION
    -fsanitize=address,undefined -fno-omit-frame-pointer
    -I"$root/tests/host" -I"$root/src"
    -isystem "$deps/ArduinoEigen/ArduinoEigen"
)

# Give IntelliSense the same host headers, compiler and defines as this test.
# Only index the test entry point here; firmware sources keep their MCU settings.
python3 - "$root" "$compiler" "${flags[@]}" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
source = root / "tests/SimulationModeTest.cpp"
database = root / ".pio/host-tests/compile_commands.json"
database.parent.mkdir(parents=True, exist_ok=True)
database.write_text(json.dumps([{
    "directory": str(root),
    "file": str(source),
    "arguments": [sys.argv[2], *sys.argv[3:], "-fsyntax-only", str(source)],
}], indent=2) + "\n")
PY

"$compiler" "${flags[@]}" \
    "$root/tests/SimulationModeTest.cpp" \
    "$root/src/simMode/SimRocket.cpp" \
    "$root/src/simMode/physics/RigidBody.cpp" \
    "$root/src/helpers/coordinates/IMURocketCoordinateConverter.cpp" \
    -o "$build/simulation-mode-tests"
"$build/simulation-mode-tests"