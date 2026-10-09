#pragma once

#include <cstdint>

namespace hostArduino {
// Controllable clock for deterministic timeout tests; no wall-clock delays.
inline uint32_t nowMillis = 100;
}

inline uint32_t millis() { return hostArduino::nowMillis; }
inline uint32_t micros() { return 100000; }