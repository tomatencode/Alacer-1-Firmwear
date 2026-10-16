#pragma once

#include "../Arduino.h"
#include <array>

inline constexpr int LOW = 0;
inline constexpr int HIGH = 1;
inline constexpr int INPUT = 0;
inline constexpr int OUTPUT = 1;

namespace hostArduino {
inline std::array<int, 16> pins{};
inline std::array<int, 16> modes{};
}

inline void pinMode(int pin, int mode) { hostArduino::modes.at(pin) = mode; }
inline int digitalRead(int pin) { return hostArduino::pins.at(pin); }
inline void digitalWrite(int pin, int value) { hostArduino::pins.at(pin) = value; }