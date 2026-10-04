#pragma once

#include <array>

constexpr float INV_SQRT_2 = 0.70710678f;

constexpr std::array<std::array<float, 3>, 3> IMU_TO_ROCKET_ROTATION_MATRIX = {{
    {{INV_SQRT_2, 0.0f, INV_SQRT_2}},
    {{INV_SQRT_2, 0.0f, -INV_SQRT_2}},
    {{0.0f,       1.0f, 0.0f}}
}};

constexpr float IMU_TO_ROCKET_X = 0.0f;
constexpr float IMU_TO_ROCKET_Y = 0.0f;
constexpr float IMU_TO_ROCKET_Z = 0.0f;