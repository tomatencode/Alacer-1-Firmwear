#pragma once

#include <array>

// Rocket body frame: +Z points toward the nose. Roll, pitch, and yaw are
// rotations about +Z, +Y, and +X respectively.
constexpr float INV_SQRT_2 = 0.70710678f;

constexpr std::array<std::array<float, 3>, 3> IMU_TO_ROCKET_ROTATION_MATRIX = {{
    {{-1.0f, 0.0f,  0.0f}},
    {{0.0f, 0.0f,  1.0f}},
    {{0.0f, 1.0f, 0.0f}}
}};

constexpr float IMU_TO_ROCKET_X = 0.0f;
constexpr float IMU_TO_ROCKET_Y = 0.0f;
constexpr float IMU_TO_ROCKET_Z = 0.0f;