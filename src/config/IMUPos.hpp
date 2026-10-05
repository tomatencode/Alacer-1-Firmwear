#pragma once

#include <array>

// Rocket body frame: +Z points toward the nose. Roll, pitch, and yaw are
// rotations about +Z, +Y, and +X respectively.
constexpr float INV_SQRT_2 = 0.70710678f;
// Base mounting is 180 deg about Y vs. chip, plus 45 deg clockwise roll because the PCB is rotated compared to the gimbal.
// (clockwise when looking forward along +Z from the tail, i.e. +45 deg
// right-hand about +Z: Rz(45)*R_base).
constexpr std::array<std::array<float, 3>, 3> IMU_TO_ROCKET_ROTATION_MATRIX = {{
    {{INV_SQRT_2, 0.0f, -INV_SQRT_2}},
    {{INV_SQRT_2, 0.0f,  INV_SQRT_2}},
    {{0.0f, -1.0f, 0.0f}}
}};

constexpr float IMU_TO_ROCKET_X = 0.0f;
constexpr float IMU_TO_ROCKET_Y = 0.0f;
constexpr float IMU_TO_ROCKET_Z = 0.0f;