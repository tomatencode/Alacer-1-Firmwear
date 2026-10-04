#pragma once

#include <cstddef>
#include <span>

#include <ArduinoEigen.h>

#include "FixedPointCodec.hpp"

// Shared codec for Eigen quaternions on the radio link.
// Components are serialised in (x, y, z, w) order, each one as a little-endian
// int32 fixed-point value scaled by `fixedPoint::kScale` (100), so a quaternion
// occupies 16 bytes.
namespace quaternionCodec {

constexpr size_t ENCODED_SIZE = 4 * sizeof(int32_t); // 16 bytes

// Encodes `q` (x, y, z, w) into buffer[offset..offset+16).
inline void encode(const Eigen::Quaternionf& q, std::span<uint8_t> buffer, size_t offset) {
    fixedPoint::encode32(q.x(), buffer, offset + 0);
    fixedPoint::encode32(q.y(), buffer, offset + 4);
    fixedPoint::encode32(q.z(), buffer, offset + 8);
    fixedPoint::encode32(q.w(), buffer, offset + 12);
}

// Decodes a quaternion (x, y, z, w) from buffer[offset..offset+16).
inline Eigen::Quaternionf decode(std::span<const uint8_t> buffer, size_t offset) {
    Eigen::Quaternionf q;
    q.x() = fixedPoint::decode32(buffer, offset + 0);
    q.y() = fixedPoint::decode32(buffer, offset + 4);
    q.z() = fixedPoint::decode32(buffer, offset + 8);
    q.w() = fixedPoint::decode32(buffer, offset + 12);
    return q;
}

} // namespace quaternionCodec
