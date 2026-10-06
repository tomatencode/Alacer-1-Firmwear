#pragma once

#include <cassert>
#include <cstdint>
#include <span>

#include "LittleEndianCodec.hpp"

namespace fixedPoint {

constexpr float kScale = 100.0f;

// Encodes `value` as a little-endian int16, scaled by `kScale`, into buffer[offset..offset+2).
inline void encode16(float value, std::span<uint8_t> buffer, size_t offset) {
    assert(offset + 2 <= buffer.size());
    int16_t scaled = static_cast<int16_t>(value * kScale);
    littleEndian::encodeU16(static_cast<uint16_t>(scaled), buffer, offset);
}

// Decodes a little-endian int16 from buffer[offset..offset+2), divided by `kScale`.
inline float decode16(std::span<const uint8_t> buffer, size_t offset) {
    assert(offset + 2 <= buffer.size());
    auto raw = littleEndian::decodeU16(buffer, offset);
    return static_cast<float>(static_cast<int16_t>(raw)) / kScale;
}

// Encodes `value` as a little-endian int32, scaled by `kScale`, into buffer[offset..offset+4).
inline void encode32(float value, std::span<uint8_t> buffer, size_t offset) {
    assert(offset + 4 <= buffer.size());
    int32_t scaled = static_cast<int32_t>(value * kScale);
    littleEndian::encodeU32(static_cast<uint32_t>(scaled), buffer, offset);
}

// Decodes a little-endian int32 from buffer[offset..offset+4), divided by `kScale`.
inline float decode32(std::span<const uint8_t> buffer, size_t offset) {
    assert(offset + 4 <= buffer.size());
    auto raw = littleEndian::decodeU32(buffer, offset);
    return static_cast<float>(static_cast<int32_t>(raw)) / kScale;
}

} // namespace fixedPoint