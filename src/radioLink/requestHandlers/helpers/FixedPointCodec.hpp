#pragma once

#include <cassert>
#include <cstdint>
#include <span>

namespace fixedPoint {

// Encodes `value` as a little-endian int16, scaled by `scale`, into buffer[offset..offset+2).
inline void encode16(float value, std::span<uint8_t> buffer, size_t offset, float scale = 100.0f) {
    assert(offset + 2 <= buffer.size());
    int16_t scaled = static_cast<int16_t>(value * scale);
    buffer[offset + 0] = static_cast<uint8_t>(scaled & 0xFF);
    buffer[offset + 1] = static_cast<uint8_t>((scaled >> 8) & 0xFF);
}

// Decodes a little-endian int16 from buffer[offset..offset+2), divided by `scale`.
inline float decode16(std::span<const uint8_t> buffer, size_t offset, float scale = 100.0f) {
    assert(offset + 2 <= buffer.size());
    auto raw = static_cast<uint16_t>(buffer[offset + 0] | (buffer[offset + 1] << 8));
    return static_cast<float>(static_cast<int16_t>(raw)) / scale;
}

// Encodes `value` as a little-endian int32, scaled by `scale`, into buffer[offset..offset+4).
inline void encode32(float value, std::span<uint8_t> buffer, size_t offset, float scale = 100.0f) {
    assert(offset + 4 <= buffer.size());
    int32_t scaled = static_cast<int32_t>(value * scale);
    buffer[offset + 0] = static_cast<uint8_t>(scaled & 0xFF);
    buffer[offset + 1] = static_cast<uint8_t>((scaled >> 8) & 0xFF);
    buffer[offset + 2] = static_cast<uint8_t>((scaled >> 16) & 0xFF);
    buffer[offset + 3] = static_cast<uint8_t>((scaled >> 24) & 0xFF);
}

// Decodes a little-endian int32 from buffer[offset..offset+4), divided by `scale`.
inline float decode32(std::span<const uint8_t> buffer, size_t offset, float scale = 100.0f) {
    assert(offset + 4 <= buffer.size());
    auto raw = static_cast<uint32_t>(buffer[offset + 0]) |
               (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
               (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
               (static_cast<uint32_t>(buffer[offset + 3]) << 24);
    return static_cast<float>(static_cast<int32_t>(raw)) / scale;
}

} // namespace fixedPoint