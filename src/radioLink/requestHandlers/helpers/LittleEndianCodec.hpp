#pragma once

#include <cassert>
#include <cstdint>
#include <span>

namespace littleEndian {

// Decodes a little-endian uint16 from buffer[offset..offset+2).
inline uint16_t decodeU16(std::span<const uint8_t> buffer, size_t offset) {
    assert(offset + 2 <= buffer.size());
    return static_cast<uint16_t>(buffer[offset + 0]) |
           (static_cast<uint16_t>(buffer[offset + 1]) << 8);
}

// Encodes `value` as little-endian uint16 into buffer[offset..offset+2).
inline void encodeU16(uint16_t value, std::span<uint8_t> buffer, size_t offset) {
    assert(offset + 2 <= buffer.size());
    buffer[offset + 0] = static_cast<uint8_t>(value & 0xFF);
    buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

// Decodes a little-endian uint32 from buffer[offset..offset+4).
inline uint32_t decodeU32(std::span<const uint8_t> buffer, size_t offset) {
    assert(offset + 4 <= buffer.size());
    return static_cast<uint32_t>(buffer[offset + 0]) |
           (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
           (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
           (static_cast<uint32_t>(buffer[offset + 3]) << 24);
}

// Encodes `value` as little-endian uint32 into buffer[offset..offset+4).
inline void encodeU32(uint32_t value, std::span<uint8_t> buffer, size_t offset) {
    assert(offset + 4 <= buffer.size());
    buffer[offset + 0] = static_cast<uint8_t>(value & 0xFF);
    buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buffer[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

// Combines two raw bytes (low first) into a uint16.
// Handy for streaming parsers that hold low/high in separate variables.
inline uint16_t combine(uint8_t low, uint8_t high) {
    return static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
}

// Splits a uint16 into low/high bytes (e.g. for streaming encoders).
inline uint8_t lowByteOf(uint16_t value) {
    return static_cast<uint8_t>(value & 0xFF);
}

// Splits a uint16 into low/high bytes (e.g. for streaming encoders).
inline uint8_t highByteOf(uint16_t value) {
    return static_cast<uint8_t>((value >> 8) & 0xFF);
}

} // namespace littleEndian
