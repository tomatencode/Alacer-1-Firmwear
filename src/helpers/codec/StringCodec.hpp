#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include <etl/string.h>

namespace stringCodec {

// Wire format: uint8 byte length followed by that many bytes, without a NUL.
// Returns the number of consumed/written bytes; failure leaves output unchanged.
inline std::optional<size_t> encode(const etl::istring& value, std::span<uint8_t> buffer, size_t offset) {
    if (value.size() > UINT8_MAX || offset >= buffer.size() ||
        value.size() > buffer.size() - offset - 1) {
        return std::nullopt;
    }
    buffer[offset] = static_cast<uint8_t>(value.size());
    for (size_t i = 0; i < value.size(); ++i) {
        buffer[offset + 1 + i] = static_cast<uint8_t>(value[i]);
    }
    return value.size() + 1;
}

inline std::optional<size_t> decode(std::span<const uint8_t> buffer, size_t offset, etl::istring& value) {
    if (offset >= buffer.size()) {
        return std::nullopt;
    }
    const size_t length = buffer[offset];
    if (length > value.capacity() || length > buffer.size() - offset - 1) {
        return std::nullopt;
    }
    // Embedded NULs cannot round-trip through the storage filename API.
    for (size_t i = 0; i < length; ++i) {
        if (buffer[offset + 1 + i] == 0) {
            return std::nullopt;
        }
    }
    value.assign(reinterpret_cast<const char*>(buffer.data() + offset + 1), length);
    return length + 1;
}

} // namespace stringCodec