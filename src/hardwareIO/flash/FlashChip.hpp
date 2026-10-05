#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace hardware {

class FlashChip {
public:
    virtual ~FlashChip() = default;

    virtual void begin() = 0;
    virtual bool isConnected() const = 0;

    virtual uint32_t getCapacity() const = 0;
    virtual size_t getPageSize() const = 0;
    virtual size_t getSectorSize() const = 0;

    // Addresses and sizes are in bytes. Operations block until completion or
    // timeout and return false on invalid arguments or hardware failure.
    virtual bool read(uint32_t address, std::span<uint8_t> data) = 0;

    // Programming only changes bits from 1 to 0; erase before reusing storage.
    // Writes may span pages. A failed write may have programmed a prefix.
    virtual bool write(uint32_t address, std::span<const uint8_t> data) = 0;

    // Sector addresses must be aligned to getSectorSize(). Erases are destructive.
    // Long erases block the caller; do not call from a time-critical flight loop.
    virtual bool eraseSector(uint32_t address) = 0;
    virtual bool eraseChip() = 0;
};

} // namespace hardware
