#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "FlashChip.hpp"

namespace hardware {

class W25Q32JV : public FlashChip {
public:
    explicit W25Q32JV(int csPin, SPIClass &spi = SPI);

    // Uses standard SPI mode 0 at 8 MHz; /WP and /HOLD must be held high.
    // begin() wakes the chip and checks its ID, but does not change protection.
    void begin() override;
    bool isConnected() const override;

    uint32_t getCapacity() const override;
    size_t getPageSize() const override;
    size_t getSectorSize() const override;

    bool read(uint32_t address, std::span<uint8_t> data) override;
    bool write(uint32_t address, std::span<const uint8_t> data) override;
    bool eraseSector(uint32_t address) override;
    bool eraseChip() override;
    // Block addresses must be aligned to 32 KiB and 64 KiB respectively.
    bool eraseBlock32K(uint32_t address);
    bool eraseBlock64K(uint32_t address);

private:
    static constexpr uint32_t CAPACITY = 4U * 1024U * 1024U;
    static constexpr size_t PAGE_SIZE = 256;
    static constexpr size_t SECTOR_SIZE = 4096;
    static constexpr uint32_t JEDEC_ID = 0xEF4016;

    static constexpr uint8_t CMD_READ = 0x03;
    static constexpr uint8_t CMD_PAGE_PROGRAM = 0x02;
    static constexpr uint8_t CMD_WRITE_ENABLE = 0x06;
    static constexpr uint8_t CMD_WRITE_DISABLE = 0x04;
    static constexpr uint8_t CMD_READ_STATUS = 0x05;
    static constexpr uint8_t CMD_SECTOR_ERASE = 0x20;
    static constexpr uint8_t CMD_BLOCK_ERASE_32K = 0x52;
    static constexpr uint8_t CMD_BLOCK_ERASE_64K = 0xD8;
    static constexpr uint8_t CMD_CHIP_ERASE = 0xC7;
    static constexpr uint8_t CMD_JEDEC_ID = 0x9F;
    static constexpr uint8_t CMD_RELEASE_POWER_DOWN = 0xAB;
    static constexpr uint8_t STATUS_BUSY = 0x01;
    static constexpr uint8_t STATUS_WRITE_ENABLED = 0x02;

    // Datasheet maximum times plus margin, in milliseconds.
    static constexpr uint32_t PROGRAM_TIMEOUT_MS = 10;
    static constexpr uint32_t SECTOR_TIMEOUT_MS = 500;
    static constexpr uint32_t BLOCK32_TIMEOUT_MS = 2000;
    static constexpr uint32_t BLOCK64_TIMEOUT_MS = 2500;
    static constexpr uint32_t CHIP_TIMEOUT_MS = 60000;

    bool validRange(uint32_t address, size_t length) const;
    void select();
    void deselect();
    void sendCommand(uint8_t command);
    void sendAddress(uint32_t address);
    uint8_t readStatus();
    bool waitUntilReady(uint32_t timeoutMs);
    bool enableWrite();
    bool erase(uint8_t command, uint32_t address, uint32_t length, uint32_t timeoutMs);
    bool verify(uint32_t address, const uint8_t *expected, size_t length);

    int _csPin;
    SPIClass &_spi;
    bool _connected = false;
};

} // namespace hardware
