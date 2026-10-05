#include "W25Q32JV.hpp"

#include <algorithm>

namespace {
    const SPISettings kSpiSettings(8000000, MSBFIRST, SPI_MODE0);
}

namespace hardware {

W25Q32JV::W25Q32JV(int csPin, SPIClass &spi)
    : _csPin(csPin), _spi(spi) {
}

void W25Q32JV::begin() {
    _connected = false;
    // Set the output latch high before enabling the CS output.
    digitalWrite(_csPin, HIGH);
    pinMode(_csPin, OUTPUT);
    _spi.begin();
    delay(5); // Power-up write-inhibit interval tPUW.
    sendCommand(CMD_RELEASE_POWER_DOWN);
    delayMicroseconds(5); // tRES1 max is 3 us.

    // Do not reset an already-running program/erase operation: that can corrupt data.
    if (!waitUntilReady(CHIP_TIMEOUT_MS)) {
        return;
    }

    select();
    _spi.transfer(CMD_JEDEC_ID);
    uint32_t id = static_cast<uint32_t>(_spi.transfer(0x00)) << 16;
    id |= static_cast<uint32_t>(_spi.transfer(0x00)) << 8;
    id |= _spi.transfer(0x00);
    deselect();
    _connected = (id == JEDEC_ID);
    if (_connected) {
        sendCommand(CMD_WRITE_DISABLE);
    }
}

bool W25Q32JV::isConnected() const {
    return _connected;
}

uint32_t W25Q32JV::getCapacity() const {
    return CAPACITY;
}

size_t W25Q32JV::getPageSize() const {
    return PAGE_SIZE;
}

size_t W25Q32JV::getSectorSize() const {
    return SECTOR_SIZE;
}

bool W25Q32JV::validRange(uint32_t address, size_t length) const {
    return address <= CAPACITY && length <= CAPACITY - address;
}

bool W25Q32JV::read(uint32_t address, std::span<uint8_t> data) {
    if (!_connected || !validRange(address, data.size()) ||
        (!data.empty() && data.data() == nullptr)) {
        return false;
    }
    if (data.empty()) {
        return true;
    }
    if (!waitUntilReady(CHIP_TIMEOUT_MS)) {
        return false;
    }

    select();
    _spi.transfer(CMD_READ);
    sendAddress(address);
    for (uint8_t &value : data) {
        value = _spi.transfer(0x00);
    }
    deselect();
    return true;
}

bool W25Q32JV::write(uint32_t address, std::span<const uint8_t> data) {
    if (!_connected || !validRange(address, data.size()) ||
        (!data.empty() && data.data() == nullptr)) {
        return false;
    }

    while (!data.empty()) {
        const size_t length = std::min<size_t>(data.size(), PAGE_SIZE - address % PAGE_SIZE);
        if (!enableWrite()) {
            return false;
        }
        select();
        _spi.transfer(CMD_PAGE_PROGRAM);
        sendAddress(address);
        for (size_t i = 0; i < length; ++i) {
            _spi.transfer(data[i]);
        }
        deselect();
        if (!waitUntilReady(PROGRAM_TIMEOUT_MS)) {
            return false;
        }
        sendCommand(CMD_WRITE_DISABLE);
        if (!verify(address, data.data(), length)) {
            return false;
        }
        address += static_cast<uint32_t>(length);
        data = data.subspan(length);
    }
    return true;
}

bool W25Q32JV::eraseSector(uint32_t address) {
    return erase(CMD_SECTOR_ERASE, address, SECTOR_SIZE, SECTOR_TIMEOUT_MS);
}

bool W25Q32JV::eraseBlock32K(uint32_t address) {
    return erase(CMD_BLOCK_ERASE_32K, address, 32U * 1024U, BLOCK32_TIMEOUT_MS);
}

bool W25Q32JV::eraseBlock64K(uint32_t address) {
    return erase(CMD_BLOCK_ERASE_64K, address, 64U * 1024U, BLOCK64_TIMEOUT_MS);
}

bool W25Q32JV::eraseChip() {
    return erase(CMD_CHIP_ERASE, 0, CAPACITY, CHIP_TIMEOUT_MS);
}

bool W25Q32JV::erase(uint8_t command, uint32_t address, uint32_t length, uint32_t timeoutMs) {
    if (!_connected || address % length != 0 || !validRange(address, length) || !enableWrite()) {
        return false;
    }
    select();
    _spi.transfer(command);
    if (command != CMD_CHIP_ERASE) {
        sendAddress(address);
    }
    deselect();
    if (!waitUntilReady(timeoutMs)) {
        return false;
    }
    sendCommand(CMD_WRITE_DISABLE);
    return verify(address, nullptr, length);
}

bool W25Q32JV::verify(uint32_t address, const uint8_t *expected, size_t length) {
    // Stream read-back without a large RAM buffer. nullptr means erased (0xFF).
    select();
    _spi.transfer(CMD_READ);
    sendAddress(address);
    bool matches = true;
    for (size_t i = 0; i < length; ++i) {
        if (_spi.transfer(0x00) != (expected ? expected[i] : 0xFF)) {
            matches = false;
            break;
        }
    }
    deselect();
    return matches;
}

void W25Q32JV::select() {
    _spi.beginTransaction(kSpiSettings);
    digitalWrite(_csPin, LOW);
}

void W25Q32JV::deselect() {
    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();
}

void W25Q32JV::sendCommand(uint8_t command) {
    select();
    _spi.transfer(command);
    deselect();
}

void W25Q32JV::sendAddress(uint32_t address) {
    _spi.transfer(static_cast<uint8_t>(address >> 16));
    _spi.transfer(static_cast<uint8_t>(address >> 8));
    _spi.transfer(static_cast<uint8_t>(address));
}

uint8_t W25Q32JV::readStatus() {
    select();
    _spi.transfer(CMD_READ_STATUS);
    const uint8_t status = _spi.transfer(0x00);
    deselect();
    return status;
}

bool W25Q32JV::waitUntilReady(uint32_t timeoutMs) {
    const uint32_t start = millis();
    while ((readStatus() & STATUS_BUSY) != 0) {
        if (static_cast<uint32_t>(millis() - start) >= timeoutMs) {
            return false;
        }
        delay(1);
    }
    return true;
}

bool W25Q32JV::enableWrite() {
    if (!waitUntilReady(CHIP_TIMEOUT_MS)) {
        return false;
    }
    sendCommand(CMD_WRITE_ENABLE);
    const uint8_t status = readStatus();
    if ((status & (STATUS_BUSY | STATUS_WRITE_ENABLED)) != STATUS_WRITE_ENABLED) {
        sendCommand(CMD_WRITE_DISABLE);
        return false;
    }
    return true;
}

} // namespace hardware
