#pragma once
#include <cstdint>
#include <vector>
constexpr int MSBFIRST = 0;
constexpr int SPI_MODE0 = 0;
class SPISettings {
public:
    SPISettings(int, int, int) {}
};
class SPIClass {
public:
    std::vector<uint8_t> replies;
    size_t cursor = 0;
    void begin() {}
    void beginTransaction(const SPISettings&) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t) {
        return cursor < replies.size() ? replies[cursor++] : 0xFF;
    }
};