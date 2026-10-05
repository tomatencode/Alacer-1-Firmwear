#pragma once

#include <cstdint>
#include <optional>

#include "etl/vector.h"

#include "hardwareIO/flash/FlashChip.hpp"

class StorageManager {
public:
    StorageManager(hardware::FlashChip &flashChip);

    void begin(); // indexes files, closes open files

    bool startFile();
    std::optional<uint16_t> finishFile(); // returns the ID of the finished file if successful, std::nullopt otherwise

    const etl::ivector<uint16_t>& listFiles() const;

    bool deleteFile(uint16_t fileId);
    void deleteAllFiles();

    size_t readFile(std::uint16_t fileId, std::uint32_t offset, std::span<std::uint8_t> output);

    void update();

    enum class Result : uint8_t { Ok, BufferFull, NoOpenFile, WriteError };
    Result write(std::span<const uint8_t> data);

private:
    std::array<uint8_t, 1024> _byteBuffer;

    hardware::FlashChip &_flashChip;
};