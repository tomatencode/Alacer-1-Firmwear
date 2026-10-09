#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include <vector>

#include "radioLink/requestHandlers/logs/StartLogHandler.hpp"
#include "radioLink/requestHandlers/logs/FinishLogHandler.hpp"
#include "radioLink/requestHandlers/logs/IsLoggingHandler.hpp"
#include "radioLink/requestHandlers/logs/ListLogsHandler.hpp"

class MemoryFlash : public hardware::FlashChip {
public:
    std::vector<uint8_t> bytes = std::vector<uint8_t>(1024 * 1024, 0xFF);
    bool failWrites = false;

    void begin() override {}
    bool isConnected() const override { return true; }
    uint32_t getCapacity() const override { return bytes.size(); }
    size_t getPageSize() const override { return 256; }
    size_t getSectorSize() const override { return 4096; }

    bool read(uint32_t address, std::span<uint8_t> data) override {
        if (address > bytes.size() || data.size() > bytes.size() - address) return false;
        std::copy_n(bytes.begin() + address, data.size(), data.begin());
        return true;
    }

    bool write(uint32_t address, std::span<const uint8_t> data) override {
        if (failWrites || address > bytes.size() || data.size() > bytes.size() - address) return false;
        for (size_t i = 0; i < data.size(); ++i) bytes[address + i] &= data[i];
        return true;
    }

    bool eraseSector(uint32_t address) override {
        if (address % 4096 != 0 || address > bytes.size() - 4096) return false;
        std::fill_n(bytes.begin() + address, 4096, 0xFF);
        return true;
    }

    bool eraseChip() override {
        std::fill(bytes.begin(), bytes.end(), 0xFF);
        return true;
    }
};

constexpr auto SUCCESS = MessageScheduler::HandlerResultStatus::SUCCESS;
constexpr auto FAILURE = MessageScheduler::HandlerResultStatus::FAILURE;

void testStringCodec() {
    std::array<uint8_t, 34> buffer{};
    StorageManager::Filename value("flight.bin"), decoded("unchanged");
    const auto size = stringCodec::encode(value, buffer, 1);
    assert(size && *size == 11);
    assert(stringCodec::decode(buffer, 1, decoded) == size && decoded == value);
    assert(!stringCodec::encode(value, std::span(buffer).first(5), 0));
    assert(!stringCodec::encode(value, buffer, buffer.size()));
    assert(!stringCodec::decode(buffer, buffer.size(), decoded));
    assert(!stringCodec::decode(std::span(buffer).first(5), 1, decoded));
    assert(decoded == value);
    buffer[1] = 33;
    assert(!stringCodec::decode(buffer, 1, decoded));
    buffer[1] = 1;
    buffer[2] = 0;
    assert(!stringCodec::decode(buffer, 1, decoded));
    value.assign(32, 'x');
    assert(stringCodec::encode(value, buffer, 0) == 33);
    assert(stringCodec::decode(buffer, 0, decoded) == 33 && decoded == value);
    value.clear();
    assert(stringCodec::encode(value, buffer, 0) == 1);
    assert(stringCodec::decode(buffer, 0, decoded) == 1 && decoded.empty());
}

void testHandlers() {
    MemoryFlash flash;
    StorageManager storage(flash);
    LogManager manager(storage);
    StartLogHandler start(manager);
    FinishLogHandler finish(manager);
    IsLoggingHandler status(manager);
    ListLogsHandler list(storage);
    std::array<uint8_t, 256> response{};
    const std::array<uint8_t, 1> extra{1};
    assert(list.handle({}, response).status == FAILURE);
    assert(storage.begin());
    auto result = list.callback()({}, response);
    assert(result.status == SUCCESS && result.responseLength == 3);
    assert(response[0] == 0 && response[1] == 0 && response[2] == 0);
    assert(status.callback()({}, response).status == SUCCESS && response[0] == 0);
    assert(status.handle({}, {}).status == FAILURE);
    assert(status.handle(extra, response).status == FAILURE);
    assert(finish.handle({}, response).status == FAILURE);

    LogProtocol::LogMetadata metadata{123456789,
        Eigen::Quaternionf(0.5f, 0.1f, 0.2f, 0.3f), Eigen::Quaternionf::Identity(),
        1.25f, -2.5f, 3.75f, 100.5f};
    std::array<uint8_t, LogProtocol::headerSize> header{};
    LogProtocol::encodeHeader(metadata, header);
    std::vector<uint8_t> request(header.begin(), header.begin() + StartLogHandler::METADATA_SIZE);
    StorageManager::Filename filename("flight.bin");
    request.resize(52 + 1 + filename.size());
    assert(stringCodec::encode(filename, request, 52));

    for (size_t size = 0; size < request.size(); ++size) {
        assert(start.handle(std::span(request).first(size), response).status == FAILURE);
        assert(!manager.isLogging());
    }
    auto malformed = request;
    malformed.push_back(0);
    assert(start.handle(malformed, response).status == FAILURE);
    malformed = request;
    malformed[53] = 0;
    assert(start.handle(malformed, response).status == FAILURE);
    malformed.resize(53);
    malformed[52] = 0;
    assert(start.handle(malformed, response).status == FAILURE);
    malformed.resize(86, 'x');
    malformed[52] = 33;
    assert(start.handle(malformed, response).status == FAILURE);

    result = start.callback()(request, response);
    assert(result.status == SUCCESS && result.responseLength == 0);
    assert(storage.getCurrentFile() == filename);
    assert(status.handle({}, response).status == SUCCESS && response[0] == 1);
    assert(start.handle(request, response).status == FAILURE); // already logging
    assert(list.handle({}, response).status == SUCCESS && response[0] == 0); // open file excluded
    assert(finish.handle(extra, response).status == FAILURE && manager.isLogging());
    flash.failWrites = true;
    assert(finish.handle({}, response).status == FAILURE && manager.isLogging());
    flash.failWrites = false;
    result = finish.callback()({}, response);
    assert(result.status == SUCCESS && result.responseLength == 0);
    assert(status.handle({}, response).status == SUCCESS && response[0] == 0);
    assert(finish.handle({}, response).status == FAILURE);
    assert(start.handle(request, response).status == FAILURE); // duplicate filename
    std::array<uint8_t, LogProtocol::headerSize> storedHeader{};
    assert(storage.readFile(filename, 0, storedHeader) == storedHeader.size());
    assert(storedHeader == header); // every metadata field decoded correctly

    // Fill the directory with maximum-length names to exercise pagination.
    for (size_t i = 1; i < StorageManager::MAX_FILES; ++i) {
        StorageManager::Filename name;
        name.assign(32, 'x');
        name[0] = static_cast<char>('A' + i / 26);
        name[1] = static_cast<char>('A' + i % 26);
        assert(storage.startFile(name));
        assert(storage.finishFile());
    }
    size_t next = 0;
    size_t pages = 0;
    do {
        const std::array<uint8_t, 1> page{static_cast<uint8_t>(next)};
        result = list.handle(page, response);
        assert(result.status == SUCCESS && result.responseLength <= 255);
        assert(response[0] == StorageManager::MAX_FILES && response[2] > 0);
        size_t offset = 3;
        for (size_t i = 0; i < response[2]; ++i) {
            StorageManager::Filename decoded;
            const auto consumed = stringCodec::decode(std::span(response).first(result.responseLength), offset, decoded);
            assert(consumed && decoded == storage.listFiles()[next + i]);
            offset += *consumed;
        }
        assert(offset == result.responseLength && response[1] == next + response[2]);
        next = response[1];
        ++pages;
    } while (next < storage.listFiles().size());
    assert(pages > 1);
    const std::array<uint8_t, 1> end{63}, invalid{64};
    result = list.handle(end, response);
    assert(result.status == SUCCESS && result.responseLength == 3 && response[2] == 0);
    assert(list.handle(invalid, response).status == FAILURE);
    assert(list.handle({}, std::span(response).first(2)).status == FAILURE);
    assert(list.handle({}, std::span(response).first(3)).status == FAILURE);
    const std::array<uint8_t, 2> invalidPayload{0, 0};
    assert(list.handle(invalidPayload, response).status == FAILURE);
}

int main() {
    testStringCodec();
    testHandlers();
    std::cout << "Log request handler and string codec tests passed\n";
}