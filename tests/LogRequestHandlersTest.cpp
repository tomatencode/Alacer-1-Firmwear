#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include <vector>

#include "radioLink/requestHandlers/logs/StartLogHandler.hpp"
#include "radioLink/requestHandlers/logs/FinishLogHandler.hpp"
#include "radioLink/requestHandlers/logs/IsLoggingHandler.hpp"
#include "radioLink/requestHandlers/logs/ListLogsHandler.hpp"
#include "radioLink/requestHandlers/logs/GetLogInfoHandler.hpp"
#include "radioLink/requestHandlers/logs/GetLogBytesHandler.hpp"
#include "radioLink/requestHandlers/logs/DeleteLogHandler.hpp"
#include "radioLink/requestHandlers/logs/DeleteAllLogsHandler.hpp"
#include "radioLink/requestHandlers/logs/DownloadManager.hpp"

class MemoryFlash : public hardware::FlashChip {
public:
    std::vector<uint8_t> bytes = std::vector<uint8_t>(1024 * 1024, 0xFF);
    bool failWrites = false;
    bool failReads = false;
    bool failErases = false;
    int readsUntilFailure = -1;
    size_t readBytes = 0;
    size_t readCalls = 0;

    void begin() override {}
    bool isConnected() const override { return true; }
    uint32_t getCapacity() const override { return bytes.size(); }
    size_t getPageSize() const override { return 256; }
    size_t getSectorSize() const override { return 4096; }

    bool read(uint32_t address, std::span<uint8_t> data) override {
        ++readCalls;
        readBytes += data.size();
        if (failReads || readsUntilFailure == 0) return false;
        if (readsUntilFailure > 0) --readsUntilFailure;
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
        if (failErases) return false;
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

std::vector<uint8_t> makeInfoRequest(const StorageManager::Filename& filename) {
    std::vector<uint8_t> request(filename.size() + 1);
    assert(stringCodec::encode(filename, request, 0));
    return request;
}

std::vector<uint8_t> makeBytesRequest(const StorageManager::Filename& filename, uint32_t offset, uint16_t length) {
    auto request = makeInfoRequest(filename);
    const size_t fields = request.size();
    request.resize(fields + 6);
    littleEndian::encodeU32(offset, request, fields);
    littleEndian::encodeU16(length, request, fields + 4);
    return request;
}

void testDownloads() {
    MemoryFlash flash;
    StorageManager storage(flash);
    GetLogInfoHandler info(storage);
    GetLogBytesHandler bytes(storage);
    const StorageManager::Filename filename("download.bin"), unknown("missing.bin"), empty("empty.bin");
    const auto infoRequest = makeInfoRequest(filename);
    auto request = makeBytesRequest(filename, 0, 240);
    std::array<uint8_t, 256> response{};
    assert(info.handle(infoRequest, response).status == FAILURE);
    assert(bytes.handle(request, response).status == FAILURE);
    assert(storage.begin());
    assert(info.handle(makeInfoRequest(unknown), response).status == FAILURE);
    assert(bytes.handle(makeBytesRequest(unknown, 0, 1), response).status == FAILURE);
    assert(storage.startFile(filename));
    assert(info.handle(infoRequest, response).status == FAILURE);
    assert(bytes.handle(request, response).status == FAILURE);
    std::vector<uint8_t> data(2500);
    for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i * 17);
    assert(storage.write(data) == StorageManager::WriteResult::Ok);
    assert(storage.finishFile());
    auto result = info.callback()(infoRequest, response);
    assert(result.status == SUCCESS && result.responseLength == 6);
    assert(littleEndian::decodeU32(response, 0) == data.size());
    assert(littleEndian::decodeU16(response, 4) == 240);
    assert(info.handle(infoRequest, std::span(response).first(5)).status == FAILURE);
    for (size_t n = 0; n < infoRequest.size(); ++n)
        assert(info.handle(std::span(infoRequest).first(n), response).status == FAILURE);
    auto malformed = infoRequest;
    malformed.push_back(0);
    assert(info.handle(malformed, response).status == FAILURE);
    malformed[1] = 0;
    assert(info.handle(malformed, response).status == FAILURE);
    const std::array<uint8_t, 1> emptyName{0};
    assert(info.handle(emptyName, response).status == FAILURE);
    for (size_t n = 0; n < request.size(); ++n)
        assert(bytes.handle(std::span(request).first(n), response).status == FAILURE);
    malformed = request;
    malformed.push_back(0);
    assert(bytes.handle(malformed, response).status == FAILURE);
    malformed = request;
    malformed[1] = 0;
    assert(bytes.handle(malformed, response).status == FAILURE);
    assert(bytes.handle(makeBytesRequest(StorageManager::Filename(), 0, 1), response).status == FAILURE);
    assert(bytes.handle(makeBytesRequest(filename, 0, 0), response).status == FAILURE);
    assert(bytes.handle(makeBytesRequest(filename, 0, 241), response).status == FAILURE);
    assert(bytes.handle(makeBytesRequest(filename, 2501, 1), response).status == FAILURE);
    assert(bytes.handle(makeBytesRequest(filename, UINT32_MAX, 240), response).status == FAILURE);
    assert(bytes.handle(request, std::span(response).first(243)).status == FAILURE);

    std::vector<uint8_t> downloaded;
    for (uint32_t offset = 0; offset < data.size();) {
        request = makeBytesRequest(filename, offset, 240);
        result = bytes.callback()(request, response);
        assert(result.status == SUCCESS && result.responseLength > 4 && result.responseLength <= 244);
        assert(littleEndian::decodeU32(response, 0) == offset);
        const auto first = response;
        const auto retry = bytes.handle(request, response);
        assert(retry.status == SUCCESS && retry.responseLength == result.responseLength && response == first);
        downloaded.insert(downloaded.end(), response.begin() + 4, response.begin() + result.responseLength);
        offset += result.responseLength - 4;
    }
    assert(downloaded == data); // includes a read across an internal flash-frame boundary
    result = bytes.handle(makeBytesRequest(filename, 2499, 240), response);
    assert(result.status == SUCCESS && result.responseLength == 5 && response[4] == data.back());
    result = bytes.handle(makeBytesRequest(filename, 2500, 240), std::span(response).first(4));
    assert(result.status == SUCCESS && result.responseLength == 4 && littleEndian::decodeU32(response, 0) == 2500);
    assert(bytes.handle(makeBytesRequest(filename, 2500, 1), std::span(response).first(3)).status == FAILURE);
    assert(storage.startFile(empty) && storage.finishFile());
    assert(info.handle(makeInfoRequest(empty), response).status == SUCCESS && littleEndian::decodeU32(response, 0) == 0);
    assert(bytes.handle(makeBytesRequest(empty, 0, 1), response).responseLength == 4);

    flash.failReads = true;
    assert(info.handle(infoRequest, response).status == FAILURE);
    assert(bytes.handle(makeBytesRequest(filename, 0, 240), response).status == FAILURE);
    flash.failReads = false;
    // The size scan uses 13 reads for three frames; fail during the subsequent chunk read.
    flash.readsUntilFailure = 13;
    result = bytes.handle(makeBytesRequest(filename, 0, 240), response);
    assert(result.status == FAILURE && result.responseLength == 0);
    flash.readsUntilFailure = -1;
    // Corrupt payload after mounting: never report a truncated size or false EOF.
    flash.bytes[2 * 4096 + 8] ^= 1;
    assert(!storage.fileSizePayloadChecked(filename));
    assert(info.handle(infoRequest, response).status == FAILURE);
    assert(bytes.handle(makeBytesRequest(filename, 0, 240), response).status == FAILURE);
}

class MemoryRadio : public hardware::Radio {
public:
    std::vector<uint8_t> incoming;
    std::vector<std::vector<uint8_t>> sent;
    size_t position = 0;
    bool failSend = false;
    bool send(std::span<const uint8_t> data) override {
        if (failSend) return false;
        sent.emplace_back(data.begin(), data.end());
        return true;
    }
    bool available() override { return position < incoming.size(); }
    std::optional<uint8_t> read() override {
        if (!available()) return std::nullopt;
        return incoming[position++];
    }
};

void testDownloadScheduling() {
    MemoryFlash flash;
    StorageManager storage(flash);
    assert(storage.begin());
    const StorageManager::Filename filename("queued.bin");
    assert(storage.startFile(filename));
    std::array<uint8_t, 240> data{};
    assert(storage.write(data) == StorageManager::WriteResult::Ok && storage.finishFile());
    GetLogBytesHandler bytes(storage);
    Protocol::Parser parser;
    MemoryRadio radio;
    MessageScheduler scheduler(parser, radio);
    scheduler.registerRequestHandler(Protocol::MessageType::GET_LOG_BYTES, bytes.callback());
    Protocol::Frame requests{};
    const auto payload = makeBytesRequest(filename, 0, 240);
    for (uint8_t i = 0; i < 6; ++i) {
        Protocol::Message message{};
        message.type = Protocol::MessageType::GET_LOG_BYTES;
        message.seqId = i;
        message.messageLen = payload.size();
        message.payload.assign(payload.begin(), payload.end());
        requests.messages.push_back(message);
    }
    requests.numMessages = requests.messages.size();
    std::array<uint8_t, Protocol::MAX_FRAME_SIZE> encoded{};
    const auto size = Protocol::encode(requests, encoded);
    assert(size);
    radio.incoming.assign(encoded.begin(), encoded.begin() + *size);
    radio.failSend = true;
    scheduler.update();
    assert(radio.sent.empty());
    radio.failSend = false;
    scheduler.update();
    scheduler.update();
    assert(radio.sent.size() == 2);
    size_t received = 0;
    for (const auto& frameBytes : radio.sent) {
        assert(frameBytes.size() <= Protocol::MAX_FRAME_SIZE);
        Protocol::Parser client;
        for (uint8_t byte : frameBytes) client.feed(byte);
        const auto frame = client.takeFrame();
        assert(frame);
        for (const auto& message : frame->messages) {
            assert(message.seqId == received++ && message.status == Protocol::RequestStatus::SUCCESS);
            assert(message.messageLen == 244 && littleEndian::decodeU32(
                std::span<const uint8_t>(message.payload.data(), message.payload.size()), 0) == 0);
        }
    }
    assert(received == 6 && scheduler.getDroppedMessages() == 0);
    struct FlightState {
        bool isMidFlight() { return true; }
    } flight;
    scheduler.setMidFlightCallback(etl::delegate<bool()>::create<FlightState, &FlightState::isMidFlight>(flight));
    radio.incoming.assign(encoded.begin(), encoded.begin() + *size);
    radio.position = 0;
    scheduler.update();
    Protocol::Parser client;
    for (uint8_t byte : radio.sent.back()) client.feed(byte);
    const auto rejected = client.takeFrame();
    assert(rejected && rejected->numMessages == 6);
    for (const auto& message : rejected->messages)
        assert(message.status == Protocol::RequestStatus::FAILURE && message.messageLen == 0);
}

void testDeletes() {
    MemoryFlash flash;
    StorageManager storage(flash);
    DeleteLogHandler single(storage);
    DeleteAllLogsHandler all(storage);
    const StorageManager::Filename first("first.bin"), second("second.bin"), active("active.bin");
    const auto request = makeInfoRequest(first);
    assert(single.handle(request, {}).status == FAILURE);
    assert(all.handle({}, {}).status == FAILURE);
    assert(storage.begin());
    const std::array<uint8_t, 3> data{10, 20, 30};
    assert(storage.startFile(first));
    assert(storage.write(data) == StorageManager::WriteResult::Ok && storage.finishFile());
    assert(storage.startFile(second));
    assert(storage.write(data) == StorageManager::WriteResult::Ok && storage.finishFile());
    const auto used = storage.usedSpace();
    for (size_t n = 0; n < request.size(); ++n)
        assert(single.handle(std::span(request).first(n), {}).status == FAILURE);
    auto malformed = request;
    malformed.push_back(0);
    assert(single.handle(malformed, {}).status == FAILURE);
    malformed = request;
    malformed[1] = 0;
    assert(single.handle(malformed, {}).status == FAILURE);
    malformed.assign(34, 'x');
    malformed[0] = 33;
    assert(single.handle(malformed, {}).status == FAILURE);
    const std::array<uint8_t, 1> emptyName{0}, extra{1};
    assert(single.handle(emptyName, {}).status == FAILURE);
    assert(single.handle(makeInfoRequest(StorageManager::Filename("missing.bin")), {}).status == FAILURE);
    assert(all.handle(extra, {}).status == FAILURE);
    assert(storage.listFiles().size() == 2 && storage.usedSpace() == used);

    flash.failReads = true;
    assert(single.handle(request, {}).status == FAILURE);
    flash.failReads = false;
    flash.failWrites = true;
    assert(single.handle(request, {}).status == FAILURE);
    assert(all.handle({}, {}).status == FAILURE);
    flash.failWrites = false;
    flash.failErases = true;
    assert(all.handle({}, {}).status == FAILURE);
    flash.failErases = false;
    assert(storage.listFiles().size() == 2 && storage.usedSpace() == used);
    // Failed formatting must leave the previous directory valid after remount.
    StorageManager afterFailure(flash);
    assert(afterFailure.begin() && afterFailure.listFiles().size() == 2);

    assert(storage.startFile(active));
    assert(storage.write(data) == StorageManager::WriteResult::Ok);
    assert(single.handle(makeInfoRequest(active), {}).status == FAILURE);
    assert(all.handle({}, {}).status == FAILURE);
    assert(storage.isFileOpen() && storage.bufferedBytes() == data.size());
    auto result = single.callback()(request, {}); // closed file may be deleted while logging
    assert(result.status == SUCCESS && result.responseLength == 0);
    assert(storage.isFileOpen() && storage.listFiles().size() == 1);
    assert(single.handle(request, {}).status == FAILURE); // already deleted
    assert(!storage.fileSizePayloadChecked(first));
    assert(storage.usedSpace() == used); // single deletion doesn't reclaim arena space
    assert(storage.finishFile());
    StorageManager afterSingle(flash);
    assert(afterSingle.begin() && afterSingle.listFiles().size() == 2);
    assert(!afterSingle.fileSizePayloadChecked(first));
    result = all.callback()({}, {});
    assert(result.status == SUCCESS && result.responseLength == 0);
    assert(storage.listFiles().empty() && !storage.isFileOpen());
    assert(storage.usedSpace() == 0 && storage.freeSpace() == storage.capacity() - 8192);
    assert(!storage.fileSizePayloadChecked(second));
    StorageManager afterAll(flash);
    assert(afterAll.begin() && afterAll.listFiles().empty() && afterAll.usedSpace() == 0);
    assert(all.handle({}, {}).status == SUCCESS); // repeated delete-all on empty directory
    StorageManager afterRepeat(flash);
    assert(afterRepeat.begin() && afterRepeat.listFiles().empty());
    assert(afterRepeat.startFile(first));
    assert(afterRepeat.write(data) == StorageManager::WriteResult::Ok && afterRepeat.finishFile());
    std::array<uint8_t, 3> restored{};
    assert(afterRepeat.readFile(first, 0, restored) == data.size() && restored == data);
}

void testDeleteScheduling() {
    MemoryFlash flash;
    StorageManager storage(flash);
    assert(storage.begin());
    const StorageManager::Filename filename("protected.bin");
    assert(storage.startFile(filename) && storage.finishFile());
    DeleteLogHandler single(storage);
    DeleteAllLogsHandler all(storage);
    Protocol::Parser parser;
    MemoryRadio radio;
    struct FlightState {
        bool midFlight = true;
        bool isMidFlight() { return midFlight; }
    } flight;
    MessageScheduler scheduler(parser, radio,
        etl::delegate<bool()>::create<FlightState, &FlightState::isMidFlight>(flight));
    scheduler.registerRequestHandler(Protocol::MessageType::DELETE_LOG, single.callback());
    scheduler.registerRequestHandler(Protocol::MessageType::DELETE_ALL_LOGS, all.callback());
    Protocol::Frame requests{};
    Protocol::Message message{};
    const auto payload = makeInfoRequest(filename);
    message.type = Protocol::MessageType::DELETE_LOG;
    message.seqId = 10;
    message.messageLen = payload.size();
    message.payload.assign(payload.begin(), payload.end());
    requests.messages.push_back(message);
    message.type = Protocol::MessageType::DELETE_ALL_LOGS;
    message.seqId = 11;
    message.messageLen = 0;
    message.payload.clear();
    requests.messages.push_back(message);
    requests.numMessages = 2;
    std::array<uint8_t, Protocol::MAX_FRAME_SIZE> encoded{};
    const auto size = Protocol::encode(requests, encoded);
    assert(size);
    for (bool midFlight : {true, false}) {
        flight.midFlight = midFlight;
        radio.incoming.assign(encoded.begin(), encoded.begin() + *size);
        radio.position = 0;
        scheduler.update();
        Protocol::Parser client;
        for (uint8_t byte : radio.sent.back()) client.feed(byte);
        const auto response = client.takeFrame();
        assert(response && response->numMessages == 2);
        for (size_t i = 0; i < 2; ++i) {
            const auto& reply = response->messages[i];
            assert(reply.seqId == 10 + i && reply.messageLen == 0);
            assert(reply.status == (midFlight ? Protocol::RequestStatus::FAILURE : Protocol::RequestStatus::SUCCESS));
        }
        assert(storage.listFiles().size() == (midFlight ? 1 : 0));
    }
}

std::array<uint8_t, 8> makeChunkRequest(uint32_t session, uint32_t index) {
    std::array<uint8_t, 8> request{};
    littleEndian::encodeU32(session, request, 0);
    littleEndian::encodeU32(index, request, 4);
    return request;
}

std::vector<uint8_t> makeStartDownloadRequest(const StorageManager::Filename& filename, uint32_t token) {
    auto request = makeInfoRequest(filename);
    const size_t fields = request.size();
    request.resize(fields + 4);
    littleEndian::encodeU32(token, request, fields);
    return request;
}

void testSequentialReader() {
    MemoryFlash flash;
    StorageManager storage(flash);
    const StorageManager::Filename filename("reader.bin");
    assert(!storage.openSequentialReader(filename));
    assert(storage.begin() && storage.startFile(filename));
    assert(!storage.openSequentialReader(filename));
    std::vector<uint8_t> data(2500);
    for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i * 13);
    assert(storage.write(data) == StorageManager::WriteResult::Ok && storage.finishFile());
    auto reader = storage.openSequentialReader(filename);
    assert(reader && reader->valid() && reader->size() == data.size() && reader->position() == 0);
    const size_t before = flash.readBytes;
    std::vector<uint8_t> restored(data.size());
    for (size_t offset = 0; offset < data.size(); offset += 137) {
        const size_t length = std::min(size_t{137}, data.size() - offset);
        assert(reader->read(std::span(restored).subspan(offset, length)) == length);
    }
    assert(restored == data && reader->position() == data.size());
    assert(flash.readBytes - before == storage.fileSizeFramed(filename));
    std::array<uint8_t, 100> scratch{};
    assert(reader->read(scratch) == 0);
    assert(!reader->advanceTo(0) && !reader->advanceTo(2501));
    assert(reader->copyCached(2400, scratch));
    assert(std::equal(scratch.begin(), scratch.end(), data.begin() + 2400));
    assert(!reader->copyCached(0, scratch));
    reader = storage.openSequentialReader(filename);
    assert(reader && reader->advanceTo(1100) && reader->read(scratch) == scratch.size());
    assert(std::equal(scratch.begin(), scratch.end(), data.begin() + 1100));
    // Cached readers cannot survive deletion/remount or filename reuse.
    assert(storage.deleteAllFiles() && !reader->valid());
    assert(reader->read(scratch) == 0 && !reader->copyCached(1100, scratch));
    assert(storage.startFile(filename));
    assert(storage.write(data) == StorageManager::WriteResult::Ok && storage.finishFile());
    reader = storage.openSequentialReader(filename);
    assert(reader && storage.begin() && !reader->valid());
    reader = storage.openSequentialReader(filename);
    assert(reader);
    // Corruption after preparation is caught when the frame is actually loaded.
    flash.bytes[8192 + 8] ^= 1;
    assert(reader->read(scratch) == 0 && !reader->valid());
    assert(!storage.openSequentialReader(filename));
    // Explicit flushes produce variable-length frames, not fixed 1024B blocks.
    assert(storage.deleteAllFiles() && storage.startFile(filename));
    const std::array<size_t, 5> lengths{17, 513, 1, 1024, 945};
    size_t offset = 0;
    for (size_t length : lengths) {
        assert(storage.write(std::span(data).subspan(offset, length)) == StorageManager::WriteResult::Ok);
        assert(storage.flush());
        offset += length;
    }
    assert(offset == data.size() && storage.finishFile());
    reader = storage.openSequentialReader(filename);
    assert(reader && reader->read(restored) == restored.size() && restored == data);
}

void testDownloadManager() {
    MemoryFlash flash;
    StorageManager storage(flash);
    DownloadManager download(storage);
    const StorageManager::Filename filename("session.bin"), empty("empty.bin");
    const auto start = makeStartDownloadRequest(filename, 123);
    std::array<uint8_t, 256> response{};
    assert(download.start(start, response).status == FAILURE);
    assert(storage.begin() && storage.startFile(filename));
    assert(download.start(start, response).status == FAILURE);
    std::vector<uint8_t> data(5500);
    for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i * 17 + i / 256);
    assert(storage.write(data) == StorageManager::WriteResult::Ok && storage.finishFile());
    for (size_t n = 0; n < start.size(); ++n)
        assert(download.start(std::span(start).first(n), response).status == FAILURE);
    auto malformed = start;
    malformed.push_back(0);
    assert(download.start(malformed, response).status == FAILURE);
    assert(download.start(start, std::span(response).first(13)).status == FAILURE);
    auto result = download.startCallback()(start, response);
    assert(result.status == SUCCESS && result.responseLength == 14);
    uint32_t session = littleEndian::decodeU32(response, 0);
    assert(session != 0 && littleEndian::decodeU32(response, 4) == data.size());
    assert(littleEndian::decodeU16(response, 8) == 240);
    const uint32_t chunks = littleEndian::decodeU32(response, 10);
    assert(chunks == 23);
    size_t before = flash.readBytes;
    assert(download.start(start, response).status == SUCCESS);
    assert(littleEndian::decodeU32(response, 0) == session && flash.readBytes == before);
    assert(download.start(makeStartDownloadRequest(filename, 124), response).status == FAILURE);
    assert(download.getChunk(makeChunkRequest(session + 1, 0), response).status == FAILURE);
    assert(download.getChunk(makeChunkRequest(session, chunks), response).status == FAILURE);
    assert(download.getChunk(makeChunkRequest(session, 0), std::span(response).first(247)).status == FAILURE);
    const auto first = makeChunkRequest(session, 0);
    for (size_t n = 0; n < first.size(); ++n)
        assert(download.getChunk(std::span(first).first(n), response).status == FAILURE);
    auto check = [&](uint32_t index) {
        const auto reply = download.chunkCallback()(makeChunkRequest(session, index), response);
        const size_t offset = index * 240;
        const size_t count = std::min(size_t{240}, data.size() - offset);
        assert(reply.status == SUCCESS && reply.responseLength == count + 8);
        assert(littleEndian::decodeU32(response, 0) == session);
        assert(littleEndian::decodeU32(response, 4) == index);
        assert(std::equal(response.begin() + 8, response.begin() + reply.responseLength,
                          data.begin() + offset));
    };
    before = flash.readBytes;
    for (uint32_t i = 0; i < chunks; ++i) {
        check(i);
        const size_t retryBefore = flash.readBytes;
        check(i);
        assert(flash.readBytes == retryBefore);
    }
    // One read of each frame, not a full-size scan per chunk.
    assert(flash.readBytes - before == storage.fileSizeFramed(filename));
    before = flash.readBytes;
    check(22); // final chunk remains retryable
    check(0); // old frame: inefficient readFile fallback
    assert(flash.readBytes > before);
    before = flash.readBytes;
    check(0); // last-chunk cache
    check(22); // current frame cache, despite the backward request
    assert(flash.readBytes == before);
    auto stop = makeChunkRequest(session, 0);
    assert(download.stop(std::span(stop).first(3), {}).status == FAILURE);
    assert(download.stop(makeChunkRequest(session + 1, 0), {}).status == FAILURE);
    assert(download.stopCallback()(std::span(stop).first(4), {}).status == SUCCESS);
    assert(download.stop(std::span(stop).first(4), {}).status == SUCCESS);
    assert(download.getChunk(first, response).status == FAILURE);
    assert(download.start(start, response).status == SUCCESS);
    const uint32_t newerSession = littleEndian::decodeU32(response, 0);
    assert(newerSession != session);
    assert(download.stop(std::span(stop).first(4), {}).status == FAILURE);
    session = newerSession;
    check(10); // forward seek verifies skipped frames
    check(0); // fallback must not rewind the sequential reader
    before = flash.readBytes;
    check(11);
    assert(flash.readBytes == before); // same frame as chunk 10
    hostArduino::nowMillis += DownloadManager::TIMEOUT_MS;
    download.update();
    assert(download.getChunk(makeChunkRequest(session, 11), response).status == FAILURE);
    assert(download.start(start, response).status == SUCCESS);
    session = littleEndian::decodeU32(response, 0);
    check(0);
    assert(storage.deleteFile(filename));
    assert(download.getChunk(makeChunkRequest(session, 0), response).status == FAILURE);
    assert(storage.startFile(empty) && storage.finishFile());
    assert(download.start(makeStartDownloadRequest(empty, 200), response).status == SUCCESS);
    session = littleEndian::decodeU32(response, 0);
    assert(littleEndian::decodeU32(response, 4) == 0 && littleEndian::decodeU32(response, 10) == 0);
    assert(download.getChunk(makeChunkRequest(session, 0), response).status == FAILURE);
    assert(storage.deleteAllFiles());
    assert(storage.startFile(filename));
    assert(storage.write(data) == StorageManager::WriteResult::Ok && storage.finishFile());
    assert(download.start(start, response).status == SUCCESS);
    session = littleEndian::decodeU32(response, 0);
    // Fail after loading the first frame, during a chunk spanning two frames.
    check(3);
    flash.failReads = true;
    assert(download.getChunk(makeChunkRequest(session, 4), response).status == FAILURE);
    flash.failReads = false;
    assert(download.getChunk(makeChunkRequest(session, 4), response).status == FAILURE);
    assert(download.start(start, response).status == SUCCESS);
    session = littleEndian::decodeU32(response, 0);
    check(10);
    flash.failReads = true;
    assert(download.getChunk(makeChunkRequest(session, 0), response).status == FAILURE);
    flash.failReads = false;
    assert(download.getChunk(makeChunkRequest(session, 10), response).status == FAILURE);
    assert(download.start(start, response).status == SUCCESS);
    session = littleEndian::decodeU32(response, 0);
    flash.bytes[8192 + 1032 + 8] ^= 1; // second frame, after initial verification
    assert(download.getChunk(makeChunkRequest(session, 4), response).status == FAILURE);
    assert(download.start(start, response).status == FAILURE);
}

void testSessionDownloadScheduling() {
    MemoryFlash flash;
    StorageManager storage(flash);
    assert(storage.begin());
    const StorageManager::Filename filename("wire.bin");
    assert(storage.startFile(filename));
    std::array<uint8_t, 240> data{};
    data.fill(73);
    assert(storage.write(data) == StorageManager::WriteResult::Ok && storage.finishFile());
    DownloadManager download(storage);
    MemoryRadio radio;
    Protocol::Parser parser;
    struct FlightState {
        bool midFlight = false;
        bool isMidFlight() { return midFlight; }
    } flight;
    MessageScheduler scheduler(parser, radio,
        etl::delegate<bool()>::create<FlightState, &FlightState::isMidFlight>(flight));
    scheduler.registerRequestHandler(Protocol::MessageType::START_LOG_DOWNLOAD, download.startCallback());
    scheduler.registerRequestHandler(Protocol::MessageType::GET_LOG_CHUNK, download.chunkCallback());
    scheduler.registerRequestHandler(Protocol::MessageType::STOP_LOG_DOWNLOAD, download.stopCallback());
    auto exchange = [&](Protocol::MessageType type, std::span<const uint8_t> payload) {
        Protocol::Frame frame{};
        Protocol::Message message{};
        message.type = type;
        message.seqId = 42;
        message.messageLen = static_cast<uint8_t>(payload.size());
        message.payload.assign(payload.begin(), payload.end());
        frame.messages.push_back(message);
        frame.numMessages = 1;
        std::array<uint8_t, Protocol::MAX_FRAME_SIZE> encoded{};
        const auto length = Protocol::encode(frame, encoded);
        assert(length);
        radio.incoming.assign(encoded.begin(), encoded.begin() + *length);
        radio.position = 0;
        scheduler.update();
        Protocol::Parser client;
        for (uint8_t byte : radio.sent.back()) client.feed(byte);
        auto reply = client.takeFrame();
        assert(reply && reply->numMessages == 1);
        assert(reply->messages[0].seqId == 42 && reply->messages[0].type == type);
        return reply->messages[0];
    };
    const auto start = makeStartDownloadRequest(filename, 1);
    flight.midFlight = true;
    auto reply = exchange(Protocol::MessageType::START_LOG_DOWNLOAD, start);
    assert(reply.status == Protocol::RequestStatus::FAILURE && reply.messageLen == 0);
    flight.midFlight = false;
    reply = exchange(Protocol::MessageType::START_LOG_DOWNLOAD, start);
    assert(reply.status == Protocol::RequestStatus::SUCCESS && reply.messageLen == 14);
    const uint32_t session = littleEndian::decodeU32(
        std::span<const uint8_t>(reply.payload.data(), reply.payload.size()), 0);
    const auto chunk = makeChunkRequest(session, 0);
    reply = exchange(Protocol::MessageType::GET_LOG_CHUNK, chunk);
    assert(reply.status == Protocol::RequestStatus::SUCCESS && reply.messageLen == 248);
    assert(std::equal(reply.payload.begin() + 8, reply.payload.end(), data.begin()));
    flight.midFlight = true;
    for (auto type : {Protocol::MessageType::GET_LOG_CHUNK, Protocol::MessageType::STOP_LOG_DOWNLOAD}) {
        reply = exchange(type, type == Protocol::MessageType::GET_LOG_CHUNK
            ? std::span(chunk) : std::span(chunk).first(4));
        assert(reply.status == Protocol::RequestStatus::FAILURE && reply.messageLen == 0);
    }
    flight.midFlight = false;
    reply = exchange(Protocol::MessageType::STOP_LOG_DOWNLOAD, std::span(chunk).first(4));
    assert(reply.status == Protocol::RequestStatus::SUCCESS && reply.messageLen == 0);
    reply = exchange(Protocol::MessageType::GET_LOG_CHUNK, chunk);
    assert(reply.status == Protocol::RequestStatus::FAILURE && reply.messageLen == 0);
}

int main() {
    testStringCodec();
    testHandlers();
    testDownloads();
    testDownloadScheduling();
    testDeletes();
    testDeleteScheduling();
    testSequentialReader();
    testDownloadManager();
    testSessionDownloadScheduling();
    std::cout << "Log request handler and string codec tests passed\n";
}