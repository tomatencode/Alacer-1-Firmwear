#pragma once
#include <cstdint>
#include <optional>
#include <etl/vector.h>
#include <span>

namespace Protocol {

constexpr uint8_t START_BYTE = 0xAA;
constexpr uint16_t CRC16_INITIAL = 0xFFFF;

const size_t MAX_MESSAGES_PER_FRAME = 16;
const size_t MAX_FRAME_SIZE = 1024;

enum class MessageType : uint8_t {
    PING = 0x00,
    TELEMETRY = 0x01,
    GET_GIMBAL = 0x02,
    SET_GIMBAL = 0x03,
    DO_BEEP = 0x04,
    FIRE_PYRO = 0x05,
    GET_IMU = 0x06,
    GET_BAROMETER = 0x07,
    GET_ROTATION = 0x08,
    SET_ROTATION = 0x09,
    GET_POSITION = 0x0A,
    SET_POSITION = 0x0B,
};

enum class RequestStatus : uint8_t {
    BUSY = 0x00,
    SUCCESS = 0x01,
    FAILURE = 0x02,
};

struct Message {
    MessageType type;
    uint8_t seqId;
    RequestStatus status; // only meaningfull on responses
    uint8_t messageLen;
    etl::vector<uint8_t, 256> payload;
};

struct Frame {
    uint8_t numMessages;
    etl::vector<Message, MAX_MESSAGES_PER_FRAME> messages;
};

uint16_t updateCrc16(uint16_t crc, uint8_t byte);


std::optional<size_t> encode(const Frame& frame, std::span<uint8_t> outBuffer);

class Parser {
public:
    void feed(uint8_t byte);

    bool hasFrame() const { return _pendingFrame.has_value(); };

    std::optional<Frame> takeFrame() {
        auto frame = _pendingFrame;
        _pendingFrame = std::nullopt;
        return frame;
    };

    void reset();

    uint32_t crcErrorCount() const { return _crcErrorCount; };
    uint32_t overflowCount() const { return _lenErrorCount; };
    uint32_t overwrittenFramesCount() const { return _overwrittenFramesCount; };
private:
    enum FrameParseState {
        WAITING_FOR_START_BYTE,
        WAITING_FOR_FRAME_LEN_LOW,
        WAITING_FOR_FRAME_LEN_HIGH,
        WAITING_FOR_NUM_MESSAGES,
        WAITING_FOR_MESSAGE,
        WAITING_FOR_CRC_LOW,
        WAITING_FOR_CRC_HIGH
    };

    enum MessageParseState {
        WAITING_FOR_TYPE,
        WAITING_FOR_SEQ_ID,
        WAITING_FOR_STATUS,
        WAITING_FOR_MESSAGE_LEN,
        WAITING_FOR_PAYLOAD,
    };

    void resetParseState();

    std::optional<Message> feedMessage(uint8_t byte);

    uint16_t _currentCrc = CRC16_INITIAL;

    FrameParseState _frameState = WAITING_FOR_START_BYTE;
    MessageParseState _messageState = WAITING_FOR_TYPE;
    Frame _currentFrame;
    std::optional<Frame> _pendingFrame;
    Message _currentMessage;

    uint8_t _crcLow = 0;

    uint8_t _messagePayloadIndex = 0;

    uint16_t _MessagesExpectedLen = 0;
    uint16_t _MessagesBytesCount = 0;

    uint32_t _crcErrorCount = 0;
    uint32_t _lenErrorCount = 0;
    uint32_t _overwrittenFramesCount = 0;

};

} // namespace Protocol
