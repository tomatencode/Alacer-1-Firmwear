#pragma once
#include <cstdint>
#include <optional>
#include <etl/vector.h>

namespace Protocol {

constexpr uint8_t START_BYTE = 0xAA;

enum class MessageType : uint8_t {
    GET_TELEMETRY = 0x01,
    TELEMETRY = 0x02,
    SET_GIMBAL = 0x11,
    SET_GIMBAL_ACK = 0x12,
    DO_BEEP = 0x21,
    DO_BEEP_ACK = 0x22
};

struct Message {
    MessageType type;
    uint8_t seqId;
    uint8_t messageLen;
    etl::vector<uint8_t, 256> payload;
};

struct Frame {
    uint16_t messagesTotalLen;
    uint8_t numMessages;
    etl::vector<Message, 16> messages;
};

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
        WAITING_FOR_MESSAGE_LEN,
        WAITING_FOR_PAYLOAD,
    };

    void resetParseState();

    std::optional<Message> feedMessage(uint8_t byte);

    uint16_t _currentCrc = 0;
    void updateCrc16(uint8_t byte);

    FrameParseState _frameState = WAITING_FOR_START_BYTE;
    MessageParseState _messageState = WAITING_FOR_TYPE;
    Frame _currentFrame;
    std::optional<Frame> _pendingFrame;
    Message _currentMessage;

    uint8_t _crcLow = 0;

    uint8_t _messagePayloadIndex = 0;
    uint16_t _MessagesBytesCount = 0;

    uint32_t _crcErrorCount = 0;
    uint32_t _lenErrorCount = 0;
    uint32_t _overwrittenFramesCount = 0;

};

} // namespace Protocol
