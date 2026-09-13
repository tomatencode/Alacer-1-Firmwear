#include "Protocol.hpp"

void Protocol::Parser::feed(uint8_t byte) {
    switch (_frameState)
    {
    case WAITING_FOR_START_BYTE: {
        if (byte == START_BYTE) {
            resetParseState();
            _frameState = WAITING_FOR_FRAME_LEN_LOW;
        }
        break;
    }
    case WAITING_FOR_FRAME_LEN_LOW: {
        updateCrc16(byte);
        _currentFrame.messagesTotalLen = byte;
        _frameState = WAITING_FOR_FRAME_LEN_HIGH;
        break;
    }
    case WAITING_FOR_FRAME_LEN_HIGH: {
        updateCrc16(byte);
        _currentFrame.messagesTotalLen |= static_cast<uint16_t>(byte) << 8;
        _frameState = WAITING_FOR_NUM_MESSAGES;
        break;
    }
    case WAITING_FOR_NUM_MESSAGES: {
        updateCrc16(byte);
        _currentFrame.numMessages = byte;

        if (_currentFrame.numMessages > 16) {
            _lenErrorCount.fetch_add(1, std::memory_order_relaxed);
            _frameState = WAITING_FOR_START_BYTE;
            break;
        }


        if (_currentFrame.numMessages > 0) {
            _frameState = WAITING_FOR_MESSAGE;
        } else {
            _frameState = WAITING_FOR_CRC_LOW;
        }
        break;
    }
    case WAITING_FOR_MESSAGE: {
        updateCrc16(byte);
        _MessagesBytesCount++;
        auto msg = feedMessage(byte);
        if (msg.has_value()) {
            _currentFrame.messages.push_back(msg.value());
        }
        if (_currentFrame.messages.size() == _currentFrame.numMessages) {
            if (_MessagesBytesCount == _currentFrame.messagesTotalLen) {
                _frameState = WAITING_FOR_CRC_LOW;
            } else {
                _lenErrorCount.fetch_add(1, std::memory_order_relaxed);
                _frameState = WAITING_FOR_START_BYTE;
            }
        } else if (_MessagesBytesCount > _currentFrame.messagesTotalLen) {
            _lenErrorCount.fetch_add(1, std::memory_order_relaxed);
            _frameState = WAITING_FOR_START_BYTE;
        }
        break;
    }
    case WAITING_FOR_CRC_LOW: {
        _crcLow = byte;
        _frameState = WAITING_FOR_CRC_HIGH;
        break;
    }
    case WAITING_FOR_CRC_HIGH: {
        uint8_t crcHigh = byte;
        uint16_t receivedCrc = static_cast<uint16_t>(crcHigh) << 8 | _crcLow;

        _frameState = WAITING_FOR_START_BYTE;

        if (receivedCrc != _currentCrc) {
            _crcErrorCount.fetch_add(1, std::memory_order_relaxed);
        }
        if (receivedCrc == _currentCrc) {
            if (_pendingFrame.has_value()) {
                _overwrittenFramesCount.fetch_add(1, std::memory_order_relaxed);
            }
            _pendingFrame = _currentFrame;
        }
    }
    }
}

std::optional<Protocol::Message> Protocol::Parser::feedMessage(uint8_t byte) {
    switch (_messageState)
    {
    case WAITING_FOR_TYPE: {
        _currentMessage = Message{};
        _messagePayloadIndex = 0;

        _currentMessage.type = static_cast<Protocol::MessageType>(byte);
        _messageState = WAITING_FOR_SEQ_ID;
        break;
    }
    case WAITING_FOR_SEQ_ID: {
        _currentMessage.seqId = byte;
        _messageState = WAITING_FOR_MESSAGE_LEN;
        break;
    }
    case WAITING_FOR_MESSAGE_LEN: {
        _currentMessage.messageLen = byte;
        if (_currentMessage.messageLen > 0) {
            _messagePayloadIndex = 0;
            _messageState = WAITING_FOR_PAYLOAD;
        } else {
            _messageState = WAITING_FOR_TYPE;
            return _currentMessage;
        }
        break;
    }
    case WAITING_FOR_PAYLOAD: {
        _currentMessage.payload.push_back(byte);
        _messagePayloadIndex++;
        if (_messagePayloadIndex >= _currentMessage.messageLen) {
            _messageState = WAITING_FOR_TYPE;
            return _currentMessage;
        }
        break;
    }
    }
    return std::nullopt;
}

std::optional<Protocol::Frame> Protocol::Parser::takeFrame() {
    __disable_irq();
    auto frame = _pendingFrame;
    _pendingFrame = std::nullopt;
    __enable_irq();
    return frame;
}

void Protocol::Parser::reset() {
    __disable_irq();
    _frameState = WAITING_FOR_START_BYTE;
    _messageState = WAITING_FOR_TYPE;
    _currentFrame = Frame{};
    _currentMessage = Message{};
    _currentCrc = 0;
    _crcLow = 0;
    _messagePayloadIndex = 0;
    _MessagesBytesCount = 0;
    _pendingFrame = std::nullopt;
    __enable_irq();
}

void Protocol::Parser::resetParseState() {
    _frameState = WAITING_FOR_START_BYTE;
    _messageState = WAITING_FOR_TYPE;
    _currentFrame = Frame{};
    _currentMessage = Message{};
    _currentCrc = 0;
    _crcLow = 0;
    _messagePayloadIndex = 0;
    _MessagesBytesCount = 0;
}

void Protocol::Parser::updateCrc16(uint8_t byte) {
    _currentCrc ^= static_cast<uint16_t>(byte) << 8;
    for (int i = 0; i < 8; i++) {
        if (_currentCrc & 0x8000) {
            _currentCrc = (_currentCrc << 1) ^ 0x1021;
        } else {
            _currentCrc <<= 1;
        }
    }
}