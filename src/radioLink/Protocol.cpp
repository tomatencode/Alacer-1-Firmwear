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
        _currentCrc = updateCrc16(_currentCrc, byte);
        _MessagesExpectedLen = byte;
        _frameState = WAITING_FOR_FRAME_LEN_HIGH;
        break;
    }
    case WAITING_FOR_FRAME_LEN_HIGH: {
        _currentCrc = updateCrc16(_currentCrc, byte);
        _MessagesExpectedLen |= static_cast<uint16_t>(byte) << 8;
        _frameState = WAITING_FOR_NUM_MESSAGES;
        break;
    }
    case WAITING_FOR_NUM_MESSAGES: {
        _currentCrc = updateCrc16(_currentCrc, byte);
        _currentFrame.numMessages = byte;

        if (_currentFrame.numMessages > 16) {
            _lenErrorCount++;
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
        _currentCrc = updateCrc16(_currentCrc, byte);
        _MessagesBytesCount++;
        auto msg = feedMessage(byte);
        if (msg.has_value()) {
            _currentFrame.messages.push_back(msg.value());
        }
        if (_currentFrame.messages.size() == _currentFrame.numMessages) {
            if (_MessagesBytesCount == _MessagesExpectedLen) {
                _frameState = WAITING_FOR_CRC_LOW;
            } else {
                _lenErrorCount++;
                _frameState = WAITING_FOR_START_BYTE;
            }
        } else if (_MessagesBytesCount > _MessagesExpectedLen) {
            _lenErrorCount++;
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
            _crcErrorCount++;
        }
        if (receivedCrc == _currentCrc) {
            if (_pendingFrame.has_value()) {
                _overwrittenFramesCount++;
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
        _messageState = WAITING_FOR_STATUS;
        break;
    }
    case WAITING_FOR_STATUS: {
        _currentMessage.status = static_cast<Protocol::JobStatus>(byte);
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

void Protocol::Parser::reset() {
    _frameState = WAITING_FOR_START_BYTE;
    _messageState = WAITING_FOR_TYPE;
    _currentFrame = Frame{};
    _currentMessage = Message{};
    _currentCrc = CRC16_INITIAL;
    _crcLow = 0;
    _messagePayloadIndex = 0;
    _MessagesBytesCount = 0;
    _pendingFrame = std::nullopt;
}

void Protocol::Parser::resetParseState() {
    _frameState = WAITING_FOR_START_BYTE;
    _messageState = WAITING_FOR_TYPE;
    _currentFrame = Frame{};
    _currentMessage = Message{};
    _currentCrc = CRC16_INITIAL;
    _crcLow = 0;
    _messagePayloadIndex = 0;
    _MessagesBytesCount = 0;
}

uint16_t Protocol::updateCrc16(uint16_t crc, uint8_t byte) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

std::optional<size_t> Protocol::encode(const Frame& frame, std::span<uint8_t> outBuffer) {
    if (frame.numMessages > MAX_MESSAGES_PER_FRAME || frame.messages.size() < frame.numMessages) {
        return std::nullopt;
    }

    // START (1) + LEN_LOW (1) + LEN_HIGH (1) + NUM_MSGS (1) + CRC_LOW (1) + CRC_HIGH (1) = 6 bytes
    constexpr size_t HEADER_LEN = 4;
    constexpr size_t CRC_LEN = 2;
    constexpr size_t OVERHEAD = HEADER_LEN + CRC_LEN;

    if (outBuffer.size() < OVERHEAD) {
        return std::nullopt;
    }

    outBuffer[0] = Protocol::START_BYTE;
    outBuffer[3] = frame.numMessages;

    uint16_t messagesIndex = 0;
    const uint16_t messagesStart = 4;

    for (size_t i = 0; i < frame.numMessages; i++) {
        const auto& message = frame.messages[i];
        
        // 3 bytes for message header (type, seqId, messageLen) + payload
        size_t requiredMsgLen = 3 + message.messageLen;
        if (OVERHEAD + messagesIndex + requiredMsgLen > outBuffer.size() || 
            message.payload.size() < message.messageLen) {
            return std::nullopt;
        }

        outBuffer[messagesStart + messagesIndex++] = static_cast<uint8_t>(message.type);
        outBuffer[messagesStart + messagesIndex++] = message.seqId;
        outBuffer[messagesStart + messagesIndex++] = static_cast<uint8_t>(message.status);
        outBuffer[messagesStart + messagesIndex++] = message.messageLen;

        for (size_t j = 0; j < message.messageLen; j++) {
            outBuffer[messagesStart + messagesIndex++] = message.payload[j];
        }
    }

    // Write messages total length bytes (little-endian)
    outBuffer[1] = static_cast<uint8_t>(messagesIndex & 0xFF);
    outBuffer[2] = static_cast<uint8_t>((messagesIndex >> 8) & 0xFF);

    uint16_t crc = Protocol::CRC16_INITIAL;
    for (size_t i = 1; i < messagesStart + messagesIndex; i++) {
        crc = Protocol::updateCrc16(crc, outBuffer[i]);
    }
    outBuffer[messagesStart + messagesIndex] = static_cast<uint8_t>(crc & 0xFF);
    outBuffer[messagesStart + messagesIndex + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return OVERHEAD + messagesIndex;
}