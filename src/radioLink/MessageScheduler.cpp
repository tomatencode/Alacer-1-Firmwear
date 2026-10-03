#include "MessageScheduler.hpp"

#include <algorithm>
#include <array>

MessageScheduler::MessageScheduler(Protocol::Parser& parser, hardware::Radio& radio, etl::delegate<bool()> isMidFlightCb)
    : _parser(parser), _radio(radio), _isMidFlightCb(isMidFlightCb),
      _didRespond(false), _droppedMessages(0) {
}

void MessageScheduler::update() {
    while (_radio.available()) {
        auto byte = _radio.read();
        if (!byte.has_value())
            continue;

        _parser.feed(byte.value());

        if (!_parser.hasFrame())
            continue;

        auto frameOpt = _parser.takeFrame();
        if (!frameOpt.has_value())
            continue;

        _didRespond = false;
        auto frame = frameOpt.value();

        for (uint8_t i = 0; i < frame.numMessages; ++i) {
            auto& message = frame.messages[i];
            handleIncomingMessage(message);
        }
    }

    bool hasScheduledMessages = !_scheduledMessages.empty();

    if (hasScheduledMessages && !_didRespond) {
        Protocol::Frame frame;

        const auto scheduledMessageCount = std::min(
            _scheduledMessages.size(),
            static_cast<size_t>(Protocol::MAX_MESSAGES_PER_FRAME));

        uint8_t msgIndex = 0;
        while (msgIndex < _scheduledMessages.size() && msgIndex < Protocol::MAX_MESSAGES_PER_FRAME) {
            frame.messages.push_back(_scheduledMessages[msgIndex++]);
        }

        frame.numMessages = msgIndex;

        std::array<uint8_t, Protocol::MAX_FRAME_SIZE> encodedFrame;
        auto serializedSize = Protocol::encode(frame, encodedFrame);

        if (serializedSize.has_value()) {
            bool success = _radio.send(std::span<const uint8_t>(encodedFrame.data(), serializedSize.value()));

            if (success) {
                _didRespond = true;
                _scheduledMessages.erase(
                    _scheduledMessages.begin(),
                    _scheduledMessages.begin() + scheduledMessageCount);
            }
        }
    }
}

void MessageScheduler::handleIncomingMessage(const Protocol::Message& message) {
    if (message.type == Protocol::MessageType::PING) {
        Protocol::Message response;
        response.type = Protocol::MessageType::PING;
        response.seqId = message.seqId;
        response.status = Protocol::RequestStatus::SUCCESS;
        response.messageLen = 0;
        scheduleMessage(response);
        return;
    }


    auto handlerIt = _handlers.find(message.type);
    if (handlerIt == _handlers.end()) {
        ++_droppedMessages;
        return;
    }

    Protocol::Message response{};
    response.type = message.type;
    response.seqId = message.seqId;

    if (_isMidFlightCb.is_valid()) {
        auto groundOnlyIt = _groundOnlyHandlers.find(message.type);
        const bool groundOnly = (groundOnlyIt == _groundOnlyHandlers.end())
            ? true // fail-closed: unregistered policy blocks mid-flight
            : groundOnlyIt->second;
        if (groundOnly && _isMidFlightCb()) {
            response.status = Protocol::RequestStatus::FAILURE;
            response.messageLen = 0;
            scheduleMessage(response);
            return;
        }
    }

    std::array<uint8_t, 256> responseBuffer{};

    HandlerResult result = handlerIt->second(
        std::span<const uint8_t>(message.payload.data(), message.payload.size()),
        std::span<uint8_t>(responseBuffer.data(), responseBuffer.size()));

    const size_t responseLength = std::min(result.responseLength, responseBuffer.size());
    response.status = result.status == HandlerResultStatus::SUCCESS
        ? Protocol::RequestStatus::SUCCESS
        : Protocol::RequestStatus::FAILURE;
    response.messageLen = static_cast<uint8_t>(responseLength);
    response.payload.assign(responseBuffer.begin(), responseBuffer.begin() + responseLength);

    scheduleMessage(response);
}

void MessageScheduler::scheduleMessage(const Protocol::Message& message) {
    if (_scheduledMessages.size() >= _scheduledMessages.capacity()) {
        ++_droppedMessages;
        return;
    }
    _scheduledMessages.push_back(message);
}

void MessageScheduler::registerRequestHandler(Protocol::MessageType requestType, RequestHandler handler, HandlerOptions options) {
    auto it = _handlers.find(requestType);
    if (it != _handlers.end()) {
        it->second = handler;
    } else {
        _handlers.insert(std::make_pair(requestType, handler));
    }
    auto policyIt = _groundOnlyHandlers.find(requestType);
    if (policyIt != _groundOnlyHandlers.end()) {
        policyIt->second = options.groundOnly;
    } else {
        _groundOnlyHandlers.insert(std::make_pair(requestType, options.groundOnly));
    }
}

void MessageScheduler::registerRequestHandler(Protocol::MessageType requestType, RequestHandler handler) {
    registerRequestHandler(requestType, handler, HandlerOptions{true});
}
