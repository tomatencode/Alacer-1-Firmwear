#include "MessageSceduler.hpp"


MessageScheduler::MessageScheduler(Protocol::Parser& parser, Radio& radio)
    : _parser(parser), _radio(radio), _lastReceived_ms(0), _didRespond(false), _dropedMessages(0) {
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

        _lastReceived_ms = millis();
        auto frame = frameOpt.value();

        for (uint8_t i = 0; i < frame.numMessages; ++i) {
            auto& message = frame.messages[i];

            handleIncomingMessage(message);
        }
    }

    // Send all scheduled messages
    if (!_scheduledMessages.empty()
        && millis() - _lastReceived_ms >= SEND_TIMEOUT_MS) {
        Protocol::Frame frame;
        frame.numMessages = _scheduledMessages.size();
        for (uint8_t i = 0; i < frame.numMessages; ++i) {
            frame.messages[i] = _scheduledMessages[i];
        }
        std::array<uint8_t, Protocol::MAX_FRAME_SIZE> encodedFrame;
        auto serializedSize = Protocol::encode(frame, encodedFrame);

        if (serializedSize.has_value()) {
            _radio.send(std::span<const uint8_t>(encodedFrame.data(), serializedSize.value()));
            _scheduledMessages.clear();
        }
    }
}


void MessageScheduler::handleIncomingMessage(const Protocol::Message& message) {
    auto jobIt = _jobs.find(message.type);
    if (jobIt == _jobs.end()) {
        ++_dropedMessages;
        return;
    }

    auto& job = jobIt->second;
    job(message.payload, [this, message](Protocol::MessageType responseType, etl::vector<uint8_t, 256> responsePayload) {
        Protocol::Message response;
        response.type = responseType;
        response.seqId = message.seqId;
        response.messageLen = responsePayload.size();
        response.payload = responsePayload;
        scheduleMessage(response);
    });
}

void MessageScheduler::scheduleMessage(const Protocol::Message& message) {
    _scheduledMessages.push_back(message);
}


void MessageScheduler::registerForJob(Protocol::MessageType jobType, std::function<void(etl::vector<uint8_t, 256> params, std::function<void(Protocol::MessageType, etl::vector<uint8_t, 256>)> resultCallback)> job) {
    if (!Protocol::isDispatchable(jobType))
        return;
    _jobs.insert(std::make_pair(jobType, job));
}