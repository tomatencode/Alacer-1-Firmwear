#include "MessageSceduler.hpp"

#include <algorithm>


MessageScheduler::MessageScheduler(Protocol::Parser& parser, Radio& radio)
        : _parser(parser), _radio(radio), _lastReceived_ms(0), _didRespond(false),
            _dropedMessages(0), _responseContexts{} {
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

    bool hasBusyContexts = std::any_of(
        _responseContexts.begin(), _responseContexts.end(),
        [](const ResponseContext& context) { return context.inUse; });

    if ((!_scheduledMessages.empty() || hasBusyContexts)
        && millis() - _lastReceived_ms >= SEND_TIMEOUT_MS) {
        Protocol::Frame frame;
        
        uint8_t msgIndex = 0;
        while (msgIndex < _scheduledMessages.size() && msgIndex < Protocol::MAX_MESSAGES_PER_FRAME) {
            frame.messages[msgIndex] = _scheduledMessages[msgIndex++];
        }
        for (auto& context : _responseContexts) {
            if (msgIndex >= Protocol::MAX_MESSAGES_PER_FRAME)
                break;
            if (!context.inUse)
                continue;
            
            Protocol::Message message = Protocol::Message{};
            message.type = context.messageType;
            message.seqId = context.sequenceId;
            message.status = Protocol::JobStatus::BUSY;
            frame.messages[msgIndex++] = message;
        }

        frame.numMessages = msgIndex;

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
    auto contextIt = std::find_if(
        _responseContexts.begin(), _responseContexts.end(),
        [](const ResponseContext& context) { return !context.inUse; });
    if (contextIt == _responseContexts.end()) {
        ++_dropedMessages;
        return;
    }

    contextIt->scheduler = this;
    contextIt->messageType = message.type;
    contextIt->sequenceId = message.seqId;
    contextIt->inUse = true;
    job(std::span<const uint8_t>(message.payload.data(), message.payload.size()),
        JobResult::create<ResponseContext, &ResponseContext::respond>(*contextIt));
}

void MessageScheduler::ResponseContext::respond(
    JobResultStatus status,
    std::span<const uint8_t> responsePayload) {
    inUse = false;
    Protocol::Message response;
    response.type = messageType;
    response.seqId = sequenceId;
    response.status = status == JobResultStatus::SUCCESS
        ? Protocol::JobStatus::SUCCESS
        : Protocol::JobStatus::FAILURE;
    response.messageLen = responsePayload.size();
    response.payload.assign(responsePayload.begin(), responsePayload.end());
    scheduler->scheduleMessage(response);
}

void MessageScheduler::scheduleMessage(const Protocol::Message& message) {
    _scheduledMessages.push_back(message);
}


void MessageScheduler::registerForJob(Protocol::MessageType jobType, Job job) {
    _jobs.insert(std::make_pair(jobType, job));
}