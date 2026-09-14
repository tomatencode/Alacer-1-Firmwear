#pragma once

#include <functional>
#include "etl/vector.h"
#include "etl/map.h"

#include "Protocol.hpp"
#include "../hardwear/radio/Radio.hpp"


constexpr uint32_t SEND_TIMEOUT_MS = 50;

class MessageScheduler {
public:

    MessageScheduler(Protocol::Parser& parser, Radio& radio);

    void update();
    
    void registerForJob(Protocol::MessageType jobType, std::function<void(etl::vector<uint8_t, 256> params, std::function<void(Protocol::MessageType, etl::vector<uint8_t, 256>)> resultCallback)> job);
    
    uint32_t getDroppedMessages() const { // for diagnostics
        return _dropedMessages;
    }
    
private:
    Protocol::Parser& _parser;
    Radio& _radio;

    void handleIncomingMessage(const Protocol::Message& message);
    void scheduleMessage(const Protocol::Message& message);

    uint32_t _lastReceived_ms;
    bool _didRespond;

    uint32_t _dropedMessages;

    etl::map<Protocol::MessageType, std::function<void(etl::vector<uint8_t, 256> params, std::function<void(Protocol::MessageType, etl::vector<uint8_t, 256>)> resultCallback)>, 16> _jobs;
    etl::vector<Protocol::Message, 16> _scheduledMessages;
};