#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../hardwareIO/pyro/PyroManager.hpp"

// SET_PYRO_SOFTWARE_ARMED request:
//   payload[0] = 1 to software-arm, 0 to software-disarm.
// Response:
//   SUCCESS with empty payload if state was set.
//   FAILURE with empty payload if payload missing or payload[0] not 0/1.
class SetPyroSoftwareArmedHandler : public MessageScheduler::RequestHandler {
public:
    explicit SetPyroSoftwareArmedHandler(PyroManager& pyroManager) : _pyroManager(pyroManager) {}

    void handleRequest(std::span<const uint8_t> payload, MessageScheduler::HandlerResult resultCallback) override {
        if (payload.size() < 1) {
            resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
            return;
        }

        if (payload[0] == 1) {
            _pyroManager.SoftwareArm();
        } else if (payload[0] == 0) {
            _pyroManager.SoftwareDisarm();
        } else {
            resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
            return;
        }

        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, {});
    }

private:
    PyroManager& _pyroManager;
};
