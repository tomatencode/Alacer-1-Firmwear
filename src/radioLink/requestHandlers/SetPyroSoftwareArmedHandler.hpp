#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../hardwareIO/pyro/PyroManager.hpp"

// SET_PYRO_SOFTWARE_ARMED request:
//   payload[0] = 1 to software-arm, 0 to software-disarm.
// Response:
//   SUCCESS with empty payload if state was set.
//   FAILURE with empty payload if payload missing or payload[0] not 0/1.
class SetPyroSoftwareArmedHandler {
public:
    explicit SetPyroSoftwareArmedHandler(PyroManager& pyroManager) : _pyroManager(pyroManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < 1) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        if (payload[0] == 1) {
            _pyroManager.SoftwareArm();
        } else if (payload[0] == 0) {
            _pyroManager.SoftwareDisarm();
        } else {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<SetPyroSoftwareArmedHandler, &SetPyroSoftwareArmedHandler::handle>(*this);
    }

private:
    PyroManager& _pyroManager;
};
