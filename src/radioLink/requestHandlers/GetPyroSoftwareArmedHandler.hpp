#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../hardwareIO/pyro/PyroManager.hpp"

// GET_PYRO_SOFTWARE_ARMED request: empty payload.
// Response: SUCCESS with payload[0] = 1 if software-armed, 0 otherwise.
class GetPyroSoftwareArmedHandler : public MessageScheduler::RequestHandler {
public:
    explicit GetPyroSoftwareArmedHandler(PyroManager& pyroManager) : _pyroManager(pyroManager) {}

    void handleRequest(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) override {
        const uint8_t armed = _pyroManager.isSoftwareArmed() ? 1 : 0;
        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS,
                       std::span<const uint8_t>(&armed, 1));
    }

private:
    PyroManager& _pyroManager;
};
