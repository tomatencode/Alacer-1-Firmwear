#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../hardwareIO/pyro/PyroManager.hpp"

// GET_PYRO_SOFTWARE_ARMED request: empty payload.
// Response: SUCCESS with payload[0] = 1 if software-armed, 0 otherwise.
class GetPyroSoftwareArmedHandler {
public:
    explicit GetPyroSoftwareArmedHandler(PyroManager& pyroManager) : _pyroManager(pyroManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        response[0] = _pyroManager.isSoftwareArmed() ? 1 : 0;
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 1};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetPyroSoftwareArmedHandler, &GetPyroSoftwareArmedHandler::handle>(*this);
    }

private:
    PyroManager& _pyroManager;
};
