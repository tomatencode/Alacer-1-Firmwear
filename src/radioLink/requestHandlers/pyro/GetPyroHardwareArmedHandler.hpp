#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../hardwareIO/pyro/PyroArmManager.hpp"

namespace radioLink {

// GET_PYRO_HARDWARE_ARMED request: empty payload.
// Response: SUCCESS with payload[0] = 1 if hardware arm pin reads armed, 0 otherwise.
class GetPyroHardwareArmedHandler {
public:
    explicit GetPyroHardwareArmedHandler(hardware::PyroArmManager& pyroArmManager) : _pyroArmManager(pyroArmManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        response[0] = _pyroArmManager.isHardwareArmed() ? 1 : 0;
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 1};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetPyroHardwareArmedHandler, &GetPyroHardwareArmedHandler::handle>(*this);
    }

private:
    hardware::PyroArmManager& _pyroArmManager;
};

} // namespace radioLink
