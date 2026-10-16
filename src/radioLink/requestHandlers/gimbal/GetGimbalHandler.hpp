#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../hardwareComponents/gimbal/IGimbal.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"

namespace radioLink {

class GetGimbalHandler {
public:
    explicit GetGimbalHandler(hardware::IGimbal& gimbal) : _gimbal(gimbal) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        hardware::IGimbal::GimbalPos pos = _gimbal.getCurrentPos();
        fixedPoint::encode16(pos.pitch_deg, response, 0);
        fixedPoint::encode16(pos.yaw_deg, response, 2);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 4};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetGimbalHandler, &GetGimbalHandler::handle>(*this);
    }

private:
    hardware::IGimbal& _gimbal;
};

} // namespace radioLink
