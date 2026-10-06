#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../hardwareIO/led/BlinkLed.hpp"
#include "../../../helpers/codec/LittleEndianCodec.hpp"

// FLASH_LED request:
//   empty payload = flash with the LED's default duration.
//   payload[0..1] = optional flash duration in ms, little-endian uint16.
// Response:
//   SUCCESS with empty payload if the flash was started.
//   FAILURE with empty payload if a duration was given but is 0.
class FlashLedHandler {
public:
    explicit FlashLedHandler(hardware::BlinkLed& led) : _led(led) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() >= 2) {
            const uint32_t duration_ms = littleEndian::decodeU16(payload, 0);
            if (duration_ms == 0) {
                return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
            }
            _led.flash(duration_ms);
        } else {
            _led.flash();
        }
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<FlashLedHandler, &FlashLedHandler::handle>(*this);
    }

private:
    hardware::BlinkLed& _led;
};