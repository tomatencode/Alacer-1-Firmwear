#pragma once

#include <optional>
#include "LogProtocol.hpp"

class LastEventsTracker {
public:

    LastEventsTracker() = default;

    bool checkSameAsLast(const LogProtocol::IMUEvent& event) const {
        return lastIMUEvent.has_value() &&
               lastIMUEvent->accel_x_m_s2 == event.accel_x_m_s2 && lastIMUEvent->accel_y_m_s2 == event.accel_y_m_s2 && lastIMUEvent->accel_z_m_s2 == event.accel_z_m_s2 &&
               lastIMUEvent->gyro_x_rad_s == event.gyro_x_rad_s && lastIMUEvent->gyro_y_rad_s == event.gyro_y_rad_s && lastIMUEvent->gyro_z_rad_s == event.gyro_z_rad_s;
    }

    bool checkSameAsLast(const LogProtocol::BarometerEvent& event) const {
        return lastBarometerEvent.has_value() &&
               lastBarometerEvent->pressure_Pa == event.pressure_Pa && lastBarometerEvent->temperature_C == event.temperature_C;
    }


    void updateLast(const LogProtocol::IMUEvent& event) {
        lastIMUEvent = event;
    }

    void updateLast(const LogProtocol::BarometerEvent& event) {
        lastBarometerEvent = event;
    }

    void clearLasts() {
        lastIMUEvent.reset();
        lastBarometerEvent.reset();
    }
private:
    std::optional<LogProtocol::IMUEvent> lastIMUEvent = std::nullopt;
    std::optional<LogProtocol::BarometerEvent> lastBarometerEvent = std::nullopt;

};