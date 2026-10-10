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

    bool checkSameAsLast(const LogProtocol::BatteryEvent& event) const {
        return lastBatteryEvent.has_value() &&
               lastBatteryEvent->voltage_v == event.voltage_v;
    }

    bool checkSameAsLast(const LogProtocol::GimbalEvent& event) const {
        return lastGimbalEvent.has_value() &&
               lastGimbalEvent->pitch_deg == event.pitch_deg && lastGimbalEvent->yaw_deg == event.yaw_deg;
    }

    bool checkSameAsLast(const LogProtocol::RotationEvent& event) const {
        return lastRotationEvent.has_value() &&
               lastRotationEvent->rotation.coeffs() == event.rotation.coeffs();
    }

    bool checkSameAsLast(const LogProtocol::HorizontalMovementEvent& event) const {
        return lastHorizontalMovementEvent.has_value() &&
               lastHorizontalMovementEvent->x_m == event.x_m && lastHorizontalMovementEvent->y_m == event.y_m &&
               lastHorizontalMovementEvent->velocity_x_m_s == event.velocity_x_m_s && lastHorizontalMovementEvent->velocity_y_m_s == event.velocity_y_m_s &&
               lastHorizontalMovementEvent->ascentStage == event.ascentStage;
    }

    bool checkSameAsLast(const LogProtocol::VerticalMovementEvent& event) const {
        return lastVerticalMovementEvent.has_value() &&
               lastVerticalMovementEvent->height_m == event.height_m && lastVerticalMovementEvent->velocity_m_s == event.velocity_m_s;
    }


    void updateLast(const LogProtocol::IMUEvent& event) {
        lastIMUEvent = event;
    }

    void updateLast(const LogProtocol::BarometerEvent& event) {
        lastBarometerEvent = event;
    }

    void updateLast(const LogProtocol::BatteryEvent& event) {
        lastBatteryEvent = event;
    }

    void updateLast(const LogProtocol::GimbalEvent& event) {
        lastGimbalEvent = event;
    }

    void updateLast(const LogProtocol::RotationEvent& event) {
        lastRotationEvent = event;
    }

    void updateLast(const LogProtocol::HorizontalMovementEvent& event) {
        lastHorizontalMovementEvent = event;
    }

    void updateLast(const LogProtocol::VerticalMovementEvent& event) {
        lastVerticalMovementEvent = event;
    }

    void clearLasts() {
        lastIMUEvent.reset();
        lastBarometerEvent.reset();
        lastBatteryEvent.reset();
        lastGimbalEvent.reset();
        lastRotationEvent.reset();
        lastHorizontalMovementEvent.reset();
        lastVerticalMovementEvent.reset();
    }
private:
    std::optional<LogProtocol::IMUEvent> lastIMUEvent = std::nullopt;
    std::optional<LogProtocol::BarometerEvent> lastBarometerEvent = std::nullopt;
    std::optional<LogProtocol::BatteryEvent> lastBatteryEvent = std::nullopt;
    std::optional<LogProtocol::GimbalEvent> lastGimbalEvent = std::nullopt;
    std::optional<LogProtocol::RotationEvent> lastRotationEvent = std::nullopt;
    std::optional<LogProtocol::HorizontalMovementEvent> lastHorizontalMovementEvent = std::nullopt;
    std::optional<LogProtocol::VerticalMovementEvent> lastVerticalMovementEvent = std::nullopt;

};
