#pragma once

#include <Arduino.h>
#include <HardwareTimer.h>

namespace hardware {

class Servo {
public:
    Servo(
        uint32_t pin,
        uint32_t channel,
        float minAngle_deg,
        float maxAngle_deg,
        float initialAngle_deg,
        int minPulse_us,
        int maxPulse_us,
        float servoSpeed_deg_s = 90.0
    );
    
    void begin();

    void setTarget(float angle_deg);
    float getTarget_deg() const;

    bool isInRange(float angle_deg) const {
        return angle_deg >= _minAngle_deg && angle_deg <= _maxAngle_deg;
    }

    // Get the current position of the servo, taking into account the servo speed and elapsed time.
    float getPosition_deg() const;


    float getMinAngle_deg() const { return _minAngle_deg; }
    float getMaxAngle_deg() const { return _maxAngle_deg; }

    float getSpeed_deg_s() const { return _servoSpeed_deg_s; }

private:
    uint32_t _pin;
    uint32_t _channel;

    float _targetAngle_deg;

    float _minAngle_deg;
    float _maxAngle_deg;

    int _minPulse_us;
    int _maxPulse_us;
    
    float _servoSpeed_deg_s;
    uint32_t _targetSetTime;
    float _targetSetPosition_deg;

    float angleToDuty(float angle_deg) const;
};

} // namespace hardware