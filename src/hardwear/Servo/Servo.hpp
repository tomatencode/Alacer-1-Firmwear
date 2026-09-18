#pragma once

#include <Arduino.h>
#include <Servo.h>

namespace hardware {

class Servo {
public:
    Servo(
        int pin,
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
    int _pin;

    float _targetAngle_deg;

    ::Servo _servo;

    float _minAngle_deg;
    float _maxAngle_deg;

    int _minPulse_us;
    int _maxPulse_us;
    
    float _servoSpeed_deg_s;
    uint32_t _targetSetTime;
    float _targetSetPosition_deg;
};

} // namespace hardware