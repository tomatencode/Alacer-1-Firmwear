#pragma once

#include "../Servo/Servo.hpp"

namespace hardware {

class Gimbal {
public:

    struct GimbalPos {
        float pitch_deg;
        float yaw_deg;
    };

    GimbalPos _target;

    Gimbal(
        Servo& pitchServo, Servo& yawServo,
        
        GimbalPos initialTarget,

        float minPitch_deg, float maxPitch_deg,
        float minYaw_deg, float maxYaw_deg,

        float pitchServoTranslation, float yawServoTranslation, // unitless scaling factors for servo to gimbal angle conversion
        float pitchServoTrim_deg, float yawServoTrim_deg  // offsets for servo to gimbal angle conversion
    );

    void begin();

    void setTarget(const GimbalPos& target);
    GimbalPos getTarget() const;

    GimbalPos getCurrentPos() const; // approximation takes servo speeds into account

    void update();
private:
    hardware::Servo& _pitchServo;
    hardware::Servo& _yawServo;

    float _minPitch_deg;
    float _maxPitch_deg;
    float _minYaw_deg;
    float _maxYaw_deg;

    float _pitchServoTrim_deg;
    float _yawServoTrim_deg;

    float _pitchServoTranslation;
    float _yawServoTranslation;
};

} // namespace hardware