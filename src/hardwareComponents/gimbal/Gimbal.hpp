#pragma once

#include "../../hardwareIO/servo/Servo.hpp"

class ControlPID; // forward declaration

namespace hardware {

class Gimbal {
public:

    struct GimbalPos {
        float pitch_deg;
        float yaw_deg;
    };

    Gimbal(
        hardware::Servo& pitchServo, hardware::Servo& yawServo,
        
        GimbalPos initialTarget,

        float minPitch_deg, float maxPitch_deg,
        float minYaw_deg, float maxYaw_deg,

        float pitchServoTranslation, float yawServoTranslation, // unitless scaling factors for servo to gimbal angle conversion
        float pitchServoTrim_deg, float yawServoTrim_deg  // offsets for servo to gimbal angle conversion
    );

    void begin();

    bool setTarget(const GimbalPos& target);
    bool isLocked() const { return _exclusiveControl; }

    GimbalPos getTarget() const;

    GimbalPos getCurrentPos() const; // approximation takes servo speeds into account
    
private:

    friend class ::ControlPID; // only PID can force/lock

    void setExclusiveControl(bool l) { _exclusiveControl = l; }
    void setTargetForced(const GimbalPos& target); // bypasses lock

    bool _exclusiveControl = false;

    hardware::Servo& _pitchServo;
    hardware::Servo& _yawServo;

    GimbalPos _target;

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