#pragma once

#include <ArduinoEigen.h>
#include <optional>
#include <cstdint>
#include <numbers>

#include "../rotationEstimation/RotationAccumulator.hpp"
#include "../hardwareComponents/gimbal/Gimbal.hpp"

class ControlPID {
public:
    struct PIDParameters {
        float kp;
        float ki;
        float kd;
    };

    ControlPID(RotationAccumulator& rotationAccumulator, hardware::Gimbal& gimbal);

    void setPIDParameters(float kp, float ki, float kd);
    std::optional<PIDParameters> getPIDParameters() const; // std::nullopt until all three are set

    void setTarget(const Eigen::Quaternionf& targetAngle);
    std::optional<Eigen::Quaternionf> getTarget() const; // std::nullopt until a target is set

    void clearConfig();

    bool canStartControlling() const;

    void startControlling();
    void stopControlling();
    bool setControlling(bool controlling); // true once the requested state is reached
    bool isControlling() const { return _isControlling; }

    void update();
    
private:
    static constexpr uint32_t MAX_DT_us = 10000; 

    RotationAccumulator& _rotationAccumulator;
    hardware::Gimbal& _gimbal;

    std::optional<Eigen::Quaternionf> _targetAngle;

    uint32_t _lastUpdateTime_us = 0;

    bool _isControlling;

    std::optional<float> _kp;
    std::optional<float> _ki;
    std::optional<float> _kd;

    float _integralPitch_rad = 0.0f;
    float _integralYaw_rad = 0.0f;
};