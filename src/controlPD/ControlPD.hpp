#pragma once
#include "timing/Clock.hpp"

#include <ArduinoEigen.h>
#include <optional>
#include <cstdint>
#include <numbers>

#include "../rotationEstimation/RotationAccumulator.hpp"
#include "../hardwareComponents/gimbal/Gimbal.hpp"

class ControlPD {
public:
    struct PDParameters {
        float kp;
        float kd;
    };

    ControlPD(const Clock& clock, RotationAccumulator& rotationAccumulator, hardware::Gimbal& gimbal);

    void setPDParameters(float kp, float kd);
    std::optional<PDParameters> getPDParameters() const; // std::nullopt until both are set

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
    const Clock& _clock;
    static constexpr uint32_t MAX_DT_us = 10000; 

    RotationAccumulator& _rotationAccumulator;
    hardware::Gimbal& _gimbal;

    std::optional<Eigen::Quaternionf> _targetAngle;

    uint32_t _lastUpdateTime_us = 0;

    bool _isControlling;

    std::optional<float> _kp;
    std::optional<float> _kd;
};