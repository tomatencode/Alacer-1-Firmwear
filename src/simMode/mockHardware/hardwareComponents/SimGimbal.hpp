#pragma once
#include <algorithm>

#include "simMode/SimRocket.hpp"
#include "hardwareComponents/gimbal/IGimbal.hpp"

namespace simHardware {

class SimGimbal : public hardware::IGimbal {
public:

    using GimbalPos = hardware::IGimbal::GimbalPos;

    SimGimbal(SimRocket& simRocket, float limit_deg)
        : _simRocket(simRocket), _limit_deg(limit_deg) {}

    void begin() override { setTarget({0.0f, 0.0f}); }

    void setTarget(const GimbalPos& target) override {
        const float pitchNoise = static_cast<float>(rand()) / RAND_MAX * 0.05f;
        const float yawNoise = static_cast<float>(rand()) / RAND_MAX * 0.05f;
        _simRocket.setGimbalPos({
            std::clamp(target.pitch_deg + pitchNoise, -_limit_deg, _limit_deg),
            std::clamp(target.yaw_deg + yawNoise, -_limit_deg, _limit_deg)});
    }

    GimbalPos getTarget() const override { return _simRocket.getGimbalPos(); }

    GimbalPos getCurrentPos() const override { return _simRocket.getGimbalPos(); }
private:
    SimRocket &_simRocket;
    float _limit_deg;
};

} // namespace simHardware