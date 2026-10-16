#pragma once

#include "hardwareComponents/motor/MotorIgniter.hpp"

#include "simMode/SimRocket.hpp"

namespace simHardware {

class SimMotorIgniter : public hardware::PyroMotorIgniter {
public:
    explicit SimMotorIgniter(SimRocket& simRocket) : _simRocket(simRocket) {}

    bool ignite() override {
        if (!hardware::PyroMotorIgniter::ignite()) return false;
        _simRocket.igniteMotor();
        return true;
    }

private:
    SimRocket& _simRocket;
};

} // namespace simHardware