#pragma once

#include "hardwareComponents/parachute/Parachute.hpp"
#include "simMode/SimRocket.hpp"

namespace simHardware {

class SimParachute : public hardware::Parachute {
public:
    explicit SimParachute(SimRocket& simRocket) : _simRocket(simRocket) {}

    bool deploy() override {
        if (!hardware::Parachute::deploy()) return false;
        _simRocket.deployParachute();
        return true;
    }

private:
    SimRocket& _simRocket;
};

} // namespace simHardware