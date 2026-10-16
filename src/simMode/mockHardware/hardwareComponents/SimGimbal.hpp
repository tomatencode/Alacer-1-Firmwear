#pragma once

#include "simMode/SimRocket.hpp"
#include "hardwareComponents/gimbal/Gimbal.hpp"

namespace simHardware {

class SimGimbal : public hardware::Gimbal {
public:

    using GimbalPos = hardware::Gimbal::GimbalPos;

    explicit SimGimbal(SimRocket &simRocket);

    void begin() {};

    void setTarget(const GimbalPos& target) { _simRocket.setGimbalPos(target); }

    GimbalPos getTarget() const { return _simRocket.getGimbalPos(); }

    GimbalPos getCurrentPos() const { return _simRocket.getGimbalPos(); }
private:
    SimRocket &_simRocket;
};

} // namespace simHardware