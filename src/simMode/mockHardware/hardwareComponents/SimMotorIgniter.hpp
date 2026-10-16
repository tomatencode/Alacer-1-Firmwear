#pragma once

#include "hardwareComponents/motor/IMotorIgniter.hpp"

#include "simMode/SimRocket.hpp"

namespace simHardware {

// Declarations only; simulation behavior is implemented separately.
class SimMotorIgniter : public hardware::IMotorIgniter {
public:
    explicit SimMotorIgniter(SimRocket& simRocket);

    bool canIgnite() override { return true; };
    bool ignite() override { _simRocket.igniteMotor();  return true; };
    bool stopIgniting() override { return true; };

private:
    SimRocket& _simRocket;
};

} // namespace simHardware