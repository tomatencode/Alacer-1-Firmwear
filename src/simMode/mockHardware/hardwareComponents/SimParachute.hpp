#pragma once

#include "hardwareComponents/parachute/IParachute.hpp"
#include "simMode/SimRocket.hpp"

namespace simHardware {

// Declarations only; simulation behavior is implemented separately.
class SimParachute : public hardware::IParachute {
public:
    explicit SimParachute(SimRocket& simRocket);

    bool canDeploy() override { return !_didDeploy; };
    bool deploy() override { _simRocket.deployParachute(); _didDeploy = true; return true; };
    bool stopDeploying() override { return true; };

    void reset() override { _didDeploy = false; };
    bool isDeployed() override { return _didDeploy; };

private:
    SimRocket& _simRocket;
    bool _didDeploy = false;
};

} // namespace simHardware