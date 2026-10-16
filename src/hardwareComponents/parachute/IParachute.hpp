#pragma once

namespace hardware {

class IParachute {
public:
    virtual ~IParachute() = default;

    virtual bool canDeploy() = 0;
    virtual bool deploy() = 0;
    // Stops the deployment output; does not retract the parachute.
    virtual bool stopDeploying() = 0;

    virtual void reset() = 0;
    // Records an accepted deployment command, not confirmed physical deployment.
    virtual bool isDeployed() = 0;
};

} // namespace hardware