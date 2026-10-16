#pragma once

namespace hardware {

class IMotorIgniter {
public:
    virtual ~IMotorIgniter() = default;

    virtual bool canIgnite() = 0;
    virtual bool ignite() = 0;
    // Stops the ignition output, not an already-burning motor.
    virtual bool stopIgniting() = 0;
};

} // namespace hardware