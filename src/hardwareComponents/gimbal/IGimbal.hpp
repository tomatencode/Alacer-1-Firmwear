#pragma once

namespace hardware {

class IGimbal {
public:
    struct GimbalPos {
        float pitch_deg;
        float yaw_deg;
    };

    virtual ~IGimbal() = default;
    virtual void begin() = 0;
    virtual void setTarget(const GimbalPos& target) = 0;
    virtual GimbalPos getTarget() const = 0;
    virtual GimbalPos getCurrentPos() const = 0;
};

} // namespace hardware