#pragma once

#include <cstdint>

namespace hardware {

class IPyroChannel {
public:
    virtual ~IPyroChannel() = default;

    virtual void begin() = 0;
    virtual bool canFire() = 0;
    virtual bool fire(uint32_t duration) = 0;
    virtual void stopFiring() = 0;
    virtual bool hasContinuity() = 0;
    virtual void update() = 0;
};

} // namespace hardware