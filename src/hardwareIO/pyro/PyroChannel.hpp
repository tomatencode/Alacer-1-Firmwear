#pragma once
#include "timing/Clock.hpp"

#include <cstdint>

#include "IPyroChannel.hpp"
#include "IPyroArmManager.hpp"

namespace hardware {

class PyroChannel : public IPyroChannel {
public:
    PyroChannel(const Clock& clock, int mosfetPin, int contPin, IPyroArmManager& pyroArmManager);

    void begin() override;

    bool canFire() override { return hasContinuity() && _pyroArmManager.isArmed(); };
    bool fire(uint32_t duration) override;
    void stopFiring() override;

    bool hasContinuity() override;

    void update() override;
private:
    const Clock& _clock;


    int _mosfetPin;
    int _contPin;

    uint32_t _fireTime = 0;
    uint32_t _fireDuration = 0;

    IPyroArmManager& _pyroArmManager;
};

} // namespace hardware
