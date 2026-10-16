#pragma once

#include "hardwareIO/pyro/IPyroChannel.hpp"
#include "hardwareIO/pyro/IPyroArmManager.hpp"
#include "timing/Clock.hpp"

namespace simHardware {

// Declarations only; no GPIO access. Timing uses the simulation clock.
class SimPyroChannel : public hardware::IPyroChannel {
public:
    SimPyroChannel(Clock& clock, hardware::IPyroArmManager& pyroArmManager);

    void begin() override {};
    bool canFire() override { return _hasContinuity && _pyroArmManager.isArmed() && !_isFiring; };
    bool fire(uint32_t duration) override {
        if (canFire()) {
            _isFiring = true;
            _fireTime = _clock.getTimeFlight_us();
            _fireDuration = duration;
            return true;
        }
        return false;
    };
    void stopFiring() override { _isFiring = false; };
    bool hasContinuity() override { return _hasContinuity; };
    void update() override {
        if (_isFiring && (_clock.getTimeFlight_us() - _fireTime >= _fireDuration)) {
            _isFiring = false;
        }
    };

    void setContinuity(bool continuity) { _hasContinuity = continuity; };
    bool isFiring() const { return _isFiring; };

private:
    Clock& _clock;
    hardware::IPyroArmManager& _pyroArmManager;
    bool _hasContinuity = false;
    bool _isFiring = false;
    uint32_t _fireTime = 0;
    uint32_t _fireDuration = 0;
};

} // namespace simHardware