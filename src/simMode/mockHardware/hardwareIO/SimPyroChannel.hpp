#pragma once

#include "hardwareIO/pyro/IPyroChannel.hpp"
#include "hardwareIO/pyro/IPyroArmManager.hpp"
#include "timing/Clock.hpp"

namespace simHardware {

// No GPIO access. Pulse duration is milliseconds in flight time.
class SimPyroChannel : public hardware::IPyroChannel {
public:
    SimPyroChannel(Clock& clock, hardware::IPyroArmManager& pyroArmManager)
        : _clock(clock), _pyroArmManager(pyroArmManager) {}

    void begin() override {};
    bool canFire() override { return _hasContinuity && _pyroArmManager.isArmed() && !_isFiring; };
    bool fire(uint32_t duration) override {
        if (canFire()) {
            _isFiring = true;
            _fireTime = _clock.getTimeFlight_ms();
            _fireDuration = duration;
            return true;
        }
        return false;
    };
    void stopFiring() override { _isFiring = false; };
    bool hasContinuity() override { return _hasContinuity; };
    void update() override {
        if (_isFiring && (!_pyroArmManager.isArmed() ||
            _clock.getTimeFlight_ms() - _fireTime >= _fireDuration)) {
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