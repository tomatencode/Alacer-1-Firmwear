#include "timing/Clock.hpp"

static Clock testClock;

#include <Arduino.h>
#include <cassert>
#include <iostream>
#include <type_traits>

#include "hardwareComponents/motor/MotorIgniter.hpp"
#include "hardwareComponents/parachute/Parachute.hpp"
#include "hardwareIO/pyro/PyroArmManager.hpp"
#include "hardwareIO/pyro/PyroChannel.hpp"
#include "simMode/mockHardware/hardwareComponents/SimMotorIgniter.hpp"
#include "simMode/mockHardware/hardwareComponents/SimParachute.hpp"
#include "simMode/mockHardware/hardwareIO/SimPyroArmManager.hpp"
#include "simMode/mockHardware/hardwareIO/SimPyroChannel.hpp"

// These assertions validate declaration-only simulation headers without linking
// simulation implementations or pulling in SimRocket's physics dependencies.
static_assert(std::is_base_of_v<hardware::IMotorIgniter, simHardware::SimMotorIgniter>);
static_assert(std::is_base_of_v<hardware::IParachute, simHardware::SimParachute>);
static_assert(std::is_base_of_v<hardware::IPyroChannel, simHardware::SimPyroChannel>);
static_assert(std::is_base_of_v<hardware::IPyroArmManager, simHardware::SimPyroArmManager>);
static_assert(!std::is_abstract_v<simHardware::SimMotorIgniter>);
static_assert(!std::is_abstract_v<simHardware::SimParachute>);
static_assert(!std::is_abstract_v<simHardware::SimPyroChannel>);
static_assert(!std::is_abstract_v<simHardware::SimPyroArmManager>);
static_assert(std::has_virtual_destructor_v<hardware::IMotorIgniter>);
static_assert(std::has_virtual_destructor_v<hardware::IParachute>);
static_assert(std::has_virtual_destructor_v<hardware::IPyroChannel>);
static_assert(std::has_virtual_destructor_v<hardware::IPyroArmManager>);

class TestPyroChannel : public hardware::IPyroChannel {
public:
    bool ready = false;
    bool firing = false;
    uint32_t duration = 0;

    void begin() override {}
    bool canFire() override { return ready; }
    bool fire(uint32_t duration_ms) override {
        if (!ready) return false;
        duration = duration_ms;
        firing = true;
        return true;
    }
    void stopFiring() override { firing = false; }
    bool hasContinuity() override { return ready; }
    void update() override {}
};

class TestArmManager : public hardware::IPyroArmManager {
public:
    void begin() override {}
    bool isArmed() override { return _armed; }
    bool isHardwareArmed() override { return true; }
    void softwareArm() override { _armed = true; }
    void softwareDisarm() override { _armed = false; }
    bool isSoftwareArmed() override { return _armed; }

private:
    bool _armed = false;
};

static void testComponentsThroughInterfaces() {
    hardware::PyroMotorIgniter motor;
    hardware::Parachute parachute;
    hardware::IMotorIgniter& igniter = motor;
    hardware::IParachute& chute = parachute;

    assert(!igniter.canIgnite());
    assert(!igniter.ignite());
    assert(!igniter.stopIgniting());
    assert(!chute.canDeploy());
    assert(!chute.deploy());
    assert(!chute.stopDeploying());
    assert(!chute.isDeployed());

    TestPyroChannel channel;
    motor.setPyroChannel(channel);
    parachute.setPyroChannel(channel);
    assert(!igniter.ignite());
    assert(!chute.deploy());
    assert(!chute.isDeployed());

    channel.ready = true;
    assert(igniter.canIgnite());
    assert(igniter.ignite());
    assert(channel.firing && channel.duration == 1000);
    assert(igniter.stopIgniting());
    assert(!channel.firing);

    assert(chute.canDeploy());
    assert(chute.deploy());
    assert(channel.firing && channel.duration == 1000);
    assert(chute.isDeployed());
    assert(chute.stopDeploying());
    assert(!channel.firing && chute.isDeployed());
    chute.reset();
    assert(!chute.isDeployed());

    motor.clearPyroChannel();
    parachute.clearPyroChannel();
    assert(!igniter.canIgnite());
    assert(!chute.canDeploy());
}

static void testRealPyroThroughInterfaces() {
    hardware::PyroArmManager realArm(1);
    hardware::IPyroArmManager& arm = realArm;
    arm.begin();
    assert(hostArduino::modes[1] == INPUT);
    assert(!arm.isArmed());
    arm.softwareArm();
    assert(arm.isSoftwareArmed() && !arm.isArmed());
    hostArduino::pins[1] = HIGH;
    assert(arm.isHardwareArmed() && arm.isArmed());

    hardware::PyroChannel realChannel(testClock, 2, 3, arm);
    hardware::IPyroChannel& channel = realChannel;
    channel.begin();
    assert(hostArduino::modes[2] == OUTPUT);
    assert(hostArduino::modes[3] == INPUT);
    assert(!channel.canFire());
    assert(!channel.fire(1000));
    hostArduino::pins[3] = HIGH;
    assert(channel.hasContinuity() && channel.canFire());
    hostArduino::nowMillis = 100;
    assert(channel.fire(1000));
    assert(hostArduino::pins[2] == HIGH);
    hostArduino::pins[3] = LOW;
    assert(channel.hasContinuity());
    hostArduino::nowMillis = 1099;
    channel.update();
    assert(hostArduino::pins[2] == HIGH);
    hostArduino::nowMillis = 1100;
    channel.update();
    assert(hostArduino::pins[2] == LOW);
    assert(!channel.hasContinuity());

    hostArduino::pins[3] = HIGH;
    assert(channel.fire(1000));
    arm.softwareDisarm();
    channel.update();
    assert(hostArduino::pins[2] == LOW);
    assert(!channel.canFire());

    arm.softwareArm();
    assert(channel.fire(1000));
    hostArduino::pins[1] = LOW;
    channel.update();
    assert(hostArduino::pins[2] == LOW);
    assert(!arm.isArmed());

    hostArduino::pins[1] = HIGH;
    assert(channel.fire(1000));
    channel.stopFiring();
    assert(hostArduino::pins[2] == LOW);
}

static void testRealChannelWithAlternateArmManager() {
    TestArmManager arm;
    hardware::PyroChannel channel(testClock, 4, 5, arm);
    hostArduino::pins[5] = HIGH;
    assert(!channel.canFire());
    arm.softwareArm();
    assert(channel.fire(1000));
    assert(hostArduino::pins[4] == HIGH);
    arm.softwareDisarm();
    channel.update();
    assert(hostArduino::pins[4] == LOW);
}

int main() {
    testComponentsThroughInterfaces();
    testRealPyroThroughInterfaces();
    testRealChannelWithAlternateArmManager();
    std::cout << "Hardware interface tests passed\n";
}