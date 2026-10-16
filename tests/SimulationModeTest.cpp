#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <type_traits>

#include "config/BuildConfig.hpp"
#include "simMode/SimulationStepper.hpp"
#include "simMode/mockHardware/hardwareComponents/SimGimbal.hpp"
#include "simMode/mockHardware/hardwareComponents/SimMotorIgniter.hpp"
#include "simMode/mockHardware/hardwareComponents/SimParachute.hpp"
#include "simMode/mockHardware/hardwareIO/SimIMU.hpp"
#include "simMode/mockHardware/hardwareIO/SimBarometer.hpp"
#include "simMode/mockHardware/hardwareIO/SimPyroArmManager.hpp"
#include "simMode/mockHardware/hardwareIO/SimPyroChannel.hpp"

static_assert(buildConfig::simulation);
static_assert(std::is_base_of_v<hardware::IGimbal, simHardware::SimGimbal>);

static void testStepper() {
    SimClock clock;
    SimulationStepper stepper(clock);
    hostArduino::nowMicros = std::numeric_limits<uint32_t>::max() - 100;
    stepper.begin();
    hostArduino::nowMicros += 200; // Wrap the real-time counter.
    stepper.update();
    assert(clock.getTimeFlight_us() == 200); // Flight time tracks real time at 1:1.
    hostArduino::nowMicros += 2500;
    stepper.update();
    assert(clock.getTimeFlight_us() == 1200); // At most 1 ms per control loop.
    stepper.update();
    assert(clock.getTimeFlight_us() == 2200); // Consume pending real time.
    stepper.update();
    assert(clock.getTimeFlight_us() == 2700); // Consume the remaining partial step.
    stepper.update();
    assert(clock.getTimeFlight_us() == 2700); // No pending or new real time.
}

static void testSimulatedFlight() {
    SimClock clock;
    SimRocket rocket(clock);
    simHardware::SimGimbal gimbal(rocket, 5.0f);
    hardware::IGimbal& controlGimbal = gimbal;
    controlGimbal.begin();
    controlGimbal.setTarget({20.0f, -20.0f});
    assert(controlGimbal.getCurrentPos().pitch_deg == 5.0f);
    assert(controlGimbal.getTarget().yaw_deg == -5.0f);
    controlGimbal.setTarget({0.0f, 0.0f});

    IMURocketCoordinateConverter converter(Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
    simHardware::SimIMU imu(rocket, converter);
    hardware::SimBarometer barometer(rocket);
    const auto padPressure = barometer.getPressure_Pa();
    simHardware::SimPyroArmManager arm;
    simHardware::SimPyroChannel motorChannel(clock, arm), chuteChannel(clock, arm);
    motorChannel.setContinuity(true);
    chuteChannel.setContinuity(true);
    simHardware::SimMotorIgniter motor(rocket);
    simHardware::SimParachute chute(rocket);
    motor.setPyroChannel(motorChannel);
    chute.setPyroChannel(chuteChannel);
    assert(!motor.canIgnite() && !motor.ignite());
    arm.setHardwareArmed(true);
    assert(!motor.canIgnite());
    arm.softwareArm();
    motorChannel.setContinuity(false);
    assert(!motor.canIgnite() && !motor.ignite());
    motorChannel.setContinuity(true);
    rocket.update();

    // Stay on the pad, then ignite long after boot: burn time is ignition-relative.
    for (int i = 0; i < 6000; ++i) {
        clock.advanceTimeBy_us(1000);
        rocket.update();
    }
    assert(rocket.getPosition().z() == 0.0f);
    assert(rocket.getOrientation().coeffs().allFinite());
    assert(motor.ignite());
    clock.advanceTimeBy_ms(999);
    motorChannel.update();
    assert(motorChannel.isFiring());
    clock.advanceTimeBy_ms(1);
    motorChannel.update();
    assert(!motorChannel.isFiring()); // Durations are ms, not us.

    for (int i = 0; i < 1000; ++i) {
        clock.advanceTimeBy_us(1000);
        rocket.update();
    }
    assert(rocket.getPosition().z() > 0.0f);
    assert(barometer.getPressure_Pa() < padPressure);
    assert(std::isfinite(imu.getAccel().z_m_s2));
    assert(std::isfinite(imu.getGyro().x_rad_s));
    assert(chute.deploy() && chute.isDeployed());
    assert(chuteChannel.isFiring());
    arm.softwareDisarm();
    chuteChannel.update();
    assert(!chuteChannel.isFiring());

    for (int i = 0; i < 30000; ++i) {
        clock.advanceTimeBy_us(1000);
        rocket.update();
        assert(rocket.getPosition().allFinite());
        assert(rocket.getPosition().z() >= 0.0f);
    }
    assert(rocket.getPosition().z() == 0.0f);
}

int main() {
    testStepper();
    testSimulatedFlight();
    std::cout << "Simulation mode tests passed\n";
}