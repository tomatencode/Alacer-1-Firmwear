#pragma once

#include <type_traits>
#include "config/BuildConfig.hpp"
#include "config/GimbalGears.hpp"
#include "hardwareIO/imu/ICM45686.hpp"
#include "hardwareIO/barometer/MS5611.hpp"
#include "hardwareIO/pyro/PyroArmManager.hpp"
#include "hardwareIO/pyro/PyroChannel.hpp"
#include "hardwareComponents/gimbal/Gimbal.hpp"
#include "hardwareComponents/motor/MotorIgniter.hpp"
#include "hardwareComponents/parachute/Parachute.hpp"
#include "simMode/SimulationStepper.hpp"
#include "simMode/mockHardware/hardwareIO/SimIMU.hpp"
#include "simMode/mockHardware/hardwareIO/SimBarometer.hpp"
#include "simMode/mockHardware/hardwareIO/SimPyroArmManager.hpp"
#include "simMode/mockHardware/hardwareIO/SimPyroChannel.hpp"
#include "simMode/mockHardware/hardwareComponents/SimGimbal.hpp"
#include "simMode/mockHardware/hardwareComponents/SimMotorIgniter.hpp"
#include "simMode/mockHardware/hardwareComponents/SimParachute.hpp"

struct RealFlightHardware {
    Clock clock;
    hardware::ICM45686 imu;
    hardware::MS5611 barometer;
    hardware::Servo pitchServo{clock, PA1, 2, 0.0f, 180.0f, 90.0f, 544, 2400, 180.0f};
    hardware::Servo yawServo{clock, PA2, 3, 0.0f, 180.0f, 90.0f, 544, 2400, 180.0f};
    hardware::PyroArmManager pyroArmManager{PC13};
    hardware::PyroChannel pyroChannel1{clock, PB5, PB4, pyroArmManager};
    hardware::PyroChannel pyroChannel2{clock, PB6, PB7, pyroArmManager};
    hardware::PyroChannel pyroChannel3{clock, PB8, PB9, pyroArmManager};
    hardware::PyroMotorIgniter motorIgniter;
    hardware::Parachute parachute;
    hardware::Gimbal gimbal{
        pitchServo, yawServo, {0.0f, 0.0f},
        -GIMBAL_LIMIT_deg, GIMBAL_LIMIT_deg, -GIMBAL_LIMIT_deg, GIMBAL_LIMIT_deg,
        GIMBAL_PITCH_GEAR_RATIO, GIMBAL_YAW_GEAR_RATIO,
        GIMBAL_PITCH_SERVO_OFFSET_deg, GIMBAL_YAW_SERVO_OFFSET_deg};

    RealFlightHardware(SPIClass& spi, IMURocketCoordinateConverter&)
        : imu(PB10, spi), barometer(PB3, spi) {}

    void begin() {
        pyroArmManager.begin();
        pyroChannel1.begin();
        pyroChannel2.begin();
        pyroChannel3.begin();
        imu.begin();
        barometer.begin();
        pitchServo.begin();
        yawServo.begin();
        gimbal.begin();
    }

    void update() {}
};

struct SimulatedFlightHardware {
    SimClock clock;
    SimRocket rocket{clock};
    SimulationStepper stepper{clock, buildConfig::simulationSpeed};
    simHardware::SimIMU imu;
    hardware::SimBarometer barometer{rocket};
    simHardware::SimPyroArmManager pyroArmManager;
    simHardware::SimPyroChannel pyroChannel1{clock, pyroArmManager};
    simHardware::SimPyroChannel pyroChannel2{clock, pyroArmManager};
    simHardware::SimPyroChannel pyroChannel3{clock, pyroArmManager};
    simHardware::SimMotorIgniter motorIgniter{rocket};
    simHardware::SimParachute parachute{rocket};
    simHardware::SimGimbal gimbal{rocket, GIMBAL_LIMIT_deg};

    SimulatedFlightHardware(SPIClass&, IMURocketCoordinateConverter& converter)
        : imu(rocket, converter) {}

    void begin() {
        pyroArmManager.setHardwareArmed(true);
        pyroChannel1.setContinuity(true);
        pyroChannel2.setContinuity(true);
        pyroChannel3.setContinuity(true);
        gimbal.begin();
        rocket.update();
        stepper.begin();
    }

    void update() {
        stepper.update();
        rocket.update();
    }
};

using FlightHardware = std::conditional_t<buildConfig::simulation,
                                         SimulatedFlightHardware, RealFlightHardware>;