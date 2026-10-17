#pragma once

#include <cstdint>
#include <numbers>
#include <ArduinoEigen.h>

#include "hardwareComponents/gimbal/IGimbal.hpp"
#include "simMode/physics/RigidBody.hpp"
#include "timing/Clock.hpp"

class SimRocket {
public:
    SimRocket(Clock& clock);
    
    void igniteMotor();
    void deployParachute();

    void setGimbalPos(const hardware::IGimbal::GimbalPos& gimbalPos);

    hardware::IGimbal::GimbalPos getGimbalPos() const { return gimbalPos; }

    Eigen::Vector3f getPosition() { return rigidBody.getPosition(); }
    Eigen::Vector3f getVelocityWorld() { return rigidBody.getVelocityWorld(); }
    Eigen::Vector3f getVelocityBody() { return rigidBody.getVelocityBody(); }
    Eigen::Vector3f getAccelerationWorld() { return rigidBody.getAccelerationWorld(); }
    Eigen::Vector3f getAccelerationBody() { return rigidBody.getAccelerationBody(); }

    Eigen::Quaternionf getOrientation() { return rigidBody.getOrientation(); }
    Eigen::Vector3f getAngularVelocityWorld() { return rigidBody.getAngularVelocityWorld(); }
    Eigen::Vector3f getAngularVelocityBody() { return rigidBody.getAngularVelocityBody(); }
    Eigen::Vector3f getAngularAccelerationWorld() { return rigidBody.getAngularAccelerationWorld(); }
    Eigen::Vector3f getAngularAccelerationBody() { return rigidBody.getAngularAccelerationBody(); }

    void update();
private:
    void applyAerodynamicDrag(float coefficient, float area_m2, float centerOfPressure_m);

    // config
    static constexpr float mass_kg = 0.16;

    static inline const Eigen::Matrix3f inertia = (Eigen::Matrix3f() <<
        9.7e-4, 0.0f, 0.0f,
        0.0f, 9.7e-4, 0.0f,
        0.0f, 0.0f, 6.1e-5).finished();

    static constexpr float gimbalLever_m = 0.13f;
    float motorThrust_N(float time_s) const {
        if (time_s < 0.5) {
            return 10 * time_s; // linear ramp-up of thrust to 5 N
        } else if (time_s < 0.8) {
            return 5 - 10 * (time_s - 0.5f); // linear ramp-down of thrust from 5 N to 2 N
        } else if (time_s < 4.7) {
            return 2.0f; // constant thrust of 2 N
        } else if (time_s < 5.0) {
            return 2.0f * (5.0f - time_s) / 0.3f; // linear ramp-down of thrust from 2 N to 0 N over 0.3 s
        } else {
            return 0.0f; // no thrust after 5 s
        }
    }

    // position of the center of pressure along the rocket's body axis
    // relative to the center of mass, positive in front of the center of mass
    static constexpr float centerOfPressure_m = 0.05f;
    static constexpr float parachuteCenterOfPressure_m = 0.2f;

    static constexpr float dragCoefficient = 0.5f;
    static constexpr float crossSectionalArea_m2 = 0.019; // 2 * pi * (0.055m)²

    static constexpr float parachuteDragCoefficient = 1.5f;
    static constexpr float parachuteCrossSectionalArea_m2 = 0.1f;

    static constexpr float gravity_m_s2 = 9.81f;
    static constexpr float airDensity_kg_m3 = 1.225f;
    static inline const Eigen::Vector3f windVelocityWorld = Eigen::Vector3f::Zero();

    // dynamic state of the rocket
    RigidBody rigidBody;

    bool motorIgnited;
    uint32_t motorIgnitionTime_ms;

    bool parachuteDeployed;

    hardware::IGimbal::GimbalPos gimbalPos;

    uint32_t lastUpdateTime;
    bool lastUpdateTimeInitialized;

    Clock& clock;
};