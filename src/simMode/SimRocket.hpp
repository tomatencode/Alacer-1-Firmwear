#pragma once

#include <cstdint>
#include <ArduinoEigen.h>

#include "hardwareComponents/gimbal/Gimbal.hpp"
#include "simMode/physics/RigidBody.hpp"
#include "timing/Clock.hpp"

class SimRocket {
public:
    SimRocket(Clock& clock);
    
    void igniteMotor();
    void deployParachute();

    void setGimbalPos(const hardware::Gimbal::GimbalPos& gimbalPos);

    hardware::Gimbal::GimbalPos getGimbalPos() { return gimbalPos; }

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
    static constexpr float mass_kg = 0.17;
    static inline const Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
    static constexpr float gimbalLever_m = 0.1f;
    float motorThrust_N(float time_s) { // upgrade later
        if (time_s > 5.0f) {
            return 3.0f;
        }
        return 0.0f;
    }

    // position of the center of pressure along the rocket's body axis
    // relative to the center of mass, positive in front of the center of mass
    static constexpr float centerOfPressure_m = 0.05f;
    static constexpr float parachuteCenterOfPressure_m = 0.2f;

    static constexpr float dragCoefficient = 0.5f;
    static constexpr float crossSectionalArea_m2 = 0.01f;

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

    hardware::Gimbal::GimbalPos gimbalPos;

    uint32_t lastUpdateTime;
    bool lastUpdateTimeInitialized;

    Clock& clock;
};