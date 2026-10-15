#pragma once

#include <cstdint>
#include <ArduinoEigen.h>

#include "simMode/mockHardware/hardwareComponents/SimGimbal.hpp"
#include "simMode/physics/RigidBody.hpp"
#include "simMode/timing/SimClock.hpp"

class SimRocket {
public:
    SimRocket(SimClock& clock);
    
    void igniteMotor();
    void deployParachute();

    void setGimbalPos(const simHardware::SimGimbal::GimbalPos& gimbalPos);

    simHardware::SimGimbal::GimbalPos getGimbalPos() { return gimbalPos; }

    Eigen::Vector3f getPosition() { return RigidBody.getPosition(); }
    Eigen::Vector3f getVelocityWorld() { return RigidBody.getVelocityWorld(); }
    Eigen::Vector3f getVelocityBody() { return RigidBody.getVelocityBody(); }
    Eigen::Vector3f getAccelerationWorld() { return RigidBody.getAccelerationWorld(); }
    Eigen::Vector3f getAccelerationBody() { return RigidBody.getAccelerationBody(); }

    Eigen::Quaternionf getOrientation() { return RigidBody.getOrientation(); }
    Eigen::Vector3f getAngularVelocityWorld() { return RigidBody.getAngularVelocityWorld(); }
    Eigen::Vector3f getAngularVelocityBody() { return RigidBody.getAngularVelocityBody(); }
    Eigen::Vector3f getAngularAccelerationWorld() { return RigidBody.getAngularAccelerationWorld(); }
    Eigen::Vector3f getAngularAccelerationBody() { return RigidBody.getAngularAccelerationBody(); }

    void update();
private:
    // config
    float gimbalLever_m;
    float motorThrust_N(float time_s) { // upgrade later
        if (time_s > 5.0f) {
            return 3.0f;
        }
        return 0.0f;
    }

    // position of the center of pressure along the rocket's body axis
    // relative to the center of mass, positive in front of the center of mass
    float centerOfPressure_m;
    float parachuteCenterOfPressure_m;

    float dragCoefficient;
    float crossSectionalArea_m2;

    float parachuteDragCoefficient;
    float parachuteCrossSectionalArea_m2;

    RigidBody RigidBody;

    bool motorIgnited;
    uint32_t motorIgnitionTime_ms;

    bool parachuteDeployed;

    simHardware::SimGimbal::GimbalPos gimbalPos;

    uint32_t lastUpdateTime;

    SimClock& clock;
};