#pragma once

#include <cstdint>
#include <ArduinoEigen.h>

#include "timing/Clock.hpp"


class RigidBody {
public:
    RigidBody(
        Clock& clock,
        float mass,
        const Eigen::Matrix3f& inertia,
        const Eigen::Vector3f& position = Eigen::Vector3f::Zero(),
        const Eigen::Vector3f& velocity = Eigen::Vector3f::Zero()
    );

    void applyForceWorld(const Eigen::Vector3f& force);
    void applyForceBody(const Eigen::Vector3f& force);

    void applyForceBodyAtPointBody(
        const Eigen::Vector3f& force_N,
        const Eigen::Vector3f& offsetFromCenterOfMass_m);
    void applyForceWorldAtPointBody(
        const Eigen::Vector3f& force_N,
        const Eigen::Vector3f& offsetFromCenterOfMass_m);

    void applyTorqueWorld(const Eigen::Vector3f& torque);
    void applyTorqueBody(const Eigen::Vector3f& torque);

    void setPosition(const Eigen::Vector3f& pos);
    void setVelocityWorld(const Eigen::Vector3f& vel);
    void setVelocityBody(const Eigen::Vector3f& vel);


    void setOrientation(const Eigen::Quaternionf& ori);
    void setAngularVelocityWorld(const Eigen::Vector3f& angVel);
    void setAngularVelocityBody(const Eigen::Vector3f& angVel);

    Eigen::Vector3f getPosition() const { return position; };
    Eigen::Vector3f getVelocityWorld() const { return velocity_world; };
    Eigen::Vector3f getVelocityBody() const { return orientation.inverse() * velocity_world; };
    Eigen::Vector3f getAccelerationWorld() const { return acceleration_world; };
    Eigen::Vector3f getAccelerationBody() const { return orientation.inverse() * acceleration_world; };

    Eigen::Quaternionf getOrientation() const { return orientation; };
    Eigen::Vector3f getAngularVelocityWorld() const { return angularVelocity_world; };
    Eigen::Vector3f getAngularVelocityBody() const { return orientation.inverse() * angularVelocity_world; };
    Eigen::Vector3f getAngularAccelerationWorld() const { return angularAcceleration_world; };
    Eigen::Vector3f getAngularAccelerationBody() const { return orientation.inverse() * angularAcceleration_world; };

    void step();
private:
    Eigen::Vector3f position;
    Eigen::Vector3f velocity_world;
    Eigen::Vector3f acceleration_world;

    Eigen::Quaternionf orientation;
    Eigen::Vector3f angularVelocity_world;
    Eigen::Vector3f angularAcceleration_world;

    float mass;
    Eigen::Matrix3f inertia;

    Eigen::Vector3f accumulatedForce_world;
    Eigen::Vector3f accumulatedTorque_world;

    uint32_t lastUpdateTime;
    bool lastUpdateTimeInitialized;
    Clock& clock;
};