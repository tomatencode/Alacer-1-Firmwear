#pragma once

#include <cstdint>
#include <ArduinoEigen.h>


class RigidBody {
public:
    RigidBody(float mass, const Eigen::Matrix3f& inertia);

    void applyForceWorld(const Eigen::Vector3f& force);
    void applyForceBody(const Eigen::Vector3f& force);

    void applyForceAtPointBody(
        const Eigen::Vector3f& force_N,
        const Eigen::Vector3f& offsetFromCenterOfMass_m);

    void applyTorqueWorld(const Eigen::Vector3f& torque);
    void applyTorqueBody(const Eigen::Vector3f& torque);

    void setPosition(const Eigen::Vector3f& pos);
    void setVelocityWorld(const Eigen::Vector3f& vel);
    void setVelocityBody(const Eigen::Vector3f& vel);


    void setOrientation(const Eigen::Quaternionf& ori);
    void setAngularVelocity(const Eigen::Vector3f& angVel);

    Eigen::Vector3f getPosition() const;
    Eigen::Vector3f getVelocityWorld() const;
    Eigen::Vector3f getVelocityBody() const;
    Eigen::Vector3f getAccelerationWorld() const;
    Eigen::Vector3f getAccelerationBody() const;

    Eigen::Quaternionf getOrientation() const;
    Eigen::Vector3f getAngularVelocityWorld() const;
    Eigen::Vector3f getAngularVelocityBody() const;
    Eigen::Vector3f getAngularAccelerationWorld() const;
    Eigen::Vector3f getAngularAccelerationBody() const;

    void step(float dt_s);
private:
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;

    Eigen::Quaternionf orientation;
    Eigen::Vector3f angularVelocity;
    Eigen::Vector3f angularAcceleration;

    float mass;
    Eigen::Matrix3f inertia;

    Eigen::Vector3f accumulatedForce;
    Eigen::Vector3f accumulatedTorque;
};