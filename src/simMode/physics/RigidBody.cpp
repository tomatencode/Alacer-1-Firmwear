#include "RigidBody.hpp"

RigidBody::RigidBody(
        Clock& clock,
        float mass,
        const Eigen::Matrix3f& inertia,
        const Eigen::Vector3f& position,
        const Eigen::Vector3f& velocity
    ) : clock(clock),
      mass(mass),
      inertia(inertia),
      position(position),
      velocity_world(velocity),
      acceleration_world(Eigen::Vector3f::Zero()),
      orientation(Eigen::Quaternionf::Identity()),
      angularVelocity_world(Eigen::Vector3f::Zero()),
      angularAcceleration_world(Eigen::Vector3f::Zero()),
      accumulatedForce_world(Eigen::Vector3f::Zero()),
      accumulatedTorque_world(Eigen::Vector3f::Zero()),
      lastUpdateTime(0),
      lastUpdateTimeInitialized(false) {
}

void RigidBody::step() {
    uint32_t currentTime_us = clock.getTimeFlight_us();

    if (!lastUpdateTimeInitialized) { // First update, initialize lastUpdateTime and return
        lastUpdateTime = currentTime_us;
        lastUpdateTimeInitialized = true;
        accumulatedForce_world.setZero();
        accumulatedTorque_world.setZero();
        return;
    }

    uint32_t deltaTime_us = currentTime_us - lastUpdateTime;
    float deltaTime_s = static_cast<float>(deltaTime_us) * 1e-6f;
    lastUpdateTime = currentTime_us;

    // Update linear motion
    acceleration_world = accumulatedForce_world / mass;
    velocity_world += acceleration_world * deltaTime_s;
    position += velocity_world * deltaTime_s;

    // Update angular motion
    const Eigen::Vector3f omegaBody =
        orientation.conjugate() * angularVelocity_world;

    const Eigen::Vector3f torqueBody =
        orientation.conjugate() * accumulatedTorque_world;

    const Eigen::Vector3f alphaBody =
        inertia.inverse() *
        (torqueBody - omegaBody.cross(inertia * omegaBody));

    angularAcceleration_world = orientation * alphaBody;
    angularVelocity_world += angularAcceleration_world * deltaTime_s;
    orientation = Eigen::Quaternionf(Eigen::AngleAxisf(angularVelocity_world.norm() * deltaTime_s, angularVelocity_world.normalized())) * orientation;
    orientation.normalize();

    // Clear accumulated forces and torques
    accumulatedForce_world.setZero();
    accumulatedTorque_world.setZero();
}

void RigidBody::applyForceWorld(const Eigen::Vector3f& force) {
    accumulatedForce_world += force;
}

void RigidBody::applyForceBody(const Eigen::Vector3f& force) {
    accumulatedForce_world += orientation * force;
}

void RigidBody::applyForceBodyAtPointBody(
        const Eigen::Vector3f& force_N,
        const Eigen::Vector3f& offsetFromCenterOfMass_m) {
    accumulatedForce_world += orientation * force_N;
    accumulatedTorque_world += (orientation * offsetFromCenterOfMass_m).cross(orientation * force_N);
}

void RigidBody::applyForceWorldAtPointBody(
        const Eigen::Vector3f& force_N,
        const Eigen::Vector3f& offsetFromCenterOfMass_m) {
    accumulatedForce_world += force_N;
    accumulatedTorque_world += offsetFromCenterOfMass_m.cross(force_N);
}

void RigidBody::applyTorqueWorld(const Eigen::Vector3f& torque) {
    accumulatedTorque_world += torque;
}

void RigidBody::applyTorqueBody(const Eigen::Vector3f& torque) {
    accumulatedTorque_world += orientation * torque;
}

void RigidBody::setPosition(const Eigen::Vector3f& pos) {
    position = pos;
}

void RigidBody::setVelocityWorld(const Eigen::Vector3f& vel) {
    velocity_world = vel;
}

void RigidBody::setVelocityBody(const Eigen::Vector3f& vel) {
    velocity_world = orientation * vel;
}

void RigidBody::setOrientation(const Eigen::Quaternionf& ori) {
    orientation = ori.normalized();
}

void RigidBody::setAngularVelocityWorld(const Eigen::Vector3f& angVel) {
    angularVelocity_world = angVel;
}
void RigidBody::setAngularVelocityBody(const Eigen::Vector3f& angVel) {
    angularVelocity_world = orientation * angVel;
}
