#include "SimRocket.hpp"

#include <numbers>

SimRocket::SimRocket(Clock& clock)
    : rigidBody(clock, mass_kg, inertia, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()),
      motorIgnited(false),
      motorIgnitionTime_ms(0),
      parachuteDeployed(false),
      gimbalPos{0.0f, 0.0f},
      lastUpdateTime(0),
      lastUpdateTimeInitialized(false),
      clock(clock) {
}

void SimRocket::deployParachute() {
    parachuteDeployed = true;
}

void SimRocket::igniteMotor() {
    motorIgnited = true;
    motorIgnitionTime_ms = clock.getTimeFlight_ms();
    parachuteDeployed = false;
}

void SimRocket::setGimbalPos(const hardware::IGimbal::GimbalPos& pos) {
    gimbalPos = pos;
}

void SimRocket::applyAerodynamicDrag(float coefficient, float area_m2, float centerOfPressure_m) {
    const Eigen::Vector3f offsetBody(0.0f, 0.0f, centerOfPressure_m);
    const Eigen::Vector3f offsetWorld = rigidBody.getOrientation() * offsetBody;

    // Air-relative velocity at the pressure center includes motion caused by rotation.
    const Eigen::Vector3f airRelativeVelocityWorld =
        rigidBody.getVelocityWorld() - windVelocityWorld +
        rigidBody.getAngularVelocityWorld().cross(offsetWorld);

    // Quadratic drag opposes the local airflow; no normalization is needed at rest.
    const Eigen::Vector3f dragForceWorld =
        -0.5f * airDensity_kg_m3 * coefficient * area_m2 *
        airRelativeVelocityWorld.norm() * airRelativeVelocityWorld;

    // Use the body-frame API so both force and lever arm are rotated consistently.
    rigidBody.applyForceBodyAtPointBody(
        rigidBody.getOrientation().conjugate() * dragForceWorld, offsetBody);
}

void SimRocket::update() {
    uint32_t currentTime_us = clock.getTimeFlight_us();
    if (!lastUpdateTimeInitialized) {
        lastUpdateTimeInitialized = true;
        lastUpdateTime = currentTime_us;
        rigidBody.step(); // Initialize the rigid body's clock on the same update.
        return;
    }
    if (currentTime_us == lastUpdateTime) {
        return;
    }
    lastUpdateTime = currentTime_us;

    // Apply gravity force to the rocket's center of mass
    rigidBody.applyForceWorld(Eigen::Vector3f(0.0f, 0.0f, -mass_kg * gravity_m_s2));

    applyAerodynamicDrag(dragCoefficient, crossSectionalArea_m2, centerOfPressure_m);
    if (parachuteDeployed) {
        applyAerodynamicDrag(parachuteDragCoefficient, parachuteCrossSectionalArea_m2,
                             parachuteCenterOfPressure_m);
    }

    if (motorIgnited) {
        const float motorThrust = motorThrust_N(
            static_cast<float>(clock.getTimeFlight_ms() - motorIgnitionTime_ms) * 1e-3f);

        constexpr float degToRad = std::numbers::pi_v<float> / 180.0f;
        const float pitch = gimbalPos.pitch_deg * degToRad;
        const float yaw = gimbalPos.yaw_deg * degToRad;

        const Eigen::Quaternionf gimbalRotation =
            Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY());

        const Eigen::Vector3f motorForce =
            gimbalRotation * Eigen::Vector3f(0.0f, 0.0f, motorThrust);

        rigidBody.applyForceBodyAtPointBody(
            motorForce,
            Eigen::Vector3f(0.0f, 0.0f, gimbalLever_m)
        );
    }

    // Step the rigid body simulation
    rigidBody.step();

    // The pad supports a stationary rocket; stop penetration after touchdown.
    if (getPosition().z() < 0.0f) {
        const auto position = getPosition();
        rigidBody.setPosition({position.x(), position.y(), 0.0f});
        rigidBody.setVelocityWorld(Eigen::Vector3f::Zero());
        rigidBody.setAngularVelocityWorld(Eigen::Vector3f::Zero());
    }
}