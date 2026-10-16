#include <ArduinoEigen.h>
#include "simMode/timing/SimClock.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

#include "hardwareComponents/gimbal/Gimbal.hpp"
#include "simMode/physics/RigidBody.hpp"

// Inspect and seed simulation state without adding test-only production setters.
#define private public
#include "simMode/SimRocket.hpp"
#undef private

static void expectVector(const Eigen::Vector3f& actual, const Eigen::Vector3f& expected) {
    assert(actual.allFinite());
    assert((actual - expected).norm() < 1e-4f);
}

static void testDrag(const Eigen::Vector3f& velocity, bool parachute,
                     const Eigen::Quaternionf& orientation = Eigen::Quaternionf::Identity()) {
    SimClock clock;
    SimRocket rocket(clock);
    rocket.update();
    rocket.rigidBody.setOrientation(orientation);
    rocket.rigidBody.setVelocityWorld(velocity);
    if (parachute) {
        rocket.deployParachute();
        rocket.deployParachute(); // Deployment is idempotent.
    }

    const float bodyScale = 0.5f * rocket.airDensity_kg_m3 * rocket.dragCoefficient *
                            rocket.crossSectionalArea_m2;
    const float chuteScale = parachute ? 0.5f * rocket.airDensity_kg_m3 *
        rocket.parachuteDragCoefficient * rocket.parachuteCrossSectionalArea_m2 : 0.0f;
    const Eigen::Vector3f bodyForce = -bodyScale * velocity.norm() * velocity;
    const Eigen::Vector3f chuteForce = -chuteScale * velocity.norm() * velocity;
    const Eigen::Vector3f expectedAcceleration =
        (bodyForce + chuteForce) / rocket.mass_kg + Eigen::Vector3f(0.0f, 0.0f, -9.81f);
    // The configured inertia is the identity matrix.
    const Eigen::Vector3f expectedTorque =
        (orientation * Eigen::Vector3f(0.0f, 0.0f, rocket.centerOfPressure_m)).cross(bodyForce) +
        (orientation * Eigen::Vector3f(0.0f, 0.0f, rocket.parachuteCenterOfPressure_m)).cross(chuteForce);

    clock.advanceTimeBy_us(1000);
    rocket.update();
    expectVector(rocket.getAccelerationWorld(), expectedAcceleration);
    expectVector(rocket.getAngularAccelerationWorld(), expectedTorque);
    expectVector(rocket.getVelocityWorld(), velocity + expectedAcceleration * 0.001f);

    // No integration or force accumulation when called twice at the same time.
    const Eigen::Vector3f position = rocket.getPosition();
    rocket.update();
    expectVector(rocket.getPosition(), position);
}

static void testRotationalDamping() {
    SimClock clock;
    SimRocket rocket(clock);
    rocket.update();
    rocket.rigidBody.setAngularVelocityWorld(Eigen::Vector3f(2.0f, 0.0f, 0.0f));
    // Use a long enough interval for this tiny damping term to change a float.
    clock.advanceTimeBy_us(1000000);
    rocket.update();

    // omega x r = (0, -0.1, 0) m/s, so drag opposes pitch rotation.
    const float force = 0.5f * rocket.airDensity_kg_m3 * rocket.dragCoefficient *
                        rocket.crossSectionalArea_m2 * 0.1f * 0.1f;
    expectVector(rocket.getAccelerationWorld(), Eigen::Vector3f(0.0f, force / rocket.mass_kg, -9.81f));
    expectVector(rocket.getAngularAccelerationWorld(), Eigen::Vector3f(-0.05f * force, 0.0f, 0.0f));
    assert(rocket.getAngularVelocityWorld().x() < 2.0f);
}

int main() {
    testDrag(Eigen::Vector3f::Zero(), false);
    testDrag(Eigen::Vector3f::Zero(), true);
    testDrag(Eigen::Vector3f(0.0f, 0.0f, 10.0f), false);
    testDrag(Eigen::Vector3f(0.0f, 0.0f, -10.0f), true);
    testDrag(Eigen::Vector3f(3.0f, -4.0f, 5.0f), false);
    testDrag(Eigen::Vector3f(6.0f, -8.0f, 10.0f), false);
    testDrag(Eigen::Vector3f(3.0f, -4.0f, 5.0f), true,
             Eigen::Quaternionf(Eigen::AngleAxisf(0.7f, Eigen::Vector3f::UnitY())));
    testRotationalDamping();
    std::cout << "SimRocket aerodynamics tests passed\n";
}