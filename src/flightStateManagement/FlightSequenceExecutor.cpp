#include "FlightSequenceExecutor.hpp"

FlightSequenceExecutor::FlightSequenceExecutor(
    LogManager& logManager,
    ControlPID& controlPID,
    RotationAccumulator& rotationAccumulator,
    BarometricHeightCalculator& barometricHeightCalculator,
    VerticalMovementTracker& verticalMovementTracker,
    HorizontalMovementTracker& horizontalMovementTracker,
    MotorIgniter& motorIgniter,
    Parachute& parachute)
    : _logManager(logManager),
      _controlPID(controlPID),
      _rotationAccumulator(rotationAccumulator),
      _barometricHeightCalculator(barometricHeightCalculator),
      _verticalMovementTracker(verticalMovementTracker),
      _horizontalMovementTracker(horizontalMovementTracker),
      _motorIgniter(motorIgniter),
      _parachute(parachute)
{}

bool FlightSequenceExecutor::configureForFlight(const FlightProfile& profile) {
    if (profile.parachutePyroChannel == nullptr || profile.motorIgniterChannel == nullptr) {
        return false;
    }

    _parachute.setPyroChannel(*profile.parachutePyroChannel);
    _motorIgniter.setPyroChannel(*profile.motorIgniterChannel);

    _barometricHeightCalculator.calibrateTo(profile.initialHeight_m);

    _rotationAccumulator.stopAccumulation();
    _rotationAccumulator.setRotationQuaternion(profile.initialRotation);

    _controlPID.setPIDParameters(profile.pidKp, profile.pidKi, profile.pidKd);
    _controlPID.setTarget(profile.targetAngle);

    _verticalMovementTracker.reset();
    _horizontalMovementTracker.reset();

    _logManager.appendEvent(LogProtocol::FlightConfigurationEvent{
        .initialRotation = profile.initialRotation,
        .targetAngle = profile.targetAngle,
        .pidKp = profile.pidKp,
        .pidKi = profile.pidKi,
        .pidKd = profile.pidKd,
        .initialHeight_m = profile.initialHeight_m,
    });

    return true;
}

bool FlightSequenceExecutor::preflightChecks() const {
    if (!_motorIgniter.canIgnite()) return false;
    if (!_parachute.canDeploy()) return false;
    if (!_controlPID.canStartControlling()) return false;
    if (_logManager.isLogging()) return false;

    return true;
}

bool FlightSequenceExecutor::launch() {
    if (!_motorIgniter.ignite()) {
        return false;
    }

    _rotationAccumulator.startAccumulation();
    _verticalMovementTracker.startTracking();
    _horizontalMovementTracker.startTracking();
    _controlPID.startControlling();

    return true;
}

void FlightSequenceExecutor::finishBurn() {
    _controlPID.stopControlling();
    _horizontalMovementTracker.setAscentStage(HorizontalMovementTracker::AscentStage::COASTING);
}

bool FlightSequenceExecutor::deployForDescent() {
    const bool deployed = _parachute.deploy();
    _horizontalMovementTracker.setAscentStage(HorizontalMovementTracker::AscentStage::DESCENDING_CHUTE);
    return deployed;
}

void FlightSequenceExecutor::finishLanding() {
    _verticalMovementTracker.stopTracking();
    _horizontalMovementTracker.stopTracking();
}

void FlightSequenceExecutor::abortThrust() {
    _controlPID.stopControlling();
    _motorIgniter.stopIgniting();
}

bool FlightSequenceExecutor::deployParachute() {
    return _parachute.deploy();
}

bool FlightSequenceExecutor::hasReachedApogee() const {
    return _verticalMovementTracker.hasVelocityEstimate() && _verticalMovementTracker.getVelocity_m_s() <= 0.0f;
}

bool FlightSequenceExecutor::hasTouchedDown(float initialHeight_m) const {
    static constexpr float TOUCHDOWN_HEIGHT_TOLERANCE_m = 3.0f;
    static constexpr float TOUCHDOWN_VELOCITY_m_s = 1.0f;

    return _verticalMovementTracker.hasVelocityEstimate() &&
           _verticalMovementTracker.getVelocity_m_s() <= TOUCHDOWN_VELOCITY_m_s &&
           _verticalMovementTracker.getHeight_m() <= initialHeight_m + TOUCHDOWN_HEIGHT_TOLERANCE_m;
}
