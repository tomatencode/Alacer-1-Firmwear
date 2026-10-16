#include "FlightSequenceExecutor.hpp"
#include "etl/string.h"

FlightSequenceExecutor::FlightSequenceExecutor(
    const Clock& clock,
    logging::LogManager& logManager,
    ControlPD& controlPID,
    RotationAccumulator& rotationAccumulator,
    BarometricHeightCalculator& barometricHeightCalculator,
    VerticalMovementTracker& verticalMovementTracker,
    HorizontalMovementTracker& horizontalMovementTracker,
    hardware::IMotorIgniter& motorIgniter,
    hardware::IParachute& parachute)
    : _clock(clock),
      _logManager(logManager),
      _controlPID(controlPID),
      _rotationAccumulator(rotationAccumulator),
      _barometricHeightCalculator(barometricHeightCalculator),
      _verticalMovementTracker(verticalMovementTracker),
      _horizontalMovementTracker(horizontalMovementTracker),
      _motorIgniter(motorIgniter),
      _parachute(parachute)
{}

bool FlightSequenceExecutor::configureForFlight(const FlightProfile& profile) {
    _barometricHeightCalculator.calibrateTo(0.0f);

    _rotationAccumulator.stopAccumulation();
    _rotationAccumulator.setRotationQuaternion(profile.initialRotation);

    _controlPID.setPDParameters(profile.pidKp, profile.pidKd);
    _controlPID.setTarget(Eigen::Quaternionf::Identity());

    _verticalMovementTracker.reset();
    _horizontalMovementTracker.reset();


    bool logStarted = _logManager.startLog(profile.name);

    _logManager.appendEvent(LogProtocol::FlightConfigurationEvent{
        .name = profile.name,
        .burnPhaseDuration_ms = profile.burnPhaseDuration_ms,
        .pidKp = profile.pidKp,
        .pidKd = profile.pidKd,
        .initialRotation = profile.initialRotation,
    });

    return logStarted;
}

bool FlightSequenceExecutor::preflightChecks() const {
    if (!_motorIgniter.canIgnite()) return false;
    if (!_parachute.canDeploy()) return false;
    if (!_controlPID.canStartControlling()) return false;
    if (!_logManager.isLogging()) return false;

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

void FlightSequenceExecutor::finishFlight() {
    _controlPID.stopControlling();
    _verticalMovementTracker.stopTracking();
    _horizontalMovementTracker.stopTracking();
    _logManager.finishLog();
}

void FlightSequenceExecutor::resetPosEstimate() {
    _verticalMovementTracker.reset();
    _horizontalMovementTracker.reset();
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

bool FlightSequenceExecutor::hasTouchedDown() const {
    static constexpr float TOUCHDOWN_HEIGHT_TOLERANCE_m = 3.0f;
    static constexpr float TOUCHDOWN_VELOCITY_m_s = 1.0f;

    return _verticalMovementTracker.hasVelocityEstimate() &&
           std::abs(_verticalMovementTracker.getVelocity_m_s()) <= TOUCHDOWN_VELOCITY_m_s &&
           _verticalMovementTracker.getHeight_m() <= TOUCHDOWN_HEIGHT_TOLERANCE_m;
}
