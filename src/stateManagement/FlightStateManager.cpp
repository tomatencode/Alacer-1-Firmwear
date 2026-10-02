#include "FlightStateManager.hpp"

FlightStateManager::FlightStateManager(ControlPID& controlPID, RotationAccumulator& rotationAccumulator, BarometricHeightCalculator& barometricHeightCalculator, VerticalMovementTracker& verticalMovementTracker, HorizontalMovementTracker& horizontalMovementTracker, MotorIgniter& motorIgniter, Parachute& parachute) 
    : _controlPID(controlPID), _rotationAccumulator(rotationAccumulator), _barometricHeightCalculator(barometricHeightCalculator), _verticalMovementTracker(verticalMovementTracker), _horizontalMovementTracker(horizontalMovementTracker), _motorIgniter(motorIgniter), _parachute(parachute) {}

FlightState FlightStateManager::getCurrentState() const {
    return _currentState;
}

bool FlightStateManager::trySetIdle() {
    if (_currentState == FlightState::IDLE) {
        return false; // Already in IDLE state
    }

    if (_currentState == FlightState::COUNTDOWN || _currentState == FlightState::ABORTED || _currentState == FlightState::LANDED) {
        _currentState = FlightState::IDLE;
        return true;
    }
    return false; // Deny setting to IDLE if not allowed
}

bool FlightStateManager::abort() {
    switch (_currentState)
    {
    case FlightState::COUNTDOWN:
        _countdownStartTime = 0;
        _currentState = FlightState::ABORTED;
        return true;
    case FlightState::BURNING:
        _controlPID.stopControlling();
        _parachute.deploy();
        _currentState = FlightState::ABORTED;
        return true;
    case FlightState::COASTING:
        _parachute.deploy();
        _currentState = FlightState::ABORTED;
        return true;
    
    default:
        return false;
    }
}


bool FlightStateManager::startCountdown() {
    if (_currentState != FlightState::IDLE) {
        return false;
    }

    if (!preflightChecks()) {
        return false;
    }

    startCountdownSequence();

    return true;
}

std::optional<uint32_t> FlightStateManager::getCountdownRemaining_ms() const {
    if (_currentState != FlightState::COUNTDOWN) {
        return std::nullopt;
    }

    const uint32_t elapsed = millis() - _countdownStartTime;
    if (elapsed >= COUNTDOWN_DURATION_ms) {
        return std::nullopt;
    }
    return COUNTDOWN_DURATION_ms - elapsed;
}

void FlightStateManager::update() {
    switch (_currentState)
    {
    case FlightState::IDLE:
        // Handle IDLE state
        break;
    case FlightState::COUNTDOWN:
        // Handle COUNTDOWN state
        if (millis() - _countdownStartTime >= COUNTDOWN_DURATION_ms) {
            launchSequence();
            _currentState = FlightState::BURNING;
        }
        break;
    case FlightState::BURNING:
        if (millis() - _motorStartBurnTime >= MOTOR_BURN_DURATION_ms) {
            _controlPID.stopControlling();
            _horizontalMovementTracker.setAscentStage(HorizontalMovementTracker::AscentStage::COASTING);
            _currentState = FlightState::COASTING;
        }
        break;
    case FlightState::COASTING:
        if (_verticalMovementTracker.getVelocity_m_s() <= 0.0f && _verticalMovementTracker.hasVelocityEstimate()) {
            _parachute.deploy();
            _horizontalMovementTracker.setAscentStage(HorizontalMovementTracker::AscentStage::DESCENDING_CHUTE);
            _currentState = FlightState::DESCENDING;
        }
        break;
    case FlightState::DESCENDING:
        if (_verticalMovementTracker.getHeight_m() <= 2.0f && _verticalMovementTracker.hasVelocityEstimate() && _verticalMovementTracker.getVelocity_m_s() <= 1.0f) {
            _verticalMovementTracker.stopTracking();
            _horizontalMovementTracker.stopTracking();
            _currentState = FlightState::LANDED;
        }
        break;
    case FlightState::LANDED:
        // Handle LANDED state
        break;
    case FlightState::ABORTED:
        // Handle ABORTED state
        break;
    
    default:
        break;
    }
}
bool FlightStateManager::isMidFlight() const {
    return _currentState == FlightState::BURNING || _currentState == FlightState::COASTING || _currentState == FlightState::DESCENDING;
}

bool FlightStateManager::preflightChecks() {

    if (!_motorIgniter.canIgnite()) return false;
    if (!_parachute.canDeploy()) return false;
    if (!_controlPID.canStartControlling()) return false;

    return true;
}

void FlightStateManager::startCountdownSequence() {
    _countdownStartTime = millis();
    _currentState = FlightState::COUNTDOWN;

    _barometricHeightCalculator.calibrateTo(0);

    _verticalMovementTracker.reset();
    _horizontalMovementTracker.reset();
}

void FlightStateManager::launchSequence() {
    _rotationAccumulator.startAccumulation();
    _verticalMovementTracker.startTracking();
    _horizontalMovementTracker.startTracking();
    _controlPID.startControlling();
    
    _motorStartBurnTime = millis();
    _motorIgniter.ignite();
}