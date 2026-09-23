#include "FlightStateManager.hpp"

FlightStateManager::FlightStateManager(RotationAccumulator& rotationAccumulator, VerticalMovementTracker& verticalMovementTracker, MotorIgniter& motorIgniter, Parashoot& parashoot) 
    : _rotationAccumulator(rotationAccumulator), _verticalMovementTracker(verticalMovementTracker), _motorIgniter(motorIgniter), _parashoot(parashoot) {}

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
    case FlightState::COASTING:
        _parashoot.deploy();
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

    _currentState = FlightState::COUNTDOWN;
    _countdownStartTime = millis();
    return true;
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
            launch();
            _currentState = FlightState::BURNING;
        }
        break;
    case FlightState::BURNING:
        if (millis() - _motorStartBurnTime >= MOTOR_BURN_DURATION_ms) {
            // stop PID
            _currentState = FlightState::COASTING;
        }
        break;
    case FlightState::COASTING:
        if (_verticalMovementTracker.getVelocity_m_s() <= 0.0f && _verticalMovementTracker.hasVelocityEstimate()) {
            _parashoot.deploy();
            _currentState = FlightState::DESCENDING;
        }
        break;
    case FlightState::DESCENDING:
        if (_verticalMovementTracker.getHeight_m() <= 2.0f && _verticalMovementTracker.hasVelocityEstimate() && _verticalMovementTracker.getVelocity_m_s() <= 1.0f) {
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


bool FlightStateManager::preflightChecks() {

    if (!_motorIgniter.canLight()) return false;
    if (!_parashoot.isDeployed()) return false;

    return true;
}

void FlightStateManager::launch() {
    _motorIgniter.light();
    _rotationAccumulator.startAccumulation();
    _verticalMovementTracker.reset();
    _motorStartBurnTime = millis();
    // start PID
}