#include "FlightStateManager.hpp"

FlightStateManager::FlightStateManager(RotationAccumulator& rotationAccumulator) 
    : _rotationAccumulator(rotationAccumulator) {}

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

bool FlightStateManager::startAbort() {
    if (_currentState == FlightState::ABORTED || _currentState == FlightState::ABORTING || _currentState == FlightState::LANDED || _currentState == FlightState::IDLE) {
        return false;
    }
    _currentState = FlightState::ABORTING;
    return true;
}

bool FlightStateManager::isAbortDone() {
    return _currentState == FlightState::ABORTED;
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
        if (millis() - _countdownStartTime >= COUNTDOWN_DURATION) {
            // TODO: light motor here
            _rotationAccumulator.startAccumulation();
            _currentState = FlightState::BURNING;
        }
        break;
    case FlightState::BURNING:
        
        break;
    case FlightState::COASTING:
        // Handle COASTING state
        break;
    case FlightState::DESCENDING:
        // Handle DESCENDING state
        break;
    case FlightState::LANDED:
        // Handle LANDED state
        break;
    case FlightState::ABORTING:
        // Handle ABORTING state
        break;
    case FlightState::ABORTED:
        // Handle ABORTED state
        break;
    
    default:
        break;
    }
}


bool FlightStateManager::preflightChecks() {
    // Perform necessary preflight checks here
    // Return true if all checks pass, false otherwise
    return true;
}