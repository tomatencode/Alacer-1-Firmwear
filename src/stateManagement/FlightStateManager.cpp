#include "FlightStateManager.hpp"

FlightStateManager::FlightStateManager(ControlPID& controlPID, RotationAccumulator& rotationAccumulator, BarometricHeightCalculator& barometricHeightCalculator, VerticalMovementTracker& verticalMovementTracker, HorizontalMovementTracker& horizontalMovementTracker, PyroManager& pyroManager, MotorIgniter& motorIgniter, Parachute& parachute) 
    : _controlPID(controlPID), _rotationAccumulator(rotationAccumulator), _barometricHeightCalculator(barometricHeightCalculator), _verticalMovementTracker(verticalMovementTracker), _horizontalMovementTracker(horizontalMovementTracker), _pyroManager(pyroManager), _motorIgniter(motorIgniter), _parachute(parachute) {}

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
        _motorIgniter.stopIgniting();
        bool success = _parachute.deploy();
        if (!success) {
            return false;
        }
        _currentState = FlightState::ABORTED;
        return true;
    case FlightState::COASTING:
        bool success = _parachute.deploy();
        if (!success) {
            return false;
        }
        _currentState = FlightState::ABORTED;
        return true;
    
    default:
        return false;
    }
}


bool FlightStateManager::startCountdown(FlightProfile flightProfile) {
    if (_currentState != FlightState::IDLE) {
        return false;
    }

    _flightProfile = flightProfile;
    
    bool configSuccess = configureForFlight();
    bool checksSuccess = preflightChecks();

    if (configSuccess && checksSuccess) {
        _countdownStartTime = millis();
        _currentState = FlightState::COUNTDOWN;
        return true; // Countdown successfully started
    }

    return false; // Failed to start countdown
}

std::optional<uint32_t> FlightStateManager::getCountdownRemaining_ms() const {
    if (_currentState != FlightState::COUNTDOWN) {
        return std::nullopt;
    }

    const uint32_t elapsed = millis() - _countdownStartTime;
    if (elapsed >= _flightProfile.countdownDuration_ms) {
        return std::nullopt;
    }
    return _flightProfile.countdownDuration_ms - elapsed;
}

void FlightStateManager::update() {
    switch (_currentState)
    {
    case FlightState::IDLE:
        // Handle IDLE state
        break;
    case FlightState::COUNTDOWN:
        // Handle COUNTDOWN state
        if (millis() - _countdownStartTime >= _flightProfile.countdownDuration_ms) {
            launchSequence();
            _currentState = FlightState::BURNING;
        }
        break;
    case FlightState::BURNING:
        if (millis() - _motorStartBurnTime >= _flightProfile.motorBurnDuration_ms) {
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
        if (_verticalMovementTracker.getHeight_m() <= _flightProfile.initialHeight_m + 3.0f && _verticalMovementTracker.hasVelocityEstimate() && _verticalMovementTracker.getVelocity_m_s() <= 1.0f) {
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
    return _currentState != FlightState::IDLE && _currentState != FlightState::LANDED && _currentState != FlightState::ABORTED;
}

bool FlightStateManager::preflightChecks() {

    if (!_motorIgniter.canIgnite()) return false;
    if (!_parachute.canDeploy()) return false;
    if (!_controlPID.canStartControlling()) return false;

    return true;
}

bool FlightStateManager::configureForFlight() {
    std::optional<PyroChannel*> parachuteChannelOpt = _pyroManager.getPyroChannel(_flightProfile.parachutePyroChannel);
    std::optional<PyroChannel*> motorChannelOpt = _pyroManager.getPyroChannel(_flightProfile.motorIgniterChannel);
    
    if (!parachuteChannelOpt.has_value() || !motorChannelOpt.has_value()) {
        return false;
    }

    PyroChannel* parachuteChannel = parachuteChannelOpt.value();
    PyroChannel* motorChannel = motorChannelOpt.value();

    if (parachuteChannel == nullptr || motorChannel == nullptr) {
        return false;
    }

    _parachute.setPyroChannel(*parachuteChannel);
    _motorIgniter.setPyroChannel(*motorChannel);

    _barometricHeightCalculator.calibrateTo(_flightProfile.initialHeight_m);

    _rotationAccumulator.stopAccumulation();
    _rotationAccumulator.setRotationQuaternion(_flightProfile.initialRotation);

    _controlPID.setPIDParameters(_flightProfile.pidKp, _flightProfile.pidKi, _flightProfile.pidKd);
    _controlPID.setTarget(_flightProfile.targetAngle);

    _verticalMovementTracker.reset();
    _horizontalMovementTracker.reset();

    return true;
}

bool FlightStateManager::launchSequence() {
    bool success = _motorIgniter.ignite();

    if (!success) {
        return false;
    }

    _rotationAccumulator.startAccumulation();
    _verticalMovementTracker.startTracking();
    _horizontalMovementTracker.startTracking();
    _controlPID.startControlling();
    
    _motorStartBurnTime = millis();

    return true;
}