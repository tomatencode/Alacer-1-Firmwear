#include "FlightStateManager.hpp"

FlightStateManager::FlightStateManager(
    const Clock& clock,
    logging::LogManager& logManager,
    ControlPD& controlPID,
    RotationAccumulator& rotationAccumulator,
    BarometricHeightCalculator& barometricHeightCalculator,
    VerticalMovementTracker& verticalMovementTracker,
    HorizontalMovementTracker& horizontalMovementTracker,
    hardware::IMotorIgniter& motorIgniter,
    hardware::IParachute& parachute)
    : _clock(clock), _logManager(logManager),
      _executor(clock,
        logManager,
        controlPID,
        rotationAccumulator,
        barometricHeightCalculator,
        verticalMovementTracker,
        horizontalMovementTracker,
        motorIgniter,
        parachute)
{}

FlightState FlightStateManager::getCurrentState() const {
    return _currentState;
}

bool FlightStateManager::trySetIdle() {
    if (_currentState == FlightState::IDLE) {
        return false; // Already in IDLE state
    }

    if (_currentState == FlightState::COUNTDOWN || _currentState == FlightState::ABORTED || _currentState == FlightState::LANDED) {
        changeState(FlightState::IDLE);
        _executor.finishFlight();
        _executor.resetPosEstimate();

        return true;
    }
    return false; // Deny setting to IDLE if not allowed
}

bool FlightStateManager::abort() {
    switch (_currentState)
    {
    case FlightState::COUNTDOWN:
        _countdownStartTime = 0;
        changeState(FlightState::ABORTED);
        return true;
    case FlightState::BURNING: {
        _executor.abortThrust();
        // Always enter ABORTED (no half-abort): even if the chute fails,
        // thrust is already cut, so staying in BURNING would drift from hardware.
        const bool chuteOk = _executor.deployParachute();
        changeState(FlightState::ABORTED);
        return chuteOk;
    }
    case FlightState::COASTING:
    case FlightState::DESCENDING: // parachute deployment might have failed
    case FlightState::LANDED: // parachute deployment might have failed and falsely enterd landed state
    {
        const bool chuteOk = _executor.deployParachute();
        changeState(FlightState::ABORTED);
        return chuteOk;
    }
    
    default:
        return false;
    }
}

bool FlightStateManager::retryDeployParachute() {
    if (_currentState != FlightState::ABORTED && _currentState != FlightState::DESCENDING && _currentState != FlightState::LANDED) {
        return false;
    }
    return _executor.deployParachute();
}


bool FlightStateManager::startCountdown(FlightProfile flightProfile) {
    if (_currentState != FlightState::IDLE) {
        return false;
    }

    _flightProfile = flightProfile;
    
    bool configSuccess = _executor.configureForFlight(_flightProfile);
    //bool checksSuccess = _executor.preflightChecks();
    bool checksSuccess = true; // testing

    if (configSuccess && checksSuccess) {
        _countdownStartTime = _clock.getTimeFlight_ms();
        changeState(FlightState::COUNTDOWN);
        return true; // Countdown successfully started
    }

    _executor.finishFlight();

    return false; // Failed to start countdown
}

std::optional<uint32_t> FlightStateManager::getCountdownRemaining_ms() const {
    if (_currentState != FlightState::COUNTDOWN) {
        return std::nullopt;
    }

    const uint32_t elapsed = _clock.getTimeFlight_ms() - _countdownStartTime;
    if (elapsed >= COUNTDOWN_DURATION_MS) {
        return std::nullopt;
    }
    return COUNTDOWN_DURATION_MS - elapsed;
}

void FlightStateManager::update() {
    switch (_currentState)
    {
    case FlightState::IDLE:
        // Handle IDLE state
        break;
    case FlightState::COUNTDOWN:
        // Handle COUNTDOWN state
        if (_clock.getTimeFlight_ms() - _countdownStartTime >= COUNTDOWN_DURATION_MS) {
            if (_executor.launch()) {
                _motorStartBurnTime = _clock.getTimeFlight_ms();
                changeState(FlightState::BURNING);
            } else {
                // Ignition failed: do not enter BURNING, fail safe to ABORTED.
                changeState(FlightState::ABORTED);
            }
        }
        break;
    case FlightState::BURNING:
        if (_clock.getTimeFlight_ms() - _motorStartBurnTime >= _flightProfile.burnPhaseDuration_ms) {
            _executor.finishBurn();
            changeState(FlightState::COASTING);
        }
        break;
    case FlightState::COASTING:
        if (_executor.hasReachedApogee()) {
            _executor.deployForDescent();
            changeState(FlightState::DESCENDING);
        }
        break;
    case FlightState::DESCENDING:
        if (_executor.hasTouchedDown()) {
            _executor.finishFlight();
            changeState(FlightState::LANDED);
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
