#pragma once

#include "FlightTypes.hpp"

#include "../rotationEstimation/RotationAccumulator.hpp"
#include "../hardwareComponents/parachute/Parachute.hpp"
#include "../hardwareComponents/motor/MotorIgniter.hpp"

#include "../barometricHeightCalculation/BarometricHeightCalculator.hpp"
#include "../ascentTracking/VerticalMovementTracker.hpp"
#include "../ascentTracking/HorizontalMovementTracker.hpp"
#include "../controlPID/ControlPID.hpp"
#include "../logManagement/LogManager.hpp"

// FlightSequenceExecutor owns NO state machine state.
// It only touches hardware / estimators. It never decides WHEN to act.
// Only FlightStateManager owns an instance (by value, private), so by
// construction no other code can fire pyro / start control without going
// through the manager's guards (option C: composition, no friend needed).
class FlightSequenceExecutor {
public:
    FlightSequenceExecutor(
        LogManager& logManager,
        ControlPID& controlPID,
        RotationAccumulator& rotationAccumulator,
        BarometricHeightCalculator& barometricHeightCalculator,
        VerticalMovementTracker& verticalMovementTracker,
        HorizontalMovementTracker& horizontalMovementTracker,
        MotorIgniter& motorIgniter,
        Parachute& parachute
    );

    // Dumb hardware sequences. All return false on failure, never change FlightState.
    bool configureForFlight(const FlightProfile& profile);
    bool preflightChecks() const;

    bool launch();
    void finishBurn();
    bool deployForDescent();
    void finishLanding();

    void abortThrust();
    bool deployParachute();

    // Phase-detection observers (read-only view of trackers for the manager).
    bool hasReachedApogee() const;
    bool hasTouchedDown(float initialHeight_m) const;

private:
    LogManager& _logManager;
    ControlPID& _controlPID;
    RotationAccumulator& _rotationAccumulator;
    BarometricHeightCalculator& _barometricHeightCalculator;
    VerticalMovementTracker& _verticalMovementTracker;
    HorizontalMovementTracker& _horizontalMovementTracker;
    MotorIgniter& _motorIgniter;
    Parachute& _parachute;
};
