#pragma once

#include "FlightTypes.hpp"

#include "../rotationEstimation/RotationAccumulator.hpp"
#include "../hardwareComponents/parachute/IParachute.hpp"
#include "../hardwareComponents/motor/IMotorIgniter.hpp"

#include "../barometricHeightCalculation/BarometricHeightCalculator.hpp"
#include "../ascentTracking/VerticalMovementTracker.hpp"
#include "../ascentTracking/HorizontalMovementTracker.hpp"
#include "../controlPD/ControlPD.hpp"
#include "../logManagement/LogManager.hpp"

// FlightSequenceExecutor owns NO state machine state.
// It only touches hardware / estimators. It never decides WHEN to act.
// Only FlightStateManager owns an instance (by value, private), so by
// construction no other code can fire pyro / start control without going
// through the manager's guards (option C: composition, no friend needed).
class FlightSequenceExecutor {
public:
    FlightSequenceExecutor(
        const Clock& clock,
        logging::LogManager& logManager,
        ControlPD& controlPID,
        RotationAccumulator& rotationAccumulator,
        BarometricHeightCalculator& barometricHeightCalculator,
        VerticalMovementTracker& verticalMovementTracker,
        HorizontalMovementTracker& horizontalMovementTracker,
        hardware::IMotorIgniter& motorIgniter,
        hardware::IParachute& parachute
    );

    // Dumb hardware sequences. All return false on failure, never change FlightState.
    bool configureForFlight(const FlightProfile& profile);
    bool preflightChecks() const;

    bool launch();
    void finishBurn();
    bool deployForDescent();
    void finishLanding();
    void finishFlight();

    void abortThrust();
    bool deployParachute();

    // Phase-detection observers (read-only view of trackers for the manager).
    bool hasReachedApogee() const;
    bool hasTouchedDown(float initialHeight_m) const;

private:
    const Clock& _clock;
    logging::LogManager& _logManager;
    ControlPD& _controlPID;
    RotationAccumulator& _rotationAccumulator;
    BarometricHeightCalculator& _barometricHeightCalculator;
    VerticalMovementTracker& _verticalMovementTracker;
    HorizontalMovementTracker& _horizontalMovementTracker;
    hardware::IMotorIgniter& _motorIgniter;
    hardware::IParachute& _parachute;
};
