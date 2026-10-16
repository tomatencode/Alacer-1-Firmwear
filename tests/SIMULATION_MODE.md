# On-device simulation build

From `/home/simon/src/Alacer-1-Firmwear`, build with:

```sh
pio run -e custom_f411ce_sim
```

Upload using `pio run -e custom_f411ce_sim -t upload`. The ordinary
`custom_f411ce` environment remains the default and uses real flight hardware.

## Flags

- Defining `ALACER_SIMULATION` selects simulation at compile time. Omit the
  definition entirely for real hardware (even `=0` counts as defined).

`BuildConfig.hpp` translates the flag to a constexpr setting and `FlightHardware.hpp`
selects the hardware bundle with `std::conditional_t`. Only the selected bundle
is instantiated; there is no runtime mode switch or heap allocation for wiring.

## Hardware and safety

Simulation substitutes the IMU, barometer, gimbal, pyro channels, arming manager,
motor igniter, parachute, and flight clock. It does not initialize physical
flight sensors, servo PWM, pyro MOSFETs, or the hardware-arm input. PD control
and gimbal radio handlers use the common `IGimbal` interface.

Radio, flash storage/logging, battery measurement, status LED and buzzer remain
real. Their timing uses real time, independently of simulated flight time. This mode still
needs an MCU and those peripherals; it is not a desktop firmware build.

Simulated continuity and hardware arming start enabled. Software arming starts
disabled and must be enabled through the normal radio command. Countdown requests
select simulated motor/parachute channels through the existing channel indices.
Ignition and deployment honor simulated continuity and arming, and simulated
pulse durations use flight milliseconds. Direct FIRE_PYRO requests exercise the
simulated channel only; rocket ignition/deployment occurs via the flight sequence.

## Simulation timing and model

The firmware targets a 1:1 flight-time/real-time ratio; there is no build flag for
changing simulation speed. Each firmware loop consumes at most 1 ms of elapsed
real time before running the sensors, estimators, flight state machine and
controller. Pending time is retained, including across real-time counter rollover.
If the MCU cannot keep up with real time, simulation falls behind rather than
taking an unstable large physics step.

The existing simplified model uses a 3 N, 5 s motor burn measured from ignition,
quadratic drag, parachute drag and a ground-plane constraint. This is a functional
integration model, not a validated flight dynamics or hardware-safety simulator.
Reboot to start a fresh simulation; ending a flight does not rebuild rocket state.

## Validation

```sh
bash /home/simon/src/Alacer-1-Firmwear/tests/run-simulation-mode-tests.sh
bash /home/simon/src/Alacer-1-Firmwear/tests/run-sim-rocket-tests.sh
bash /home/simon/src/Alacer-1-Firmwear/tests/run-clock-tests.sh
bash /home/simon/src/Alacer-1-Firmwear/tests/run-hardware-interface-tests.sh
```

The simulation-mode host suite checks 1:1 timing, bounded steps, pending-time
draining, timer rollover,
virtual gimbal dispatch, arming and continuity, millisecond pulse timing, delayed
ignition, sensor feedback, parachute deployment and eventual touchdown.

## VS Code host-test diagnostics

The simulation-mode test runner requires `g++` and Python 3. It also generates
`/home/simon/src/Alacer-1-Firmwear/.pio/host-tests/compile_commands.json` for the
workspace's C/C++ IntelliSense settings. Run the test script once after cloning
or removing `.pio` to generate it.

This gives `SimulationModeTest.cpp` the host Arduino stub, native compiler and
`ALACER_SIMULATION` definition instead of the default STM32 firmware configuration.
Firmware source files continue to use PlatformIO's configuration. If stale errors
remain, run **C/C++: Reset IntelliSense Database** from the VS Code command palette.