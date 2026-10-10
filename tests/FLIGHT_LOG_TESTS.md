# Flight logging tests

Run the host tests (using the project's installed ETL and Eigen dependencies):

```sh
bash /home/simon/src/Alacer-1-Firmwear/tests/run-flight-log-tests.sh
```

The script compiles with warnings-as-errors and address/undefined-behavior
sanitizers. It tests binary layouts, stream duplicate suppression, cache resets,
independent sensor/channel caches, storage-failure retries, 50 Hz recorder timing,
clock rollover, back-to-back log sessions, pyro refusals/stops/arming, sensor health
and recovery, PID clamping, flight phase detection, and radio errors.

Hardware SPI, ADC, GPIO and servo behavior is simulated; this does not replace
bench testing with pyro outputs disconnected.

## Recorder behavior

`TelemetryRecorder::update()` runs after sensor, estimator and PID updates.
It records the latest IMU, barometer, battery, PID, velocity, altitude, position,
gimbal and rotation values at up to 50 Hz while a log is open. Raw IMU data is
therefore sampled at 50 Hz, not at the IMU's configured 400 Hz rate.

`LastEventsTracker` suppresses identical encoded samples (including changes
smaller than the fixed-point resolution). Cache entries update only after a
successful write and reset for every new log. Pyro continuity is tracked per
channel and health per sensor. Discrete actions, state transitions and flight
detections are not suppressed. Main-loop timing records are also not suppressed.

Sensor health remains in the drivers. Stale means no accepted read for at least
100 ms, not a value that remains constant. SPI provides no transfer error status:
the IMU uses identity checks and an all-0xFF burst check; the barometer uses library
read errors and measurement validity. These checks cannot detect every bus fault
or a sensor that keeps returning plausible but frozen data. Telemetry getters
retain the last accepted measurements after read failures; interpret these
alongside sensor-health records.

Radio transmit failures count failed local `send()` attempts; the HC12 interface
cannot confirm remote delivery. Diagnostic counters and health states are logged
only when their encoded values change.

New record IDs and layouts are documented in
`/home/simon/src/Alacer-1-Firmwear/src/logManagement/LogProtocol.hpp`.
External decoders must support IDs 0x06–0x16. Quaternion and PID radian fields use
the existing fixed-point resolution of 0.01; no format migration was made.