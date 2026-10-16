# Clock timing

Run `bash /home/simon/src/Alacer-1-Firmwear/tests/run-clock-tests.sh` after
installing the PlatformIO dependencies. The host tests use AddressSanitizer
and UndefinedBehaviorSanitizer.

Timing consumers receive a clock in their constructor. Keep that clock alive
for the lifetime of the consumer. The firmware entry point creates one shared
`Clock`; simulation callers can pass a `SimClock` through the same interfaces.
The clock does not select simulated hardware or automatically advance simulation
time: the simulation driver remains responsible for advancing `SimClock`.

## Time domains

- Real time (`getTime_ms()` / `getTime_us()`): HC12 AT commands, download session
  expiry, flash busy waits, physical servo travel, buzzer/LED output and physical
  pyro firing duration. These must not accelerate or stop with a simulation.
- Flight time (`getTimeFlight_ms()` / `getTimeFlight_us()`): rotation accumulation,
  vertical/horizontal tracking, PD control timing, flight countdown and motor-burn
  sequencing, log event timestamps and telemetry sampling. Existing simulated
  physics also uses flight time.
- Hardware delays and timing inside third-party Arduino drivers are unchanged:
  they still represent physical device requirements.

## Regression coverage

- Default flight time matches real time for a base `Clock`.
- Rotation integrates from flight time zero, pauses when only real time advances,
  and handles microsecond rollover.
- Physical pyro pulses expire in real time, even when passed a `SimClock`, and
  handle millisecond rollover.
- Stored event deltas follow flight time rather than elapsed real time.
- Download sessions survive advances in flight time but expire in real time.
- A source scan rejects direct `millis()` / `micros()` calls outside `Clock.hpp`.