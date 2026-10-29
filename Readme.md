# Wisp Mk1

![Wisp Mk1](recources/wisp.jpg)

Wisp Mk1 is an extremely lightweight thrust-vector-controlled (TVC) model
rocket. The project combines a compact flight computer, a radio link, and a
desktop monitoring application to provide onboard control, telemetry, flight
logging, and ground-station operation.

This repository contains the **Alacer-1 flight computer firmware**. It runs on
an STM32F411-based controller and coordinates the rocket's sensors and
actuators during flight. The firmware includes:

- IMU and barometric altitude measurement
- Rotation and ascent tracking
- PD-based thrust-vector control
- Flight-state sequencing, including launch, burn, descent, and recovery
- Gimbal, motor ignition, parachute, pyro, buzzer, LED, battery, and radio
  hardware interfaces
- Telemetry, event logging, flash storage, and log download
- A simulation mode with mock hardware and a simple rocket model
- A request/response radio protocol used by the Rocket-Link system

## Wisp project

Wisp is split into several cooperating hardware and software components:

| Component | Description |
| --- | --- |
| [Mechanical design](https://cad.onshape.com/documents/95bcdae310df42bf9c944b22/w/453d5222de5aa817fef552d9/e/846b29e1962f732657283316) | CAD model of the Wisp Mk1 airframe and mechanisms |
| [Flight Computer](https://github.com/tomatencode/Alacer-1) | Hardware design for the Alacer-1 flight computer |
| [Flight Computer Firmware](https://github.com/tomatencode/Alacer-1-Firmwear) | Embedded firmware in this repository |
| [Rocket-Link](https://github.com/tomatencode/RocketLink-1) | Hardware design for the Rocket-Link ground/radio device |
| [Rocket-Link Firmware](https://github.com/tomatencode/RocketLinkFirmwear) | Firmware for the Rocket-Link device |
| [Rocket Monitor App](https://github.com/tomatencode/RocketMonitor) | Ground-station application for monitoring and operating the rocket |

## Building

The firmware is built with [PlatformIO](https://platformio.org/) and the
Arduino framework:

```bash
pio run -e custom_f411ce
```

The default hardware target is a generic STM32F411CE board and uploads use
ST-Link:

```bash
pio run -e custom_f411ce -t upload
```

To build the on-device simulation configuration:

```bash
pio run -e custom_f411ce_sim
```

Additional host-side test runners and simulation documentation are available
in [`tests/`](tests/).

## Status and safety

Wisp Mk1 is an experimental hobby model-rocket project. The firmware and
hardware were developed by an amateur and should not be relied on without
careful, independent verification before use.