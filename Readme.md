# Wisp Mk1

![Wisp Mk1](recources/wisp.jpg)

Wisp Mk1 is a lightweight model rocket that uses thrust vector control (TVC).
Its system includes a custom flight computer, a USB-to-radio bridge, and a
desktop monitoring app. Together, they control the rocket and provide
telemetry and flight logs.

## Flight stabilization

During powered flight, the firmware keeps the rocket near its target direction
by angeling the rocket motor:

<img src="recources/tvc_concept.jpg" alt="Thrust-vector control concept" width="400">

<img src="recources/Gimbaled_thrust_animation.gif" alt="Gimbaled thrust animation" width="400">

The firmware uses gyroscope readings to estimate the rocket's attitude. A PD
controller compares this estimate with the target and uses the rotation rate
to adjust yaw and pitch. It measures roll but does not control it because the
rocket is axisymmetric. The gimbal redirects thrust to reduce unwanted
rotation and move the rocket back toward its target.

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


Splitting the project into separate parts makes them easier to develop
independently. This was the first design decision I made for the project.

## How the components communicate

The Rocket Monitor app communicates with the flight computer through
Rocket-Link. Rocket-Link connects to the computer by USB and sends data to the
flight computer over an HC-12 radio:

```text
+----------------------+   USB    +----------------------+  HC-12 radio  +----------------------+
| Rocket Monitor       |<-------->| Rocket-Link          |<------------->| Alacer-1 Flight      |
| Ground station app   |          | USB/radio bridge     |               | Computer + firmware  |
+----------------------+          +----------------------+               +----------------------+
```

## Building

The firmware is built with [PlatformIO](https://platformio.org/) and the
Arduino framework:

```bash
pio run -e custom_f411ce
```

The default target is a generic STM32F411CE board. Uploads use ST-Link:

```bash
pio run -e custom_f411ce -t upload
```

To build the on-device simulation configuration:

```bash
pio run -e custom_f411ce_sim
```

See [`tests/`](tests/) for host-side test runners and simulation documentation.

## Safety

Wisp Mk1 is a hobby model rocket project. An amateur developed its firmware
and hardware. Check them carefully before use.