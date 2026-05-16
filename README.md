# WindSwitch

**Embedded controller for automatic wind protection of external roller shutters**

WindSwitch is a complete embedded hardware-and-firmware project designed to protect external roller shutters on residential buildings from strong wind conditions. The system monitors an anemometer pulse signal, evaluates wind intensity against adjustable user settings, and automatically commands shutters into a safe raised position when dangerous wind persists.

This repository contains the firmware, hardware production assets, project documentation, and assembled device imagery required to understand, reproduce, and deploy the system.

---

## Overview

External roller shutters can be damaged by sustained wind loads and repeated gust impacts. WindSwitch addresses this problem with a dedicated controller that continuously supervises wind activity and reacts automatically when the configured trigger condition is met.

The design is intended for practical field use, with simple on-device adjustment and a focused control strategy suited to real weather behavior.

---

## Assembled Device

![Assembled WindSwitch device](assets/images/device-assembled.jpg)

The image above shows the assembled WindSwitch controller as implemented in hardware.

---

## Core Function

WindSwitch receives pulse signals from an anemometer and uses pulse frequency as an indicator of wind intensity.

When wind remains above the configured threshold long enough to confirm a real event, the controller activates the output and commands the shutters to raise. After activation, the shutters remain in the raised position for the configured active period. If a new wind event above the threshold occurs during that interval, the timer is refreshed, providing retriggerable protection behavior.

This operating model helps prevent false triggering from short gusts while maintaining protection during unstable wind conditions.

---

## Main Features

- Automatic wind-triggered protection of external roller shutters
- Adjustable wind trigger threshold
- Adjustable timing parameter for activation / hold behavior
- Retriggerable active interval for repeated gust events
- Anemometer pulse-frequency measurement
- STM32-based embedded control platform
- Isolated sensor input interface
- Output control stage for shutter actuation
- LED indication for system state and active hold state
- Hardware design and fabrication files included
- Project documentation included

---

## System Architecture

The repository contents and firmware structure indicate the following system model:

- **Wind sensor input:** pulse-output anemometer
- **Signal evaluation:** timer input capture used to measure pulse frequency
- **User adjustment:** analog settings read through ADC inputs
- **Control logic:** state-based trigger and hold behavior
- **Output stage:** shutter control output for automatic raise command
- **Status interface:** dedicated LEDs for activity and hold indication

This architecture is optimized for a compact, single-purpose protection controller.

---

## Repository Structure

```text
WindSwitch/
├── README.md
├── assets/
│   └── images/
│       └── device-assembled.jpg
├── doc/
│   ├── WindSwitch.pdf
│   └── WindSwitch.pptx
├── fw/
│   └── DE-070924/
│       ├── Core/
│       ├── Drivers/
│       ├── MDK-ARM/
│       ├── .mxproject
│       └── DE-070924.ioc
└── hw/
    ├── DE-230724/
    ├── DE-230724_FabricationOutput.zip
    └── DE-230724_FabricationOutputTest.zip
```

---

## Firmware

The firmware is located in `fw/DE-070924/` and targets the **STM32F030F4P6** microcontroller.

### Firmware characteristics

- Implemented in **C**
- Based on **STM32 HAL**
- Configured through **STM32CubeMX** project files
- Uses **TIM3 input capture** for wind pulse measurement
- Uses **ADC channels** for reading threshold and timing settings
- Implements a simple trigger / hold state machine
- Drives separate status and hold LEDs
- Supports optional independent watchdog operation

### Relevant configured signals

- `PA0` — threshold setting input
- `PA1` — timing setting input
- `PA2` — output control
- `PA3` — status LED
- `PA4` — hold/activity LED
- `PB1` — wind sensor pulse input

---

## Hardware

The hardware section contains PCB-related project data and manufacturing outputs.

Available hardware assets include:

- board design directory under `hw/DE-230724/`
- fabrication output archive
- fabrication test output archive
- assembled device image under `assets/images/`

Together with the firmware and documentation folders, this repository represents a complete development package for the WindSwitch device.

---

## Documentation

The `doc/` directory contains supporting project material:

- `doc/WindSwitch.pdf`
- `doc/WindSwitch.pptx`

These files provide additional technical and presentation context for the project.

---

## Application Context

WindSwitch is intended for residential installations using external roller shutters exposed to outdoor wind conditions. It is especially suitable where shutters need automatic protection without requiring immediate user intervention.

The project is focused, practical, and directly tied to a real deployment scenario.

---

## Safety Notice

This project includes hardware intended for use with real electrical installations and shutter control systems.

- Installation should be performed by a qualified person.
- Proper isolation, enclosure design, and mains-safety practices are required.
- Compatibility with the selected anemometer and shutter interface should be verified before deployment.
- The device must not be serviced while energized.
