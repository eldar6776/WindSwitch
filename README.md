# WindSwitch

**A dedicated embedded controller for automatic wind protection of external roller shutters.**

WindSwitch is a complete hardware-and-firmware project for protecting residential external roller shutters from strong wind conditions. The system monitors an anemometer pulse signal, evaluates wind intensity against a user-adjustable threshold, and automatically commands the shutters into a safe raised position when hazardous conditions persist.

The device is designed for practical deployment, not just demonstration. It combines a compact STM32-based control platform, isolated sensor interfacing, configurable activation behavior, and hardware design files required for manufacturing and integration.

---

## What WindSwitch Does

WindSwitch continuously measures wind activity using a pulse-output anemometer. When the measured wind level exceeds a configured threshold for long enough to confirm a real event, the controller activates the shutter-raise output.

After activation, the shutters remain raised for a configurable hold period. If a new wind event above the trigger threshold occurs during that active interval, the timer is restarted. This gives the system **retriggerable protection behavior**, which is especially useful in unstable weather with repeated gusts.

In short, WindSwitch is built to:

- protect external roller shutters from wind damage
- reduce false triggering caused by short gusts
- keep shutters in a safe state while dangerous wind persists
- provide simple field adjustment without firmware modification

---

## Main Features

- **Automatic wind-based shutter protection**
- **Adjustable wind trigger threshold**
- **Adjustable activation / hold timing**
- **Retriggerable timing logic** for repeated gust events
- **Anemometer pulse-frequency measurement**
- **STM32-based embedded control firmware**
- **Isolated sensor input stage**
- **Triac-based output control architecture**
- **Visual status indication through LEDs**
- **Hardware design and fabrication files included**
- **Supporting documentation included**

---

## System Behavior

The repository contents and firmware implementation indicate the following operating model:

1. An anemometer provides pulse output proportional to wind speed.
2. The controller measures pulse frequency using timer input capture.
3. A user-adjustable threshold determines the wind level that should trigger protection.
4. A user-adjustable timing parameter defines how long the condition must persist and/or how long the system remains active.
5. Once the wind condition is confirmed, the output is asserted and shutters are commanded to raise.
6. While the system is active, any new threshold crossing refreshes the active timer.

This behavior makes WindSwitch well-suited for real outdoor conditions where wind is irregular, intermittent, and often bursty rather than steady.

---

## Repository Contents

```text
WindSwitch/
├── README.md
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

## Firmware Overview

The firmware is located in `fw/DE-070924/` and targets the **STM32F030F4P6** microcontroller.

### Observed firmware characteristics

- Written in **C** using **STM32 HAL**
- Configured through **STM32CubeMX** project files
- Uses **TIM3 input capture** to measure incoming wind-sensor pulse frequency
- Uses **ADC channels** to read analog user settings
- Implements output logic as a simple state machine
- Drives dedicated status and hold/activity LEDs
- Includes optional **independent watchdog** support

### Key control signals from the project configuration

- `PA0` — threshold setting input
- `PA1` — delay / timing setting input
- `PA2` — output control
- `PA3` — status LED
- `PA4` — hold/activity LED
- `PB1` — timer capture input from wind sensor

This firmware structure strongly supports the intended use case of a small, reliable, single-purpose field controller.

---

## Hardware Overview

The hardware section contains PCB-related project outputs and manufacturing archives, indicating that the project includes real board development and fabrication preparation.

Available hardware assets include:

- hardware design directory under `hw/DE-230724/`
- fabrication output archive
- fabrication test output archive

Together with the firmware and documentation folders, this suggests that the repository represents a complete product-development package rather than only source code.

---

## Documentation Assets

The `doc/` directory contains project documentation material:

- `doc/WindSwitch.pdf`
- `doc/WindSwitch.pptx`

These files can be used to support installation, presentation, technical communication, or project handoff.

---

## Practical Value of the Project

WindSwitch solves a clear real-world problem: external roller shutters can be damaged by strong or repeated wind loads, especially when left unattended. By automating the protective response, the system improves both equipment safety and user convenience.

The strongest aspect of this project is its practical focus:

- dedicated purpose
- simple user adjustment model
- embedded control implementation
- deployable hardware assets
- clear integration path with wind sensing and shutter actuation

This makes WindSwitch a credible embedded control product for residential weather protection applications.

---

## Safety Notice

This project includes hardware intended for interfacing with real electrical installations and shutter control systems.

- Installation should be performed by a qualified person.
- Proper isolation, enclosure, and mains-safety practices are required.
- Compatibility with the selected anemometer and shutter system should be verified before deployment.
- The device should never be serviced while energized.

---

## Suggested Next Improvements for the Repository

If desired, the repository can be improved further with:

- product photos of the assembled device
- PCB renders or schematic snapshots
- wiring diagram examples
- a short build / flash guide for firmware
- a calibration guide for threshold and timing settings
- installation examples for single-home deployments

These additions would make the project even stronger for presentation, manufacturing handoff, and field adoption.
