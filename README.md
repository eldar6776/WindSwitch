# WindSwitch

**Operational wind protection controller for external roller shutters**

WindSwitch is a fully operational embedded device designed to protect external roller shutters on residential buildings from wind damage. It monitors an anemometer pulse signal, compares the measured wind intensity against an adjustable threshold, and automatically raises the shutters when unsafe wind conditions persist.

Once triggered, the system keeps the shutters raised for a configurable hold time. Any new wind event above the trigger threshold during that period extends the timer, making the protection logic **retriggerable**.

---

## Project Overview

This repository contains the complete WindSwitch project:

- **Firmware** for an STM32-based controller
- **Hardware design files** and fabrication outputs
- **Project documentation** and presentation material

WindSwitch is intended for real-world installation where automatic protection of roller shutters is required during strong wind conditions.

---

## Key Features

- **Automatic wind protection** for external roller shutters
- **Adjustable wind-speed trigger threshold** using a front-panel potentiometer
- **Adjustable confirmation time** before activation, preventing false triggers from short gusts
- **Retriggerable hold logic**: each new wind event above the threshold extends the active protection time
- **Automatic shutter raise output** when the trigger condition is met
- **Visual LED indication** for status and active hold state
- **Galvanically isolated wind sensor input** via optocoupler
- **Triac-based output stage** for shutter control interface
- **Watchdog-ready firmware architecture** for improved robustness

---

## Operating Principle

WindSwitch works as follows:

1. The device receives pulse signals from an anemometer.
2. The pulse frequency is used as an indicator of wind speed.
3. A potentiometer sets the trigger threshold.
4. If wind speed remains above the configured threshold continuously for the configured trigger period, the controller activates the shutter output.
5. After activation, the shutters remain raised for the configured hold time.
6. If another wind event above the threshold occurs during the hold period, the hold timer is restarted.

In practical terms, this means the device does not react to a brief gust immediately, but it does respond reliably to dangerous wind conditions and keeps the shutters protected as long as the risk continues.

---

## Behavior of the Device

Based on the repository firmware and your description, the intended application behavior is:

- Wind threshold is adjustable
- Activation occurs only after wind stays above the limit long enough to confirm a real event
- All shutters are raised automatically when the event is confirmed
- The shutters stay raised for **15 minutes**
- Every subsequent gust above the trigger threshold **extends the active timer**
- This makes the output logic **retriggerable**, which is ideal for unstable weather conditions

---

## Technical Summary

- **Application:** wind protection for residential external roller shutters
- **MCU:** `STM32F030F4P6`
- **Firmware language:** `C`
- **Framework:** `STM32 HAL`
- **Inputs:**
  - anemometer pulse input
  - threshold potentiometer
  - delay / hold-time potentiometer
- **Outputs:** shutter control output via triac stage
- **Indicators:** status LED and hold/activity LED
- **Timer input capture:** used for pulse-frequency measurement
- **ADC channels:** used for reading user-adjustable settings

---

## Repository Structure

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
│       └── DE-070924.ioc
└── hw/
    ├── DE-230724/
    ├── DE-230724_FabricationOutput.zip
    └── DE-230724_FabricationOutputTest.zip
```

---

## Firmware Notes

The firmware in `fw/DE-070924` is based on STM32Cube / HAL and targets the **STM32F030F4P6** microcontroller.

The implementation includes:

- timer input capture for measuring anemometer pulse frequency
- ADC sampling for user-set threshold and delay values
- output state machine for trigger / hold behavior
- LED signaling for device status and active protection state
- optional independent watchdog support

This confirms that the repository is not just a concept archive, but a complete embedded project with both hardware and firmware assets.

---

## Hardware Notes

The hardware part of the repository includes PCB design and fabrication outputs. From the provided project context, the device is built as a mains-powered controller intended for installation with shutter systems and a wind sensor.

**Important:** because the device interfaces with mains voltage and shutter control hardware, installation and commissioning should be performed only by a qualified electrician or technician.

---

## Installation Concept

Typical system connection flow:

1. Connect mains power to the WindSwitch unit.
2. Connect the anemometer pulse output to the isolated input stage.
3. Connect the shutter control interface to the output stage.
4. Set the desired wind threshold.
5. Set the desired confirmation / hold timing.
6. Test the trigger behavior before final deployment.

---

## Safety Notice

- This project includes hardware intended for use with **mains voltage**.
- Improper installation may cause equipment damage or electric shock.
- Always ensure correct isolation, enclosure, and field wiring.
- Verify anemometer operation and shutter interface compatibility before deployment.
- Do not work on the device while it is energized.

---

## Why This Project Stands Out

WindSwitch addresses a real and practical problem in residential automation: protecting exterior shutters from weather damage without requiring user intervention. The combination of adjustable thresholding, delayed validation, and retriggerable hold behavior makes the device robust and suitable for real outdoor conditions where wind can fluctuate rapidly.

This is a focused, practical, and deployment-oriented embedded product rather than only an experimental prototype.

---

## Documentation

Additional project material is available in:

- `doc/WindSwitch.pdf`
- `doc/WindSwitch.pptx`
- `hw/DE-230724/`
- `hw/DE-230724_FabricationOutput.zip`
- `hw/DE-230724_FabricationOutputTest.zip`

---

## Author

- [eldar6776](https://github.com/eldar6776)

---

## License

The repository currently contains STM32 HAL-based firmware and project assets. Unless a separate license file is added, treat the project as shared **as-is**.

If you want, the next step could be to also add:

- a cleaner **project badge/header section**
- a **product photo section**
- a **How it works** diagram
- a **Build & flash firmware** section
- a **Bosnian/Croatian/Serbian + English bilingual README**
