# 🤖 CDFR 2026 - PAMI Firmware

![Status](https://img.shields.io/badge/status-finished-green?style=for-the-badge)
![Language](https://img.shields.io/badge/language-C%2B%2B-blue?style=for-the-badge)
![Hardware](https://img.shields.io/badge/hardware-ESP32-blue?style=for-the-badge&logo=espressif)

This repository contains the firmware for Robotronik's 2026 PAMIs (Small Intelligent Mobile Actuator) for the French Robotics Cup (CDFR). The project runs on an ESP32 and controls motion, sensing, [...]

---

## Overview

The firmware is structured around a main finite-state machine in `main/main.cpp` and a modular set of components under `main/src` and `main/include`:

- locomotion and feedback control for the drive system
- servo and actuator control for match mechanisms
- ultrasonic sensing for obstacle detection
- battery voltage monitoring for safety
- team color and status LEDs
- optional Wi-Fi UDP remote logging

The robot behavior is driven by a simple state machine:

- `INIT`: initialize peripherals and safety checks
- `IDLE`: wait for team selection and pull-cord start
- `ACTIVE`: execute the strategy and action sequence
- `DONE`: stop after completion
- `ERROR`: restart the system if a critical fault occurs

---

## Hardware

The board uses an ESP32 and a set of on-board peripherals wired according to `main/include/pins.h`.

| Component | ESP32 Pin | Purpose |
| :--- | :--- | :--- |
| DC motors | 18, 17, 19, 21 | Drive motor PWM control |
| Hall encoders | 33, 32, 36, 39 | Odometry / speed feedback |
| Servos | 26, 27 | Mechanism actuation |
| Pull cord switch | 23 | Start signal |
| Team switch | 25 | Team selection |
| Ultrasonic trigger | 16 | Range sensor trigger |
| Ultrasonic echo | 35 | Range sensor feedback |
| Battery sense | 34 | Voltage monitoring |
| Team RGB LED | 22 | Team status indicator |
| Status LED | 4 | System state indication |

---

## Project Structure

```text
.
├── CMakeLists.txt              # ESP-IDF project entry point
├── build.sh                   # Convenience build / flash script
├── sdkconfig.defaults         # Default ESP-IDF configuration values
├── main/
│   ├── CMakeLists.txt         # App component registration
│   ├── Kconfig.projbuild       # Remote logging wifi settings
│   ├── idf_component.yml      # Component dependencies
│   ├── main.cpp               # Main FSM and app entry
│   ├── include/               # Public headers
│   └── src/                  # Firmware modules
│       ├── action.cpp
│       ├── battery_monitor.cpp
│       ├── actuators/
│       ├── locomotion/
│       ├── navigation/
│       └── wireless/
└── README.md
```

---

## Features

### Navigation and control

- closed-loop motion control with encoder feedback
- motor control abstraction for left/right drive channels
- support for smooth linear movement and orientation commands

### Perception and safety

- ultrasonic range detection for obstacle avoidance
- battery voltage monitoring to avoid deep discharge
- emergency logic for critical failures and unsafe conditions

### Match behavior

- start from pull-cord trigger
- team selection via hardware switch
- configurable LED feedback for state and team
- autonomous action sequence execution

### Remote logging

The project can enable Wi-Fi UDP remote logging through the ESP-IDF configuration.

---

## Development Setup

This project uses the standard ESP-IDF toolchain.

1. Install ESP-IDF and ensure `idf.py` is available.
2. Clone the repository:

```bash
git clone git@github.com:robotronik/CDFR2026-PAMIs-2-CODE.git
cd CDFR2026-PAMIs-2-CODE
```

3. Source the ESP-IDF environment:

```bash
source /path/to/esp-idf/export.sh
```

4. Build the project:

```bash
./build.sh build
```

5. Flash the firmware to the ESP32:

```bash
./build.sh deploy
```

6. Optional: reset flash memory:

```bash
./build.sh reset
```

---

## Configuration

The default project configuration is stored in `sdkconfig.defaults` and includes Wi-Fi settings used by the remote logging stack:

```ini
CONFIG_WIFI_SSID="Wifitronik"
CONFIG_WIFI_PASSWORD="36164626"
CONFIG_UDP_LOG_PORT=1234
CONFIG_HTTPD_WS_SUPPORT=y
```

The project also exposes runtime configuration options in `main/Kconfig.projbuild` for enabling/disabling remote logging and setting the Wi-Fi SSID, password, and UDP port.

---

## Build Script

The repository includes `build.sh`, which wraps common ESP-IDF commands:

```bash
./build.sh build
./build.sh deploy
./build.sh clean
./build.sh reset
./build.sh setup_lsp
```

This helps simplify local build, upload, and indexing workflows for editors such as VS Code and CLion.

---

## Notes

- `N_PAMI` is used to disable parts of the logic for test builds or ninja-mode configurations.
- `DEBUG_LEVEL` controls the amount of logging emitted at runtime.
- The current project is focused on the 2026 CDFR robot firmware and is still under active development.

---

## License and Contribution

This project is intended for Robotronik internal development and competition use. Contributions should follow the team workflow and hardware constraints for the robot.
