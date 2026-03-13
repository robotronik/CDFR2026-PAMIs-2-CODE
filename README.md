# 🤖 CDFR 2026 - PAMI Firmware

![Status](https://img.shields.io/badge/status-in--development-orange?style=for-the-badge)
![Hardware](https://img.shields.io/badge/hardware-ESP32-blue?style=for-the-badge&logo=espressif)

This repository contains the firmware for Robotronik's 2026 PAMIs (Small Intelligent Mobile Actuator) designed for the French Robotics Cup.

---

## 🚀 Core Features

### 🧭 Advanced Navigation
* **High-Frequency Odometry:** Real-time position tracking using magnetic Hall effect encoders.
* **Closed-Loop Control:** Dual PID controllers for precise velocity and heading management.
* **Smooth Trajectories:** Integrated support for linear movements and "LookAt" orientation commands.

### 🛡️ Perception & Safety
* **Obstacle Avoidance:** Real-time distance monitoring via ultrasonic sensors to prevent collisions on the field.
* **Power Management:** Active battery voltage monitoring (ADC) to protect LiPo cells from deep discharge.
* **Safety Stops:** Automatic emergency stop logic triggered by sensors or match timers.

### 🎯 Match Strategy
* **Automated Sequence:** Manages the full match lifecycle from pull-cord detection to final stop.
* **Action Handling:** Precise control over servo-driven mechanisms for game element manipulation.
* **User Interface:** RGB LED feedback for team color identification and system diagnostics.

---

## 🔌 Hardware Specifications

| Component | ESP32 Pin | Function |
| :--- | :--- | :--- |
| **DC Motors** | 17, 18, 19, 21 | PWM Driver Control |
| **Encoders** | 32, 33, 36, 39 | Hall Effect (A/B Channels) |
| **Servos** | 26, 27 | Mechanism Actuation |
| **Start Cord** | 23 | Pull-cord Interrupt (Tirette) |
| **Ultrasonics** | 35 (TX), 16 (RX) | Range Sensing |
| **Battery Sense** | 34 | Voltage Monitoring (Analog) |

---

## 🛠 Setup & Installation

This project uses the standard **ESP-IDF** toolchain. Ensure you have the environment exported before running the build scripts.

1.  **Clone the repository:**
    ```bash
    git clone git@github.com:robotronik/CDFR2026-PAMIs-2-CODE.git
    cd CDFR2026-PAMIs-2-CODE
    ```
2.  **Install ESP-IDF:** Ensure `idf.py` is installed on your machine.
3.  **Configure:** (Optional) Run `idf.py menuconfig` to adjust project-specific settings.

---

## 🛠 Setup & Installation

1.  **Environment:** Open the project in **VS Code** with the **PlatformIO** extension.
2.  **Configuration:** Adjust physical constants (wheel diameter, PID gains) in the configuration headers.
3.  **Upload:** Connect the ESP32 and use the `Upload` task to flash the firmware.
4.  **Monitor:** Use the Serial Monitor at `115200` baud for real-time telemetry and debugging.
