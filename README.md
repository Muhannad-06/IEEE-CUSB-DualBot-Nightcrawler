# 🏆 Nightcrawler: ESP32 Line Follower & Billiard Robot

An ESP32-based autonomous and manual multi-moded competition robot. 

**Competition Results:**
* 🥇 **1st Place Overall (Scored an unprecedented 126 points out of 125!)**
* 🥇 **1st Place in LineFollowing Phase**
* 🥇 **1st Place in Billiard Phase**

## 📖 Overview
This repository contains the software architecture for a competition-winning robotic vehicle developed in August 2026. The system seamlessly switches between an autonomous PID-based line-following mode and a manual, Bluetooth-controlled billiard phase. 

Designed with modular bare-metal principles in C++, the robot leverages a state-machine architecture to handle checkpoint dwells, dynamic steering kinematics, and a servo-driven impulse mechanism.

## ✨ Key Features
* **Dual-Mode Operation:** Easily switch between `AUTONOMOUS` and `MANUAL` modes using a paired PS5 Controller (edge-triggered via the Triangle button).
* **PID Line Following:** Utilizes a custom PID control loop to read a 3-channel IR sensor array, applying smooth differential drive output to stay on track.
* **Checkpoint State Machine:** Intelligently detects black strap checkpoints, executing a precise 5-second dwell and a 200ms clearance creep before resuming standard PID behavior.
* **Kinematic Inversion (Manual Mode):** During the billiard phase, the physical rear of the robot logically becomes the "front", optimizing handling for the impulse mechanism.
* **Billiard Cue Mechanism:** A 360-degree servo-driven impulse system to strike objects, activated seamlessly via the PS5 Cross button.
* **Deadzone & Anti-Windup Handling:** Software-level joystick deadzones prevent resting drift, while integral clamping prevents violent spinning when the robot loses the line.

## ⚙️ Hardware Components
* **Microcontroller:** ESP32 Development Board
* **Motor Driver:** L298N (controlling a 4-wheel drive setup)
* **Sensors:** 3x IR Line Tracking Modules (Left, Middle, Right)
* **Actuators:** Servo Motor (Cue Mechanism), DC Motors
* **Input:** PS5 Wireless Controller
* **Indicators:** Status LED, Laser Module

## 📂 Software Architecture & Technical Details
The codebase is strictly modularized for maintainability and debugging:

* **Core Environment:** Developed and tested specifically on **ESP32 Board Package version 2.0.14** to guarantee stable Bluetooth communication with the `ps5Controller` library.
* **`Config.h`**: The central configuration file. This file holds essential system constants including the PS5 MAC address, motor pin layouts, PWM settings, and the core PID tuning values (`Kp`, `Ki`, `Kd`, `BASE_SPEED`).
* **`Motors.h` & `Motors.cpp`**: Hardware abstraction for the L298N motor driver, handling direction polarity and PWM duty cycles.
* **`LineFollower.h` & `LineFollower.cpp`**: Houses the autonomous logic, IR sensor initialization, PID calculations, and checkpoint timer management.
* **`CueMechanism.h` & `CueMechanism.cpp`**: Dedicated logic for the servo-driven billiard striker, managing pulse widths and spin timers.
* **`main.ino`**: The core loop that polls the controller, manages the mode state machine, and dispatches commands to the respective subsystems.

## 🚀 Getting Started
1. **Configure Hardware:** Ensure your ESP32 is wired correctly according to the pin definitions in `Config.h`.
2. **Environment Setup:** Set your ESP32 board package to **v2.0.14** in your Arduino IDE or VS Code environment for full compatibility. 
3. **Set PS5 MAC:** Update the `PS5_MAC` constant in `Config.h` to match your controller's Bluetooth MAC address.
4. **Library Dependencies:** Install the required `ESP32Servo` and `ps5Controller` libraries.
5. **Build & Upload:** Flash the firmware to your ESP32 and power up the system.

## 👨‍💻 Developer
**Muhannad Mahfouz El-Shahiedy**
*Undergraduate Student, Computer and Systems Engineering*
*Zagazig University, Faculty of Engineering*