# Low-Cost Precision Guidance and Smart Electronic Fuze System for a 155 mm Artillery Shell

> A low-cost embedded sensing, telemetry, and actuation prototype for developing the electronics, environmental monitoring, navigation, and multi-mode event-detection subsystems of a precision guidance and smart electronic fuze architecture.

## Overview

The **Low-Cost Precision Guidance and Smart Electronic Fuze System** is a prototype development platform focused on integrating embedded sensing, navigation, telemetry, actuation, and a multi-mode electronic fuze architecture.

The system is designed around a modular embedded platform that combines inertial sensing, GNSS positioning, atmospheric monitoring, power telemetry, proximity detection, impact/event sensing, and **BLDC-based actuation**.

The current V1 prototype uses an **ESP32** as the primary embedded controller and integrates:

- **MPU6050** — 6-axis inertial measurement
- **NEO-6M** — GNSS positioning
- **BMP280** — Barometric pressure and temperature
- **INA219** — Voltage and current monitoring
- **IR Proximity Sensor** — Proximity detection
- **SW-240** — Impact/vibration event detection
- **BLDC Motor** — Actuation mechanism for the prototype guidance subsystem

The system supports a **Multi-Mode Smart Fuze architecture** consisting of:

- **Proximity Mode**
- **Time Mode**
- **Impact Mode**

The V1 platform is intended primarily for **laboratory testing, sensor integration, firmware development, simulation, telemetry, and proof-of-concept validation**.

---

## Project Objectives

The project aims to develop a compact and modular embedded platform capable of supporting:

1. Inertial sensing
2. GNSS positioning
3. Environmental sensing
4. Power monitoring
5. Proximity detection
6. Impact/event detection
7. BLDC-based actuation
8. Multi-mode fuze state management
9. Sensor fusion
10. Real-time telemetry

---

## Prototype Hardware

| Component | Function | Interface |
|---|---|---|
| **ESP32** | Main microcontroller | — |
| **MPU6050** | 6-axis inertial measurement | I2C |
| **NEO-6M** | GNSS/GPS positioning | UART |
| **BMP280** | Barometric pressure and temperature | I2C |
| **INA219** | Voltage and current monitoring | I2C |
| **IR Sensor** | Proximity detection | GPIO |
| **SW-240** | Impact/vibration detection | GPIO |
| **BLDC Motor** | Prototype actuation | Motor Driver / PWM |

---

## System Architecture

```text
                         +-------------------------+
                         |          ESP32          |
                         |    Main Controller      |
                         +------------+------------+
                                      |
              +-----------------------+-----------------------+
              |                       |                       |
              v                       v                       v
        +-----------+           +-----------+           +-----------+
        |    I2C    |           |   UART    |           |   GPIO    |
        |    Bus    |           | Interface |           |  Inputs   |
        +-----+-----+           +-----+-----+           +-----+-----+
              |                       |                       |
       +------+------+------+          |                 +----+----+
       |      |      |      |          |                 |         |
       v      v      v      |          v                 v         v
    MPU6050 BMP280 INA219  |       NEO-6M           IR Sensor   SW-240
       |      |      |      |          |             Proximity   Impact
       +------+------+------+          |                 |         |
              |                         |                 |         |
              v                         v                 |         |
       Motion / Environment       GNSS Position           |         |
              |                         |                 |         |
              +-------------+-----------+-----------------+---------+
                            |
                            v
                  +----------------------+
                  |   Sensor Processing  |
                  |   & State Estimation |
                  +----------+-----------+
                             |
                +------------+------------+
                |                         |
                v                         v
        +---------------+         +---------------+
        | Guidance &    |         | Smart Fuze    |
        | Navigation    |         | Manager       |
        +-------+-------+         +-------+-------+
                |                         |
                v                         |
        +---------------+                 |
        | Actuation     |                 |
        | Controller    |                 |
        +-------+-------+                 |
                |                         |
                v                         |
        +---------------+                 |
        | BLDC Motor &  |                 |
        | Motor Driver  |                 |
        +---------------+                 |
                                          |
                              +-----------+-----------+
                              |           |           |
                              v           v           v
                         +---------+ +---------+ +---------+
                         |Proximity| |  Time   | | Impact  |
                         |  Mode   | |  Mode   | |  Mode   |
                         +---------+ +---------+ +---------+
                              |           |           |
                              +-----------+-----------+
                                          |
                                          v
                                +----------------------+
                                | Telemetry & System   |
                                | Status Monitoring    |
                                +----------------------+
