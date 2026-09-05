# Low Cost Precision Guidance and Smart Electronic Fuze System for a 155 mm Artillery Shell

A low cost Precision Guidance System Kit prototype designed as a modular electronics package that can be integrated with an existing 155 mm artillery shell form factor. The system combines inertial sensing, GNSS positioning, environmental monitoring, BLDC based actuation and multi mode event detection to demonstrate the architecture of a retrofit guidance and electronic fuze platform.

## Demo Video

https://github.com/user-attachments/assets/ea4bcfed-2732-404f-be3d-91d5685b1a28

## Overview

The **Low Cost Precision Guidance and Smart Electronic Fuze System** is focused on integrating embedded sensing, navigation, actuation and a multi-mode electronic fuze architecture.

The system is designed around a modular embedded platform that combines inertial sensing, GNSS positioning, atmospheric monitoring, power telemetry, proximity detection, impact/event sensing, and **BLDC based actuation**.

The current V1 prototype uses an **ESP32** as the primary embedded controller and integrates:

- **MPU6050**:  6 axis inertial measurement
- **NEO-6M**:  GNSS positioning
- **BMP280**:  Barometric pressure and temperature
- **INA219**:  Voltage and current monitoring
- **IR Proximity Sensor**:  Proximity detection
- **SW-240**:  Impact/vibration event detection
- **BLDC Motor**:  Actuation mechanism for the prototype guidance subsystem

The system supports a **Multi-Mode Smart Fuze architecture** consisting of:

- **Proximity Mode**
- **Time Mode**
- **Impact Mode**

The V1 platform is intended primarily for **Testing, sensor integration, firmware development, simulation, telemetry, and proof of concept validation**.

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
8. Multi mode fuze state management
9. Sensor fusion
10. Real time telemetry

---

## Prototype Hardware

| Component | Function | Interface |
|---|---|---|
| **ESP32** | Main microcontroller | — |
| **MPU6050** | 6 axis inertial measurement | I2C |
| **NEO-6M** | GNSS/GPS positioning | UART |
| **BMP280** | Barometric pressure and temperature | I2C |
| **INA219** | Voltage and current monitoring | I2C |
| **IR Sensor** | Proximity detection | GPIO |
| **SW-240** | Impact/vibration detection | GPIO |
| **BLDC Motor** | Prototype actuation | Motor Driver / PWM |

---


## Simulation Stack                                                                                                                                                                      
                                                                                                                                                                                            
   The simulation is a three-layer chain, from pure physics to the real embedded                                                                                                            
   controller. Each layer validates the one above it, and every equation is                                                                                                                 
   traceable to published exterior-ballistics literature (McCoy *Modern Exterior                                                                                                            
   Ballistics*, Raza & Wang 2022, Cheng 2019, Chusilp 2012, Glebocki 2022).                                                                                                                 
                                                                                                                                                                                            
   ```text                                                                                                                                                                                  
   +---------------------------------------------------------------+                                                                                                                        
   |  LAYER 1 — 3-DOF FLIGHT PHYSICS (truth model)                 |                                                                                                                        
   |  point-mass trajectory: ISA atmosphere, Mach-dependent Cd,    |                                                                                                                        
   |  altitude wind profile, spin decay, Magnus, yaw of repose     |                                                                                                                        
   +------------------------------+--------------------------------+                                                                                                                        
                                  |                                                                                                                                                         
           +----------------------+----------------------+                                                                                                                                  
           v                                             v                                                                                                                                  
   +---------------------------+   +--------------------------------------+                                                                                                                 
   | LAYER 2 — HARDWARE-IN-    |   | LAYER 3 — ONBOARD-GNC SIL            |                                                                                                                 
   | THE-LOOP (HIL)            |   | (guidance_ordnance.m)                |                                                                                                                 
   | (guided_sensor.m)         |   | firmware_true.ino's exact math       |                                                                                                                 
   | PC flies the physics,     |   | (estimator + impact predictor +      |                                                                                                                 
   | REAL ESP32 streams real   |   | guidance law) flies the round using  |                                                                                                                 
   | sensor telemetry over     |   | ONLY noisy simulated baro/GPS at     |                                                                                                                 
   | UART; PC streams back     |   | board rates. Validates the onboard   |                                                                                                                 
   | canard commands in        |   | algorithm BEFORE flashing.           |                                                                                                                 
   | real time (1x - 4x).      |   |                                      |                                                                                                                 
   +---------------------------+   +--------------------------------------+                                                                                                                 
 ```

The GNC Math (implemented in firmware + validated in sim)                                                                                                                                  
                                                                                                                                                                                            
 - Firing solution (Chusilp 2012): bisection on gun elevation, secant                                                                                                                       
   iteration on azimuth for wind-drift cancellation.                                                                                                                                        
 - State estimation on board: baro altitude via hypsometric equation;                                                                                                                       
   vertical speed from a first-order complementary filter                                                                                                                                   
   vz ← 0.8·vz + 0.2·(Δh/Δt) at 10 Hz; GPS velocity smoothing at 1 Hz.                                                                                                                      
 - Impact Point Prediction (IPP) (McCoy; Raza & Wang 2022 use MPLT — we use                                                                                                                 
   the analytic point-mass equivalent an ESP32 can run in µs):                                                                                                                              
   closed-form time-to-impact τ = (vz + √(vz² + 2gz))/g, then drag-decay                                                                                                                    
   ballistic coast x_imp = x + vx·(1 − e^(−kd·τ))/kd.                                                                                                                                       
 - Guidance law: saturated proportional correction                                                                                                                                          
   δ = sat(Kp·(x_tgt − x_imp), ±15°), active post-apogee, 1 Hz.                                                                                                                             
 - Single-channel roll-orientation steering (Cheng 2019, Raza & Wang 2022):                                                                                                                 
   fixed-cant canards on the fuze cone; force orientation is the control                                                                                                                    
   variable, matching the dual-spin PGK architecture (M1156, Burke & Pergolizzi 2008).

   

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
