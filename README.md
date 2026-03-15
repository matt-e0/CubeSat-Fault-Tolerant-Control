# Fault-Tolerant CubeSat Attitude Control

**Author:** Matteo Venuti | University of Liverpool  
**Supervisor:** Dr Barry Smith | **Assessor:** Dr Simon Maher

---

A fault-tolerant attitude control system for 1U CubeSat compartments, using four reaction wheels in a pyramid configuration controlled by an Adaptive Sliding Mode Controller (ASMC). The system maintains full three-axis attitude control following single or multiple wheel failures with no manual retuning or fault detection logic required.

<p align="center">
  <a href="https://github.com/user-attachments/assets/f2ae930a-9342-4874-bb6c-7ed893be482d">
    <img src="https://github.com/matt-e0/CubeSat-Fault-Tolerant-Control/raw/main/Resources/1000039267.jpg" width="500"/>
  </a>
  <br/>
  <em>Click to watch bench demo</em>
</p>

## Overview

Reaction wheels generate torques by accelerating or decelerating flywheels, rotating the satellite through conservation of angular momentum. Four wheels are arranged in a pyramid configuration, providing a redundant fourth actuator. When a wheel fails, the adaptive gain in the controller increases automatically to compensate, distributing torque across the remaining wheels via a pseudoinverse allocation matrix.

<p align="center">
  <img src="Resources/1000038925.jpg" width="500"/>
  <br/>
  <em>Internal hardware - reaction wheels and electronics exposed</em>
</p>

## Key Results

- Attitude regulation within **5°** following sequential failure of two reaction wheels
- Adaptive gain responds automatically to fault - no controller reconfiguration needed
- Lyapunov-stable convergence demonstrated in simulation
- Hardware: bidirectional ESC firmware implemented from scratch on STM32 B-G431B

<video src="Resources/BenchDemo.mp4" width="320" height="240" controls></video>

<p align="center">
  <img src="Resources/CrowdFundingGif.gif" width="600"/>
  <br/>
  <em>Simulink simulation - attitude tracking with PD controller baseline</em>
</p>

## Repository Structure

| Folder | Contents |
|--------|----------|
| [`Simulation/`](Simulation/) | MATLAB ASMC simulation, Simulink models, plotting scripts |
| [`Controller/`](Controller/) | Teensy 4.1 ASMC implementation |
| [`Mechanical/`](Mechanical/) | CAD files, reaction wheel design |
| [`Electrical/`](Electrical/) | Circuit overview, component selection |
| [`Documents/`](Documents/) | Poster, reports, references |
| [`Initial Planning & Rough Work/`](Initial%20Planning%20%26%20Rough%20Work/) | Early designs, derivations, notes |

## Hardware

| Component | Part |
|-----------|------|
| Microcontroller | Teensy 4.1 |
| IMU | Bosch BNO055 (quaternion output, 100Hz) |
| Motor driver | STM32 B-G431B ESC1 × 4 |
| Motors | BLDC with magnetic encoder |
| Power | 3S LiPo (9.6–12.6V) |
| Structure | 3D printed PLA, aluminium flywheels |

## Controller Summary

The ASMC drives the sliding surface `s = e_ω + λ·e_q` to zero using a PD feedback term and an adaptive switching term. The switching gain `K` updates online - growing when the system deviates from the surface (e.g. after a fault) and decaying upon convergence. This eliminates the fixed-gain trade-off between disturbance rejection and actuator chattering.

```
s = e_omega + lambda * e_q
tau = J*(-Kp*e_q - Kd*e_omega) - J*K_adapt*sat(s/phi) - omega × (J*omega)
K_adapt_dot = Gamma*||s|| - sigma*(K_adapt - K_min)
```

See the 'Simulation' folder for full implementation and results.

## Acknowledgements

This project was completed as part of a BEng project at the University of Liverpool, School of Electrical Engineering. 

#### Use of AI
All design aspects and technical work of this project and validation were done by the author. AI (Claude, Anthropic) was used for the modification of STMicroelectronic Motor Control SDK to enable bidirectional control of the motor drivers. This decision was made to reduce the workload of the project, ensuring it was completed in time. Due to the license agreement of the MC SDK, modified firmware will not be shared in this project. All other components represent the authors own work.
