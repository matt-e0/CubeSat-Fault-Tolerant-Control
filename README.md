# CubeSat-Fault-Tolerant-Control

This is a repository for my final year project focused on the design of a fault-tolerant attitude control system for CubeSats. Specifically, it aims to deliver an over-actuated reaction wheel system, allowing for fault-tolerant control of a small CubeSat. It will achieve this through the use of reaction wheels. These are motor driven flywheels that generate angular momentum in order to rotate an object through conservation of momentum. The project should be able to demonstrate attitude control along a commanded axis in the event of failure in one reaction wheel or degradation of performance.
<p align="center">
  <img src="Resources/1000039267.jpg" alt="Complete hardware assembly" width="60%"/>
</p>

The project was developed in four major stages: Specifications and Reaction Wheel Design, Control Theory and Simulation, Hardware Prototype Fabrication and Performance Evaluation. The attitude control system uses four reaction wheels mounted in a pyramid configuration to achieve redundant control. Each reaction wheel is driven with a specialised ESC that uses encoder feedback and FOC for closed and/or open loop control. At the heart of the system is a Teensy 4.1 microcontroller that runs the Adaptive Sliding Mode Controller, interfacing with a BNO-055 IMU for orientation data and micro SD card to log experimental data. The picture below shows the hardware prototype open, with the reaction wheels and electronics exposed.

<p align="center">
  <img src="Resources/1000038925.jpg" alt="Open view of hardware demo" width="40%"/>
</p>
<p align="center">
  <img src="Resources/CrowdFundingGif.gif" alt="Simulink demo with PD controller" />
</p>
