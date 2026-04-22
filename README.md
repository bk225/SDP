# SDP
Overview

This repository contains the electrical system design for an Unmanned Ground Vehicle (UGV) developed for urban firefighting support. The project focuses on designing a reliable, low-noise, and modular electronics architecture to replace a previous prototype that relied on breadboards, long jumper wires, and loosely connected components.

The system includes power distribution, motor control, sensing, and embedded control hardware, with an emphasis on robustness, scalability, and real-world deployment readiness.

Objectives:

Replace unreliable breadboard wiring with a custom PCB-based solution,
Provide stable power rails (24V, 12V, 5V, 3.3V),
Minimize electrical noise from motor drivers affecting logic systems,
Improve grounding and signal integrity,
Support sensor integration for fire detection and navigation.

System Architecture

The UGV electrical system is divided into several subsystems:

Power System: 
  24V input (drive power),
  Buck conversion to 12V, 5V, and 3.3V rails,
  Power distribution to all subsystems,
Control System:
  Microcontroller (STM32 or similar),
  PWM driver (PCA9685) for actuator control,
Actuation:
  BTS7960 motor drivers (external or onboard depending on final design),
  Linear actuator (12V) for steering,
Sensing & Compute:
  Raspberry Pi for high-level processing,
  Sensors: LiDAR, thermal camera, IR, IMU,
  Fire detection sensors (gas, optical, thermal).
