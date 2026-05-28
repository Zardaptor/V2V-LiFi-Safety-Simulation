# V2V-LiFi-Safety-Simulation
A MATLAB/Simulink project that simulates vehicle-to-vehicle safety communication for collision avoidance. The system models two vehicles, obstacle detection, alert transmission delay, and braking response. It also includes a photovoltaic digital twin and LiFi/hardware documentation.

Project Overview
The simulation represents a safety scenario where Vehicle A detects an obstacle and sends an alert to Vehicle B. Vehicle B receives the alert after a communication delay and applies braking to reduce collision risk.

The project includes:

Vehicle motion modeling
Obstacle distance calculation
Alert threshold logic
V2V communication delay
Automatic braking response
Distance and velocity monitoring
PV digital twin model
LiFi/hardware explanation documents

Simulation Parameters
The V2V simulation uses:

Simulation time: 10 seconds
Vehicle A initial velocity: 20 m/s
Vehicle B initial velocity: 20 m/s
Initial vehicle gap: 15 m
Obstacle alert threshold: 15 m
V2V communication delay: 0.2 s
PV Digital Twin
The PV model includes three photovoltaic panels:

PV1: 45 series solar cells
PV2: 36 series solar cells
PV3: 36 series solar cells
PV2 and PV3 connected in parallel
Applications
This project demonstrates how vehicle-to-vehicle communication can improve road safety by reducing reaction time during obstacle detection and emergency braking scenarios.
