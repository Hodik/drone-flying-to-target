video file: https://drive.google.com/file/d/18FEQsWQeOlC9kRz5se4jjYwf3F0jXmc_/view?usp=sharing


# Drone Control System

Python version: 3.8.10
A Python-based drone navigation and control system built with DroneKit.

## Technologies
- Python
- DroneKit
- PyMAVLink

## Features
- Automated takeoff and landing
- GPS waypoint navigation
- Multiple control modes (GUIDED, ALT_HOLD)
- Heading control via yaw commands
- Distance and bearing calculations
- Mission waypoint management

## Usage
Start a sitl process on 127.0.0.1:5762, run python3.8 main.py

```python
# Example: Connect and takeoff
connection_string = "tcp:127.0.0.1:5762"  # Simulator or actual drone
vehicle = connect(connection_string, wait_ready=True)
arm_and_takeoff(vehicle, 100)  # Takeoff to 100m altitude
```

## Requirements
- DroneKit Python API
- MAVLink compatible drone or simulator
