# Autonomous Track Marking Robot

## Project Work in Cooperation with Mainfranken Racing

This project is developed in collaboration with **Mainfranken Racing**,
the Formula Student team based at THWS.

The goal is to design and implement an **autonomous differential-drive
robot** capable of spraying race track markings on asphalt according to
Formula Student discipline specifications. The robot supports the
Driverless team by automating track setup, reducing manual effort, and
increasing testing efficiency.

------------------------------------------------------------------------

# 📌 Project Overview

Before the Driverless vehicle can be tested, a complete race track must
be set up according to strict competition rules (e.g., Acceleration,
Skid Pad, Autocross, Trackdrive).

## 🎯 Objective

Develop a mobile robot that:

-   Autonomously navigates on asphalt
-   Sprays track markings using spray chalk
-   Can be configured via App/Web interface
-   Avoids obstacles (marked via GUI)
-   Reports live status information
-   Operates for \~40 minutes per discipline
-   Is robust against outdoor conditions (heat, uneven
    ground)
-   Can be controlled using a PlayStation Controller for manual testing

The system is implemented using **ROS Noetic** on embedded hardware
(Raspberry Pi).

------------------------------------------------------------------------

# 🏗️ System Architecture Overview

The system consists of three major domains:

1.  Robot Software (ROS Noetic)
2.  Backend / Communication Layer
3.  Frontend (Web/App Interface)

------------------------------------------------------------------------

# 🧠 High-Level Epics

## 🔧 Backend / Hardware-Near

-   Hardware Control
-   Sensor Integration
-   Localization (RTK + IMU + Encoder Fusion)
-   Path Planning
-   Spray Actuator Control
-   Status Monitoring
-   Battery Monitoring
-   Mission Execution

## 💻 Frontend

-   Map Import (GeoJSON / Custom Format)
-   Map Data Structures
-   Map Visualization
-   Track Configuration
-   Discipline Templates (Acceleration, Skid Pad, etc.)
-   Remote Robot Connection
-   Live Status Dashboard
-   Obstacle Marking via UI
-   Spray Level Monitoring Display

------------------------------------------------------------------------

# 🤖 Robot Architecture (ROS Noetic)

The robot is based on a **Differential Drive** system.

### Sensors

-   RTK GPS
-   IMU
-   Wheel Encoders

### Actuators

-   BLDC Motors
-   Motor Controller
-   Spray Actuator (PWM controlled)

### Compute

- Raspberry Pi 4B
- Ubuntu 20.04
- ROS Noetic
- Optional remote access via SSH / Tailscale

------------------------------------------------------------------------

# 📦 ROS Package Structure

The robot software is organized into modular ROS packages.

```text
robot/
└── src/
    ├── bringup/                  # Central startup package
    │   └── launch/               # Full and phase-specific launch files
    ├── imu/                      # IMU package
    │   ├── launch/
    │   └── scripts/              # imu_node.py
    ├── localization/             # GPS, RTK and NTRIP package
    │   ├── config/               # RTK/NTRIP credentials template
    │   ├── launch/
    │   └── scripts/              # gps_node.py, ntrip_client.py
    ├── motor_control/            # PWM control and motion filtering package
    │   ├── config/
    │   ├── launch/
    │   └── scripts/              # PWM driver and Kalman filter
    ├── navigation/               # Waypoint navigation and spray package
    │   ├── config/
    │   ├── launch/
    │   ├── src/                  # Navigation nodes and helper modules
    ├── robot_msgs/               # Custom NavigationState and RobotStatus messages
    │   ├── msg/
    │   └── src/
    ├── robot_web_bridge/         # rosbridge launch and bridge documentation
    │   └── launch/
    └── teleop/                   # Joystick control and emergency stop
        ├── launch/
        ├── msg/
        └── scripts/
```

The `navigation` package is split into entry-point nodes and reusable helper
modules:

```text
robot/src/navigation/src/
├── navigation.py                 # Main navigation state machine
├── obstacle_avoidance_node.py    # Obstacle bypass handling
├── spray_counter.py              # Spray-can fill estimation
└── navigation/
    ├── geo.py                    # Geodesy helpers
    ├── heading_calibrator.py     # Initial heading calibration
    ├── motion_controller.py      # /cmd_vel_controll publisher wrapper
    ├── params.py                 # Navigation parameter loading
    ├── rtk_tracker.py            # GPS/RTK state tracking
    └── waypoint_guidepoints.py   # Intermediate guidepoint generation
```

## 🚀 Central Launch System

All major robot components can be started together using a centralized
bringup launch file.

### robot.launch

```xml
<launch>
    <include file="$(find teleop)/launch/teleop.launch"/>
    <include file="$(find motor_control)/launch/motor.launch"/>
    <include file="$(find localization)/launch/localization.launch"/>
    <include file="$(find imu)/launch/imu.launch"/>
    <include file="$(find navigation)/launch/navigation.launch"/>
</launch>
```

### Start the complete robot stack

Run from inside the catkin workspace (tracksprayrobot/robot):

```bash
roslaunch bringup robot.launch
```

If no frontend is used and the whole robot should start immediately, run the
helper script from the repository root:

```bash
./complete_robot_start.sh
```

This script cleans up old ROS processes, starts `pigpiod`, builds the catkin
workspace, sources it, and launches `bringup robot.launch`.

For the Raspberry Pi deployment flow, `roscore` and rosbridge are started by the
systemd service in `deploy/rosbridge.service`. The frontend then starts the robot
in two phases:

```bash
deploy/scripts/start_localization.sh
deploy/scripts/start_navigation.sh
```

See `deploy/README.md` for the full Pi setup.

### RTK/NTRIP credentials

`localization.launch` loads local RTK credentials from
`robot/src/localization/config/rtk_credentials.yaml`. Create it from the example:

```bash
cd robot
cp src/localization/config/rtk_credentials.example.yaml \
   src/localization/config/rtk_credentials.yaml
```

------------------------------------------------------------------------


# 🗂️ Project Structure

    trackSprayRobot/
    │
    ├── README.md
    ├── .gitignore
    │
    ├── robot/                          # current ROS Noetic catkin workspace
    │   ├── src/
    │   │   ├── bringup/
    │   │   ├── imu/
    │   │   ├── localization/
    │   │   ├── motor_control/
    │   │   ├── navigation/
    │   │   ├── robot_msgs/
    │   │   ├── robot_web_bridge/
    │   │   └── teleop/
    │
    ├── calculations/                   # Prototype-Calculations for navigation
    │
    ├── sensors/                        # Packages to test sensors individually
    │   ├── GPS_Sensor/
    │   ├── IMU/
    │
    ├── deploy/                         # Raspberry Pi service and start scripts
    │
    ├── shared_files/                   # runtime exchange for frontend/robot data
    │
    └── simulations/                    # Robot simulations using Gazebo
        ├── ros_lidar_simulation/
        └── ros_lidar_simulation_Navigation/
        └── ros_lidar_simulation_Trackspray/


The frontend repository can be found [here](https://github.com/davidhepp/tracksprayer).
------------------------------------------------------------------------

# 📡 Communication Architecture

-   rosbridge WebSocket on port `9090` for live ROS topic access
-   ROS Noetic Topics/Services internally
-   Shared files for mission inputs and spray-fill state:
    -   `/home/ubuntu/trackSprayRobot/shared_files/waypoints.json`
    -   `/home/ubuntu/trackSprayRobot/shared_files/obstacles.json`
    -   `/home/ubuntu/trackSprayRobot/shared_files/dosenstand.txt`
-   Frontend status topics:
    -   `/navigation_state` (`robot_msgs/NavigationState`)
    -   `/robot_status` (`robot_msgs/RobotStatus`)
    -   `/emergency_reset` (`teleop/EmergencyReset`)

Detailed rosbridge topic documentation is available in
`robot/src/robot_web_bridge/doku.md`.

------------------------------------------------------------------------

# 📊 Status Information Available to User

-   Current GPS position (`gps/fix`)
-   GPS/RTK quality (`gps/quality`)
-   Estimated spray fill level (`shared_files/dosenstand.txt`)
-   Mission state and waypoint progress (`/navigation_state`)
-   Error and warning states (`/robot_status`)
-   Emergency reset command (`/emergency_reset`)

Not implemented yet:

-   Battery level

------------------------------------------------------------------------

# 🧪 Simulation & Testing

-   Gazebo Simulation (ROS Noetic)
-   Virtual test tracks
-   Field testing on asphalt

------------------------------------------------------------------------

# 🚀 Optional / Future Features

-   Automatic obstacle detection via LiDAR (already simulated)
-   Automatic discipline generator
-   Track optimization algorithm

------------------------------------------------------------------------

# 🔒 Non-Functional Requirements

-   Outdoor capable
-   40 min runtime minimum
-   Modular architecture
-   ROS Noetic-based
-   Easy maintenance
-   Scalable software architecture
-   Fault-tolerant mission execution

------------------------------------------------------------------------

# 👥 Team

Project developed as part of a university project in cooperation with:

Mainfranken Racing

Contributors: 
- [Tom Knoblach](https://github.com/Gottschalk125)
- [Jasmin Wander](https://github.com/xjasx4)
- [David Heppenheimer](https://github.com/davidhepp)
- [Maximilian Keller](https://github.com/MaxCods)
- [Joshua Pfennig](https://github.com/jshProgrammer)

------------------------------------------------------------------------

# 📌 Summary

This project combines:

-   Robotics
-   Autonomous Navigation
-   Embedded Systems
-   Web Technologies
-   Distributed Systems
-   Human-Robot Interaction

It aims to significantly reduce track setup time for Formula Student
testing and improve test efficiency for the Driverless team.
