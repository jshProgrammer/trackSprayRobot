# Autonomous Track Marking Robot

## Project Work in Cooperation with Mainfranken Racing

This project is developed in collaboration with **Mainfranken Racing**,
the Formula Student team based of the THWS.

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

The system will be implemented using **ROS Noetic** on embedded hardware
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
├── bringup/              # Central launch package
├── motor_control/        # PWM control and motor drivers
├── localization/         # GPS, RTK, NTRIP and localization nodes
├── teleop/               # Joystick control and emergency stop
```

## 🚀 Central Launch System

All major robot components can be started together using a centralized
bringup launch file.

### robot.launch

```xml
<launch>
    <include file="$(find motor_control)/launch/motor.launch"/>
    <include file="$(find localization)/launch/gps.launch"/>
    <include file="$(find teleop)/launch/teleop.launch"/>
</launch>
```

### Start the complete robot stack

Run from inside the catkin workspace (tracksprayrobot/robot):

```bash
roslaunch bringup robot.launch
```

------------------------------------------------------------------------


# 🗂️ Project Structure

    trackSprayRobot/
    │
    ├── README.md
    ├── .gitignore
    │
    ├── realrobot/ros_drive              # final ROS Noetic Workspace
    │   ├── src/
    │   │   ├── track_spray_robot/
    │   │   │   ├── config/
    │   │   │   ├── launch/
    │   │   │   ├── scripts/
    │   │   │   │   ├── gps/
    │
    ├── calculations/                   # Prototype-Calculations for navigation
    │
    ├── sensors/                        # Packages to test sensors individually
    │   ├── GPS_Sensor/
    │
    ├── backend/                        # TO BE DONE
    │   ├── api/
    │   ├── websocket_server/
    │   ├── robot_gateway/
    │   └── data_models/
    │
    ├── frontend/                       # TO BE DONE
    │   ├── web-app/
    │   │   ├── components/
    │   │   ├── map/
    │   │   ├── services/
    │   │   └── pages/
    │   │
    │   └── mobile-app/ (optional)
    │
    ├── shared/                         # TO BE DONE
    │   ├── map_models/
    │   └── communication_interfaces/
    │
    ├── docs/                           # TO BE DONE
    │   ├── architecture/
    │   ├── hardware/
    │   ├── requirements/
    │   └── diagrams/
    │
    └── simulations/                    # Robot simulations using Gazebo
        ├── ros_lidar_simulation/
        └── ros_lidar_simulation_Navigation/
        └── ros_lidar_simulation_Trackspray/



In future extension, the different scripts might be divided into individual packages depending on their responsibility, e.g.

    trackSprayRobot/
    ├── robot/     
    │   ├── src/
    │   │   ├── hardware_interface/
    │   │   ├── localization/
    │   │   ├── path_planning/
    │   │   ├── spray_controller/
    │   │   ├── mission_manager/
    │   │   └── status_monitor/

------------------------------------------------------------------------

# 📡 Communication Architecture

-   REST API for configuration
-   WebSocket for live updates
-   ROS Noetic Topics/Services internally
-   JSON / Protobuf message formats (TBD)

------------------------------------------------------------------------

# 📊 Status Information Available to User

-   Current position
-   Battery level
-   Estimated Spray fill level
-   Mission progress (%)
-   Current discipline
-   Error states

------------------------------------------------------------------------

# 🧪 Simulation & Testing

-   Gazebo Simulation (ROS Noetic)
-   Virtual test tracks
-   Field testing on asphalt

------------------------------------------------------------------------

# 🚀 Optional / Future Features

-   Obstacle Detection via LiDAR
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
