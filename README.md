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
-   Avoids obstacles
-   Can be configured via App/Web interface
-   Reports live status information
-   Operates for \~40 minutes per discipline
-   Is robust against outdoor conditions (heat, light rain, uneven
    ground)

The system will be implemented using **ROS2** on embedded hardware
(Raspberry Pi).

------------------------------------------------------------------------

# 🏗️ System Architecture Overview

The system consists of three major domains:

1.  Robot Software (ROS2)
2.  Backend / Communication Layer
3.  Frontend (Web/App Interface)

------------------------------------------------------------------------

# 🧠 High-Level Epics

## 🔧 Backend / Hardware-Near

-   Hardware Control
-   Sensor Integration
-   Localization (RTK + IMU + Encoder Fusion)
-   Path Planning
-   Obstacle Detection (LiDAR + Ultrasonic)
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
-   Obstacle Marking via UI (optional)
-   Spray Level Monitoring Display

------------------------------------------------------------------------

# 🤖 Robot Architecture (ROS2)

The robot is based on a **Differential Drive** system.

### Sensors

-   RTK GPS
-   IMU
-   Wheel Encoders
-   LiDAR (360° obstacle detection)

### Actuators

-   BLDC Motors
-   Motor Controller
-   Spray Actuator (PWM controlled)
-   Adjustable Spray Pressure

### Compute

-   Raspberry Pi 4B

------------------------------------------------------------------------

# 🗂️ Project Structure

    trackSprayRobot/
    │
    ├── README.md
    │
    ├── robot/                         # ROS2 Workspace
    │   ├── src/
    │   │   ├── hardware_interface/
    │   │   ├── localization/
    │   │   ├── path_planning/
    │   │   ├── obstacle_detection/
    │   │   ├── spray_controller/
    │   │   ├── mission_manager/
    │   │   └── status_monitor/
    │   │
    │   ├── launch/
    │   └── config/
    │
    ├── backend/
    │   ├── api/
    │   ├── websocket_server/
    │   ├── robot_gateway/
    │   └── data_models/
    │
    ├── frontend/
    │   ├── web-app/
    │   │   ├── components/
    │   │   ├── map/
    │   │   ├── services/
    │   │   └── pages/
    │   │
    │   └── mobile-app/ (optional)
    │
    ├── shared/
    │   ├── map_models/
    │   └── communication_interfaces/
    │
    ├── docs/
    │   ├── architecture/
    │   ├── hardware/
    │   ├── requirements/
    │   └── diagrams/
    │
    └── simulations/
        ├── gazebo/
        └── test_tracks/

------------------------------------------------------------------------

# 📡 Communication Architecture

-   REST API for configuration
-   WebSocket for live updates
-   ROS2 Topics/Services internally
-   JSON / Protobuf message formats (TBD)

------------------------------------------------------------------------

# 📊 Status Information Available to User

-   Current position
-   Battery level
-   Spray fill level
-   Mission progress (%)
-   Current discipline
-   Obstacle detected warning
-   Error states

------------------------------------------------------------------------

# 🧪 Simulation & Testing

-   Gazebo Simulation (ROS2)
-   Virtual test tracks
-   Hardware-in-the-loop tests
-   Field testing on asphalt

------------------------------------------------------------------------

# 🚀 Optional / Future Features

-   Automatic discipline generator
-   AI-based obstacle classification
-   Cloud logging and analytics
-   Track optimization algorithm
-   Weather-adaptive spray control
-   Manual override mode via controller
-   Autonomous cone placement module (long-term vision)

------------------------------------------------------------------------

# 🔒 Non-Functional Requirements

-   Outdoor capable
-   40 min runtime minimum
-   Modular architecture
-   ROS2-based
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
