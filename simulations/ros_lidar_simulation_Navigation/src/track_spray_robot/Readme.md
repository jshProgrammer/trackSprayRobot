# Track Spray Robot Simulation

This guide explains how to restart Gazebo, clean the workspace, rebuild the project, and launch the **Track Spray Robot** simulation.

---

## Steps to Run the Simulation

### 1. Kill Existing Gazebo Instances

Stop any running Gazebo processes to avoid conflicts.

```bash
pkill -f gazebo
```

---

### 2. Clean the Catkin Workspace

Remove previous build files.

```bash
catkin_make clean
```

**Alternative manual clean:**

```bash
cd ~/catkin_ws
rm -rf build devel
```

---

### 3. Build the Workspace

Compile the ROS workspace.

```bash
catkin_make
```

---

### 4. Source the Workspace

Load the workspace environment.

```bash
source devel/setup.bash
```

---

### 5. Launch the Robot Simulation

Start the Gazebo simulation for the Track Spray Robot.

```bash
roslaunch track_spray_robot track_spray_robot.launch
```

---

### 6. Optional: Visualize lidar points with rviz
```bash
rviz
```
- Enter LaserScan and select the topic /robot/scan
- Set fixed_frame to lidar_link