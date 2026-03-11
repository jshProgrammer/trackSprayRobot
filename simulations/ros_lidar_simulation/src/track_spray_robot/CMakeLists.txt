cmake_minimum_required(VERSION 2.8.3)
project(track_spray_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_ros
  gazebo_plugins
  robot_state_publisher
  tf
)

catkin_package()

# Installation der Python-Skripte
catkin_install_python(PROGRAMS
  scripts/obstacle_avoidance_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)