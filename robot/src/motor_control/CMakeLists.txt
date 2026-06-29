cmake_minimum_required(VERSION 2.8.3)
project(motor_control)

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

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pwm_drive.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
