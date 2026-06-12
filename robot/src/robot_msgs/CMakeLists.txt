cmake_minimum_required(VERSION 3.0.2)
project(robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# Importierbarer Python-Helfer (src/robot_msgs/status.py -> StatusReporter)
catkin_python_setup()

add_message_files(
  FILES
  RobotStatus.msg
  RobotState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs rospy
)
