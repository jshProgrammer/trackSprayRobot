cmake_minimum_required(VERSION 3.0.2)
project(teleop)

find_package(catkin REQUIRED COMPONENTS
  robot_msgs
  rospy
  message_generation
)

add_message_files(
  FILES
  EmergencyReset.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime robot_msgs rospy
)
