cmake_minimum_required(VERSION 3.0.2)
project(m1_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  M1Ptp.msg
  M1Cp.msg
  M1Jog.msg
)

add_action_files(
  FILES
  M1Ptp.action
  M1Cp.action
)


generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)