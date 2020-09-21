cmake_minimum_required(VERSION 3.0.2)
project(m1_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
  M1Ptp.msg
  M1Cp.msg
  M1Jog.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/DobotDll
)

link_directories(
  lib/DobotDll_x64
)

add_executable(
  m1_controller src/m1_controller.cpp
)

target_link_libraries(
  m1_controller
  ${catkin_LIBRARIES}
  DobotDllM1
)
