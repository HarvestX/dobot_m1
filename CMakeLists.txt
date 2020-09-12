cmake_minimum_required(VERSION 3.0.2)
project(m1_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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
