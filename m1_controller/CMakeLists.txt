cmake_minimum_required(VERSION 3.0.2)
project(m1_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  m1_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs m1_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/DobotDll
  include/m1_controller
)

link_directories(
  lib/DobotDll_x64
)

# Dobot M1
add_library(
  dobot_m1 src/dobot_m1.cpp
)

target_link_libraries(
  dobot_m1
  ${catkin_LIBRARIES}
  DobotDllM1
)

# m1_controller
add_executable(
  m1_controller src/m1_controller.cpp
)

target_link_libraries(
  m1_controller
  ${catkin_LIBRARIES}
  dobot_m1
)


# m1_homing
add_executable(
  m1_homing src/m1_homing.cpp
)

target_link_libraries(
  m1_homing
  ${catkin_LIBRARIES}
  dobot_m1
)