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
)

link_directories(
  lib/DobotDll_x64
)


# m1_controller
add_executable(
  m1_controller src/m1_controller.cpp
)

target_link_libraries(
  m1_controller
  ${catkin_LIBRARIES}
  DobotDllM1
)


# m1_homing
add_executable(
  m1_homing src/m1_homing.cpp
)

target_link_libraries(
  m1_homing
  ${catkin_LIBRARIES}
  DobotDllM1
)