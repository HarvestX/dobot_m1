cmake_minimum_required(VERSION 3.0.2)
project(m1_joy)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  joy
  m1_controller
  m1_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS joy m1_controller roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  joy_commander src/joy_commander.cpp
)

target_link_libraries(
  joy_commander
  ${catkin_LIBRARIES}
)