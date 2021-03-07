cmake_minimum_required(VERSION 3.13)
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
  CATKIN_DEPENDS
  joy
  m1_controller
  roscpp
  rospy
  std_msgs
  m1_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  m1_joy_commander
)

target_sources(m1_joy_commander
  PUBLIC
    src/joy_commander.cpp
)

target_include_directories(m1_joy_commander
  PUBLIC
    include/m1_joy
)

add_dependencies(
  m1_joy_commander
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  m1_joy_commander
  ${catkin_LIBRARIES}
)