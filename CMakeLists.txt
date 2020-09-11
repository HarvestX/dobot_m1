cmake_minimum_required(VERSION 3.0.2)
project(m1_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

GET_FILENAME_COMPONENT(PROJECT_ROOT . ABSOLUTE CACHE INTERNAL "Path prefix for the project")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_ROOT}/include/DobotDll
)

link_directories(
  lib/DobotDll_x64
)

add_executable(
  ${PROJECT_NAME} src/m1_controller.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  DobotDllM1
)
