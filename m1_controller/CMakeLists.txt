cmake_minimum_required(VERSION 3.13.0)
project(m1_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  m1_msgs
  actionlib_msgs
  actionlib
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs m1_msgs
)

# m1_controller
add_executable(
  m1_controller
)

target_sources(m1_controller
  PUBLIC
    src/m1_controller.cpp
    src/dobot_m1.cpp
    src/dobot_m1_alarm.cpp
)

target_include_directories(m1_controller
  PUBLIC
    ${catkin_INCLUDE_DIRS}
    include/dobot_m1
    include/DobotDll
)

target_link_libraries(m1_controller
  ${catkin_LIBRARIES}
  DobotDllM1
)


target_link_directories(m1_controller
  PUBLIC
    lib/DobotDll_x64
)


# m1_homing
add_executable(
  m1_homing
)

target_sources(m1_homing
  PUBLIC
    src/m1_homing.cpp
    src/dobot_m1.cpp
    src/dobot_m1_alarm.cpp
)

target_include_directories(m1_homing
  PUBLIC
    ${catkin_INCLUDE_DIRS}
    include/dobot_m1
    include/DobotDll

)

target_link_libraries(m1_homing
  ${catkin_LIBRARIES}
  DobotDllM1
)

target_link_directories(m1_homing
  PUBLIC
    lib/DobotDll_x64
)