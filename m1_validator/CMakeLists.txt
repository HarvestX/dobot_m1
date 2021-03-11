cmake_minimum_required(VERSION 3.13.0)
project(m1_validator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  m1_msgs
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs m1_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# m1_validator
add_executable(
  m1_validator src/m1_validator.cpp
)

target_include_directories(m1_validator
  PUBLIC
    include/m1_validator
)

add_dependencies(
  m1_validator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  m1_validator
  ${catkin_LIBRARIES}
)

# for test
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(utest_node test/utest_launch.test src/test/utest.cpp)
  target_link_libraries(utest_node ${catkin_LIBRARIES})
endif()
