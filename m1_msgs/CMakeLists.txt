cmake_minimum_required(VERSION 3.0.2)
project(m1_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  M1PtpCmd.msg
  M1PtpParams.msg
  M1CpCmd.msg
  M1CpParams.msg
  M1JogCmd.msg
  M1JogParams.msg
  M1JointCmd.msg
)

add_service_files(
  FILES
  M1PtpCmdService.srv
  M1PtpParamsService.srv
  M1CpCmdService.srv
  M1CpParamsService.srv
  M1JointCmdService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
