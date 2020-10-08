#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

namespace dobot_api {
#include "DobotDll.h"
}


bool check_connection_(uint8_t status) {
  switch (status) {
  case dobot_api::DobotConnect_NoError:
    return true;
  case dobot_api::DobotConnect_NotFound:
    ROS_ERROR("Dobot Connect Not Found");
    break;
  case dobot_api::DobotConnect_Occupied:
    ROS_ERROR("Dobot Connect Occupied");
    break;
  default:
    ROS_ERROR("Unexpected Return");
    break;
  }
  return false;
}

bool check_communication_(uint8_t status, std::string process_name) {
  switch (status) {
  case dobot_api::DobotCommunicate_NoError:
    ROS_INFO("%s", process_name.c_str());
    return true;
  case dobot_api::DobotCommunicate_BufferFull:
    ROS_ERROR("Dobot Communication Buffer Full: %s", process_name.c_str());
    break;
  case dobot_api::DobotCommunicate_Timeout:
    ROS_ERROR("Dobot Communication Timeout: %s", process_name.c_str());
    break;
  case dobot_api::DobotCommunicate_InvalidParams:
    ROS_ERROR("Dobot Communication Invalid Params: %s", process_name.c_str());
    break;
  default:
    ROS_ERROR("Unexpected Return: %s", process_name.c_str());
    break;
  }
  return false;
}

void connectDobot() {
  while (!check_connection_(
      dobot_api::ConnectDobot("/dev/ttyUSB0", 115200, 0, 0))) {
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "m1_controller");
  connectDobot();

  // set timeout
  uint32_t timeout = 50;
  while (
      !check_communication_(dobot_api::SetCmdTimeout(timeout), "Set Timeout")) {
  }
  // clean queue
  while (!check_communication_(dobot_api::SetQueuedCmdClear(), "Queue Clear")) {
  }

  while (!check_communication_(dobot_api::ClearAllAlarmsState(),
                               "Clear All Alarm")) {
  }

  while (!check_communication_(dobot_api::SetQueuedCmdStartExec(),
                               "Start Queue")) {
  }

  uint64_t lastIndex;
  uint64_t currentIndex;

  while (!check_communication_(dobot_api::SetHOMEWithSwitch(0, true, &lastIndex), "Homing")) {
  }
  while (1) {
    while (dobot_api::GetQueuedCmdCurrentIndex(&currentIndex) !=
           dobot_api::DobotCommunicate_NoError) {
    }
    if (lastIndex <= currentIndex)
      break;
  }

  ROS_INFO("Homing Successfully Finished!");
  return 0;
}
