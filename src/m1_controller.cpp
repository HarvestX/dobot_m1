#include "DobotDll.h"
#include <m1_controller/M1Ptp.h>
#include <ros/ros.h>
#include <string>

class M1Controller {
public:
  M1Controller() : nh_(), pnh_("~") {
    pnh_.getParam("port", port_);
    initDobot_();
    sub_ = nh_.subscribe("ptp_command", 1, &M1Controller::ptpCallback, this);
  }

  void ptpCallback(const m1_controller::M1Ptp &msg) { ROS_INFO("hogehoge"); }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  std::string port_;

  void initDobot_() { return; }

  bool check_connection_(uint8_t status) {
    switch (status) {
    case DobotConnect_NoError:
      ROS_INFO("Dobot Connect No Error");
      return true;
    case DobotConnect_NotFound:
      ROS_ERROR("Dobot Connect Not Found");
      break;
    case DobotConnect_Occupied:
      ROS_ERROR("Dobot Connect Occupied");
      break;
    default:
      break;
    }
    return false;
  }

  bool check_communication_(uint8_t status) {
    switch (status) {
    case DobotCommunicate_NoError:
      ROS_INFO("Dobot Communication No Error");
      return true;
    case DobotCommunicate_BufferFull:
      ROS_ERROR("Dobot Communication Buffer Full");
      break;
    case DobotCommunicate_Timeout:
      ROS_ERROR("Dobot Communication Timeout");
      break;
    case DobotCommunicate_InvalidParams:
      ROS_ERROR("Dobot Communication Invalid Params");
      break;
    default:
      ROS_ERROR("Unexpected Return");
      break;
    }
    return false;
  }
};

void cpcmd(double x, double y, double z, double r) {
  CPCmd cmd;
  uint64_t lastIndex;
  bool ret;

  cmd.cpMode = 1; // movel:2 movej:1
  cmd.x = x;
  cmd.y = y;
  cmd.z = z;

  while (SetQueuedCmdClear() != DobotCommunicate_NoError) {
  }
  while (SetCPCmd(&cmd, true, &lastIndex) != DobotCommunicate_NoError) {
  }

  uint64_t currentIndex;
  while (1) {
    while (GetQueuedCmdCurrentIndex(&currentIndex) !=
           DobotCommunicate_NoError) {
    }
    if (lastIndex <= currentIndex)
      break;
  }
}

int tmp() {
  uint64_t queuedCmdIndex;
  uint64_t lastIndex;

  // Connect to Dobot
  // ####### need!!! ################
  // sudo chmod a+rw /dev/ttyUSB0
  // ################################
  if (ConnectDobot("/dev/ttyUSB0", 115200, 0, 0) == DobotCommunicate_NoError) {
    ROS_INFO("Opened Dobot Communication");
  } else {
    ROS_ERROR("Invalid port name or Dobot is occupied by other application or "
              "something else :)");
  }
  // Set Commuinication timeout in ms
  uint32_t timeout = 20;
  SetCmdTimeout(timeout);

  // Set some Parameter and Initialisation
  // First clear the command queue
  if (SetQueuedCmdClear() == DobotCommunicate_NoError) {
    ROS_INFO("CMD Queue Cleared");
  }
  // and start the exec queue
  if (SetQueuedCmdStartExec() == DobotCommunicate_NoError) {
    ROS_INFO("CMD Queue Started");
  }

  // while (SetHOMEWithSwitch(0, true, nullptr) != DobotCommunicate_NoError) {}
  uint8_t i;
  for (i = 0; i < 1; i++) {
    cpcmd(400, 0, 200, 0);
    cpcmd(250, 50, 200, 0);
    cpcmd(250, -50, 200, 0);
  }

  // SetQueuedCmdStopExec();

  DisconnectDobot();
  return 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "m1_controller");
  M1Controller m1_controller;
  ros::spin();
  return 0;
}
