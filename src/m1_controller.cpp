#include "ros/ros.h"
#include "DobotDll.h"

void cpcmd(double x,double y,double z, double r)
{
  CPCmd cmd;
  uint64_t lastIndex;
  bool ret;

  cmd.cpMode = 1; //movel:2 movej:1
  cmd.x = x;
  cmd.y = y;
  cmd.z = z;

  while (SetQueuedCmdClear() != DobotCommunicate_NoError) {}
  while (SetCPCmd(&cmd, true, &lastIndex) != DobotCommunicate_NoError) {}

  uint64_t currentIndex;
  while(1)
  {
    while(GetQueuedCmdCurrentIndex(&currentIndex) != DobotCommunicate_NoError) {}
    if (lastIndex <= currentIndex)break;
  }
}

int main(int argc, char**argv)
{
  uint64_t queuedCmdIndex;
  uint64_t lastIndex;

  // Connect to Dobot
  // ####### need!!! ################
  // sudo chmod a+rw /dev/ttyUSB0
  // ################################
  if (ConnectDobot("/dev/ttyUSB0", 115200, 0, 0) == DobotCommunicate_NoError) {
      ROS_INFO("Opened Dobot Communication");
  } else {
      ROS_ERROR("Invalid port name or Dobot is occupied by other application or something else :)");
  }
  // Set Commuinication timeout in ms
  uint32_t timeout=20;
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


  SetHOMECmd();

  SetQueuedCmdStopExec();

  DisconnectDobot();
  return 0;
}
