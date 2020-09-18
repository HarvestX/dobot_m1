#include <m1_controller/M1Ptp.h>
#include <ros/ros.h>
#include <string>
#include "DobotDll.h"

class M1Controller
{
public:
  M1Controller() : nh_(), pnh_("~")
  {
    pnh_.getParam("port", port_);
    pnh_.getParam("vel", vel_default_);
    pnh_.getParam("acc", acc_default_);
    initDobot();
    sub_ = nh_.subscribe("ptp_command", 1, &M1Controller::ptpCallback, this);
  }

  ~M1Controller()
  {
    check_connection_(DisconnectDobot());
  }

  void connectDobot()
  {
    while (!check_connection_(ConnectDobot(port_.c_str(), 115200, 0, 0)))
    {
    }
  }

  void initDobot()
  {
    connectDobot();

    // set timeout
    uint32_t timeout = 50;
    while (!check_communication_(SetCmdTimeout(timeout), "Set Timeout"))
    {
    }
    // clean queue
    while (!check_communication_(SetQueuedCmdClear(), "Queue Clear"))
    {
    }

    while (!check_communication_(ClearAllAlarmsState(), "Clear All Alarm"))
    {
    }

    while (!check_communication_(SetQueuedCmdStartExec(), "Start Queue"))
    {
    }

    while (!check_communication_(SetArmOrientation(LeftyArmOrientation, true, nullptr), "Set Arm Orientation"))
    {
    }

    DisconnectDobot();
    return;
  }

  void ptpCallback(const m1_controller::M1Ptp &msg)
  {
    uint64_t lastIndex;
    // set velocity
    float vel;
    if (msg.vel == 0.0)
    {
      vel = vel_default_;
    }
    else
    {
      vel = msg.vel;
    }
    vel = check_velocity_(vel);

    // set acceleration
    float acc;
    if (msg.acc == 0.0)
    {
      acc = acc_default_;
    }
    else
    {
      acc = msg.acc;
    }
    acc = check_acceleration_(acc);

    PTPCommonParams ptpCommonParams;
    ptpCommonParams.velocityRatio = vel;
    ptpCommonParams.accelerationRatio = acc;

    PTPCmd cmd;
    cmd.ptpMode = msg.ptpMode;
    cmd.x = msg.x;
    cmd.y = msg.y;
    cmd.z = msg.z;
    cmd.r = msg.r;

    // Start session with dobot
    connectDobot();
    while (!check_communication_(SetPTPCommonParams(&ptpCommonParams, true, nullptr), "Set PTP Param"))
    {
    }
    while (!check_communication_(SetPTPCmd(&cmd, true, nullptr), "Set PTP Cmd"))
    {
    }
    DisconnectDobot();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  std::string port_;
  // TODO: check limit
  float vel_default_;
  float vel_min_ = 5.0;
  float vel_max_ = 100.0;
  float acc_default_;
  float acc_min_ = 5.0;
  float acc_max_ = 100.0;

  bool check_connection_(uint8_t status)
  {
    switch (status)
    {
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
      ROS_ERROR("Unexpected Return");
      break;
    }
    return false;
  }

  bool check_communication_(uint8_t status, std::string process_name)
  {
    switch (status)
    {
    case DobotCommunicate_NoError:
      ROS_INFO("Dobot Communication No Error: %s", process_name.c_str());
      return true;
    case DobotCommunicate_BufferFull:
      ROS_ERROR("Dobot Communication Buffer Full: %s", process_name.c_str());
      break;
    case DobotCommunicate_Timeout:
      ROS_ERROR("Dobot Communication Timeout: %s", process_name.c_str());
      break;
    case DobotCommunicate_InvalidParams:
      ROS_ERROR("Dobot Communication Invalid Params: %s", process_name.c_str());
      break;
    default:
      ROS_ERROR("Unexpected Return: %s", process_name.c_str());
      break;
    }
    return false;
  }

  float check_velocity_(float vel)
  {
    if (vel < vel_min_)
    {
      ROS_WARN("velocity modified to minimum value: %f", vel_min_);
      vel = vel_min_;
    }
    else if (vel > vel_max_)
    {
      ROS_WARN("velocity modified to maximum value: %f", vel_max_);
      vel = vel_max_;
    }
    return vel;
  }

  float check_acceleration_(float acc)
  {
    if (acc < acc_min_)
    {
      ROS_WARN("acceleration modified to minimum value: %f", acc_min_);
      acc = acc_min_;
    }
    else if (acc > acc_max_)
    {
      ROS_WARN("acceleration modified to maximum value: %f", acc_max_);
      acc = acc_max_;
    }
    return acc;
  }
};

void cpcmd(double x, double y, double z, double r)
{
  CPCmd cmd;
  uint64_t lastIndex;
  bool ret;

  while (SetQueuedCmdClear() != DobotCommunicate_NoError)
  {
  }
  while (SetCPCmd(&cmd, true, &lastIndex) != DobotCommunicate_NoError)
  {
  }

  uint64_t currentIndex;
  while (1)
  {
    while (GetQueuedCmdCurrentIndex(&currentIndex) != DobotCommunicate_NoError)
    {
    }
    if (lastIndex <= currentIndex)
      break;
  }
}

int tmp()
{
  uint64_t queuedCmdIndex;
  uint64_t lastIndex;

  // Connect to Dobot
  // ####### need!!! ################
  // sudo chmod a+rw /dev/ttyUSB0
  // ################################
  if (ConnectDobot("/dev/ttyUSB0", 115200, 0, 0) == DobotCommunicate_NoError)
  {
    ROS_INFO("Opened Dobot Communication");
  }
  else
  {
    ROS_ERROR("Invalid port name or Dobot is occupied by other application or "
              "something else :)");
  }
  // Set Commuinication timeout in ms
  uint32_t timeout = 20;
  SetCmdTimeout(timeout);

  // Set some Parameter and Initialisation
  // First clear the command queue
  if (SetQueuedCmdClear() == DobotCommunicate_NoError)
  {
    ROS_INFO("CMD Queue Cleared");
  }
  // and start the exec queue
  if (SetQueuedCmdStartExec() == DobotCommunicate_NoError)
  {
    ROS_INFO("CMD Queue Started");
  }

  // while (SetHOMEWithSwitch(0, true, nullptr) != DobotCommunicate_NoError) {}
  uint8_t i;
  for (i = 0; i < 1; i++)
  {
    cpcmd(400, 0, 200, 0);
    cpcmd(250, 50, 200, 0);
    cpcmd(250, -50, 200, 0);
  }

  // SetQueuedCmdStopExec();

  DisconnectDobot();
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "m1_controller");
  M1Controller m1_controller;
  ros::spin();
  return 0;
}
