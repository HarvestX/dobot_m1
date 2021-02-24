#include <dobot_m1.hpp>

namespace dobot_m1
{
DobotM1::DobotM1() : nh_(), pnh_("~")
{
  pnh_.param<std::string>("port", port_, "192.168.100.159");

  InitDobot();
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
  timer_ = nh_.createTimer(ros::Duration(1), &DobotM1::TimerCallback_, this);
  ptp_cmd_sub_ = nh_.subscribe("ptp_cmd", 1, &DobotM1::PtpCmdCallback_, this);
  cp_cmd_sub_ = nh_.subscribe("cp_cmd", 1, &DobotM1::CpCmdCallback_, this);
  jog_cmd_sub_ = nh_.subscribe("jog_cmd", 1, &DobotM1::JogCmdCallback_, this);
  ptp_service_ = nh_.advertiseService("ptp_service", &DobotM1::PtpCmdServiceCallback_, this);
  cp_service_ = nh_.advertiseService("cp_service", &DobotM1::CpCmdServiceCallback_, this);
}

DobotM1::~DobotM1()
{
  dobot_api::DisconnectDobot();
}

void DobotM1::InitDobot()
{
  dobot_m1_interface::ConnectDobot(DobotM1::port_, 115200);
  uint32_t timeout = 50;
  dobot_m1_interface::SetCmdTimeout(timeout);
  dobot_m1_interface::SetQueuedCmdClear();
  dobot_m1_interface::ClearAllAlarmsState();
  return;
}

void DobotM1::TimerCallback_(const ros::TimerEvent &)
{
  CheckAlarm_();

  // Publish Position
  dobot_api::Pose pose;
  uint8_t status = dobot_api::GetPose(&pose);
  if (!dobot_m1_interface::CheckCommunication(status))
    return;

  sensor_msgs::JointState joint_msg;
  joint_msg.header.stamp = ros::Time::now();
  joint_msg.name.resize(4);
  joint_msg.position.resize(4);
  joint_msg.name[0] = "joint1";
  joint_msg.position[0] = (float)pose.jointAngle[0];
  joint_msg.name[1] = "joint2";
  joint_msg.position[1] = (float)pose.jointAngle[1];
  joint_msg.name[2] = "joint3";
  joint_msg.position[2] = (float)pose.jointAngle[2];
  joint_msg.name[3] = "joint4";
  joint_msg.position[3] = (float)pose.jointAngle[3];
  joint_pub_.publish(joint_msg);
}

void DobotM1::PtpCmdCallback_(const m1_msgs::M1PtpCmd &msg)
{
  uint64_t lastIndex;
  dobot_api::PTPCmd cmd;
  cmd.ptpMode = msg.ptpMode;
  cmd.x = msg.x;
  cmd.y = msg.y;
  cmd.z = msg.z;
  cmd.r = msg.r;

  uint8_t status;
  std::string str;

  // Start session with dobot
  status = dobot_api::SetArmOrientation(dobot_api::LeftyArmOrientation, false, nullptr);
  dobot_m1_interface::CommunicationStatus2String(status, str);
  if (dobot_m1_interface::CheckCommunication(status))
    ROS_ERROR("%s", str.c_str());

  dobot_api::SetPTPCmd(&cmd, false, nullptr);
  dobot_m1_interface::CommunicationStatus2String(status, str);
  if (dobot_m1_interface::CheckCommunication(status))
    ROS_ERROR("%s", str.c_str());
  return;
}

void DobotM1::PtpParamsCallback_(const m1_msgs::M1PtpParams &msg)
{
  // set velocity acceleration
  float vel = CheckVelocity_(msg.vel);
  float acc = CheckAcceleration_(msg.acc);

  dobot_api::PTPCommonParams ptpCommonParams;
  ptpCommonParams.velocityRatio = vel;
  ptpCommonParams.accelerationRatio = acc;

  uint8_t status;
  status = dobot_api::SetPTPCommonParams(&ptpCommonParams, false, nullptr);
  if (!dobot_m1_interface::CheckCommunication(status))
  {
    std::string str;
    dobot_m1_interface::CommunicationStatus2String(status, str);
    ROS_ERROR("%s", str.c_str());
  }
}

bool DobotM1::PtpCmdServiceCallback_(m1_msgs::M1PtpCmdServiceRequest &req, m1_msgs::M1PtpCmdServiceResponse &res)
{
  // todo
  return true;
}

void DobotM1::CpCmdCallback_(const m1_msgs::M1CpCmd &msg)
{
  dobot_api::CPCmd cmd;
  cmd.cpMode = msg.cpMode;
  cmd.x = msg.x;
  cmd.y = msg.y;
  cmd.z = msg.z;

  // Start session with dobot
  uint8_t status;
  std::string str;

  // Cp command only works on RightyArmOrientation
  // Set Arm Orientation
  status = dobot_api::SetArmOrientation(dobot_api::RightyArmOrientation, true, nullptr);
  dobot_m1_interface::CommunicationStatus2String(status, str);
  if (!dobot_m1_interface::CheckCommunication(status))
    ROS_ERROR("%s", str.c_str());

  // Set Cp Cmd
  status = dobot_api::SetCPCmd(&cmd, false, nullptr);
  dobot_m1_interface::CommunicationStatus2String(status, str);
  if (!dobot_m1_interface::CheckCommunication(status))
    ROS_ERROR("%s", str.c_str());

  CheckAlarm_();
}

void DobotM1::CpParamsCallback_(const m1_msgs::M1CpParams &msg)
{
  float vel = CheckVelocity_(msg.vel);
  float acc = CheckAcceleration_(msg.acc);

  dobot_api::CPParams cpParams;
  cpParams.juncitionVel = vel;
  cpParams.planAcc = acc;

  uint8_t status;
  status = dobot_api::SetCPParams(&cpParams, false, nullptr);
  if (!dobot_m1_interface::CheckCommunication(status))
  {
    std::string str;
    dobot_m1_interface::CommunicationStatus2String(status, str);
    ROS_ERROR("%s", str.c_str());
  }

  CheckAlarm_();
}


bool DobotM1::CpCmdServiceCallback_(m1_msgs::M1CpCmdServiceRequest &req, m1_msgs::M1CpCmdServiceResponse &res)
{
  // todo
  return true;
}

void DobotM1::JogCmdCallback_(const m1_msgs::M1JogCmd &msg)
{
  dobot_api::JOGCmd cmd;
  cmd.isJoint = msg.isJoint;
  cmd.cmd = msg.jogCmd;

  uint8_t status;
  status = dobot_api::SetJOGCmd(&cmd, false, nullptr);
  if (!dobot_m1_interface::CheckCommunication(status))
  {
    std::string str;
    dobot_m1_interface::CommunicationStatus2String(status, str);
    ROS_ERROR("%s", str.c_str());
  }
  DobotM1::CheckAlarm_();
}

void DobotM1::JogParamsCallback_(const m1_msgs::M1JogParams &msg)
{
  // set velocity and acceleration
  float vel = CheckVelocity_(msg.vel);
  float acc = CheckAcceleration_(msg.acc);

  dobot_api::JOGCommonParams jogCommonParams;
  jogCommonParams.velocityRatio = vel;
  jogCommonParams.accelerationRatio = acc;

  uint8_t status;
  status = dobot_api::SetJOGCommonParams(&jogCommonParams, false, nullptr);
  if (!dobot_m1_interface::CheckCommunication(status))
  {
    std::string str;
    dobot_m1_interface::CommunicationStatus2String(status, str);
    ROS_ERROR("%s", str.c_str());
  }
  DobotM1::CheckAlarm_();
}

void DobotM1::Homing()
{
  // clean queue
  dobot_m1_interface::SetQueuedCmdClear();
  dobot_m1_interface::SetQueuedCmdStartExec();

  uint64_t last_index;
  uint64_t current_index;
  uint8_t status;

  status = dobot_api::SetHOMEWithSwitch(0, true, &last_index);
  dobot_m1_interface::CheckCommunicationWithException("SetHOMEWithSwitch", status);

  dobot_m1_interface::WaitQueuedCmd(last_index);

  ROS_INFO("Homing Successfully Finished!");
  return;
}

float DobotM1::CheckVelocity_(float vel)
{
  if (vel < MIN_VEL)
  {
    ROS_WARN("velocity modified to minimum value: %f", MAX_VEL);
    vel = MIN_VEL;
  }
  else if (vel > MAX_VEL)
  {
    ROS_WARN("velocity modified to maximum value: %f", MAX_VEL);
    vel = MAX_VEL;
  }
  return vel;
}

float DobotM1::CheckAcceleration_(float acc)
{
  if (acc < MIN_ACC)
  {
    ROS_WARN("acceleration modified to minimum value: %f", MIN_ACC);
    acc = MIN_ACC;
  }
  else if (acc > MAX_ACC)
  {
    ROS_WARN("acceleration modified to maximum value: %f", MAX_ACC);
    acc = MAX_ACC;
  }
  return acc;
}

void DobotM1::CheckAlarm_()
{
  uint32_t code;
  if (!dobot_m1_interface::TryGetAlarmCode(code))
  {
    ROS_ERROR("FAILED TO GET ALARM CODE");
    return;
  }

  if (dobot_m1_interface::CheckAlarmCode(code))
    return;

  std::string str;
  dobot_m1_interface::AlarmCode2String(code, str);
  ROS_ERROR("%s", str.c_str());

  if (!dobot_m1_interface::TrySetQueuedCmdClear())
    ROS_ERROR("FAILED TO SET QUEUED CMD CLEAR");

  if (!dobot_m1_interface::TryClearAllAlarmsState())
    ROS_ERROR("FAILED TO CLEAR ALL ALARMS STATE");
}

}  // namespace dobot_m1
