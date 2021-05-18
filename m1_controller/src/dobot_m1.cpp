#include "dobot_m1.hpp"
#include "m1_software_limit.hpp"

namespace dobot_m1
{
DobotM1::DobotM1() : nh_(), pnh_("~")
{
  DobotM1::pnh_.param<std::string>("port", port_, "192.168.100.159");

  InitDobot();

  DobotM1::joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
  DobotM1::timer_ = nh_.createTimer(ros::Duration(1), &DobotM1::TimerCallback_, this);
  DobotM1::ptp_cmd_sub_ = nh_.subscribe("ptp_cmd", 1, &DobotM1::PtpCmdCallback_, this);
  DobotM1::ptp_param_sub_ = nh_.subscribe("ptp_param", 1, &DobotM1::PtpParamsCallback_, this);
  DobotM1::cp_cmd_sub_ = nh_.subscribe("cp_cmd", 1, &DobotM1::CpCmdCallback_, this);
  DobotM1::cp_param_sub_ = nh_.subscribe("cp_param", 1, &DobotM1::CpParamsCallback_, this);
  DobotM1::jog_cmd_sub_ = nh_.subscribe("jog_cmd", 1, &DobotM1::JogCmdCallback_, this);
  DobotM1::jog_param_sub_ = nh_.subscribe("jog_param", 1, &DobotM1::JogParamsCallback_, this);
  DobotM1::ptp_cmd_service_ = nh_.advertiseService("ptp_cmd_service", &DobotM1::PtpCmdServiceCallback_, this);
  DobotM1::cp_cmd_service_ = nh_.advertiseService("cp_cmd_service", &DobotM1::CpCmdServiceCallback_, this);
  DobotM1::joint_cmd_service_ = nh_.advertiseService("joint_cmd_service", &DobotM1::JointCmdServiceCallback_, this);
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

void DobotM1::PtpCmd(uint8_t mode, float x, float y, float z, float r, bool is_lefthand)
{
  // Check Softare Limit
  if (!m1_software_limit::isValid(mode, x, y, z, is_lefthand))
  {
    throw std::runtime_error("given position is out of software limit [x: %.2f, y: %.2f, z: %.2f]");
  }

  dobot_api::ArmOrientation arm_orientation = dobot_api::RightyArmOrientation;
  if (is_lefthand)
  {
    arm_orientation = dobot_api::LeftyArmOrientation;
  }
  dobot_m1_interface::SetArmOrientation(arm_orientation);
  dobot_m1_interface::SetPtpCmd(mode, x, y, z, r);
  DobotM1::CheckAlarm_();
}

bool DobotM1::TryPtpCmd(uint8_t mode, float x, float y, float z, float r, bool is_lefthand)
{
  // Check Softare Limit
  if (!m1_software_limit::isValid(mode, x, y, z, is_lefthand))
  {
    ROS_ERROR("Given Postion is out of software limit");
    return false;
  }

  dobot_api::ArmOrientation arm_orientation = dobot_api::RightyArmOrientation;
  if (is_lefthand)
  {
    arm_orientation = dobot_api::LeftyArmOrientation;
  }
  if (!dobot_m1_interface::TrySetArmOrientation(arm_orientation))
  {
    ROS_ERROR("FAILED TO SET ARM ORIENTATION");
    return false;
  }
  if (!dobot_m1_interface::TrySetPtpCmd(mode, x, y, z, r))
  {
    ROS_ERROR("FAILED TO SET PTP CMD");
    return false;
  }
  return DobotM1::TryCheckAlarm_();
}

void DobotM1::PtpCmdCallback_(const m1_msgs::M1PtpCmd &msg)
{
  DobotM1::TryPtpCmd(msg.ptpMode, msg.x, msg.y, msg.z, msg.r, msg.is_lefthand);
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

  DobotM1::TryCheckAlarm_();
}

bool DobotM1::PtpCmdServiceCallback_(m1_msgs::M1PtpCmdServiceRequest &req, m1_msgs::M1PtpCmdServiceResponse &res)
{
  dobot_api::PTPCmd cmd;
  cmd.ptpMode = req.m1_ptp_cmd.ptpMode;
  cmd.x = req.m1_ptp_cmd.x;
  cmd.y = req.m1_ptp_cmd.y;
  cmd.z = req.m1_ptp_cmd.z;
  cmd.r = req.m1_ptp_cmd.r;

  uint8_t status;
  uint64_t last_index;
  std::string str;

  // Start session with dobot
  dobot_api::ArmOrientation arm_orientation;
  if (req.m1_ptp_cmd.is_lefthand)
  {
    arm_orientation = dobot_api::LeftyArmOrientation;
  }
  else
  {
    arm_orientation = dobot_api::RightyArmOrientation;
  }
  status = dobot_api::SetArmOrientation(arm_orientation, true, nullptr);

  if (!dobot_m1_interface::CheckCommunication(status))
  {
    dobot_m1_interface::CommunicationStatus2String(status, str);
    ROS_ERROR("%s", str.c_str());
    res.status.data = false;
    return false;
  }

  dobot_api::SetPTPCmd(&cmd, true, &last_index);
  if (!dobot_m1_interface::CheckCommunication(status))
  {
    dobot_m1_interface::CommunicationStatus2String(status, str);
    ROS_ERROR("%s", str.c_str());
    res.status.data = false;
    return false;
  }

  // Check Error
  if (!DobotM1::TryCheckAlarm_())
  {
    res.status.data = false;
    return false;
  }

  // Wait Queued Cmd
  if (!dobot_m1_interface::TryWaitQueuedCmd(last_index))
  {
    ROS_ERROR("FAILED TO WAIT QUEUED CMD");
    res.status.data = false;
    return false;
  }

  res.status.data = true;
  return true;
}

void DobotM1::CpCmd(uint8_t mode, float x, float y, float z)
{
  dobot_m1_interface::SetArmOrientationRight();
  dobot_m1_interface::SetCpCmd(mode, x, y, z);
  DobotM1::CheckAlarm_();
}

bool DobotM1::TryCpCmd(uint8_t mode, float x, float y, float z)
{
  if (!dobot_m1_interface::TrySetArmOrientationRight())
  {
    ROS_ERROR("FAILED TO SET ARM ORIENTATION");
    return false;
  }
  if (!dobot_m1_interface::TrySetCpCmd(mode, x, y, z))
  {
    ROS_ERROR("FAILED TO SET CP CMD");
    return false;
  }
  return DobotM1::TryCheckAlarm_();
}

void DobotM1::CpCmdCallback_(const m1_msgs::M1CpCmd &msg)
{
  DobotM1::TryCpCmd(msg.cpMode, msg.x, msg.y, msg.z);
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

  DobotM1::TryCheckAlarm_();
}

bool DobotM1::CpCmdServiceCallback_(m1_msgs::M1CpCmdServiceRequest &req, m1_msgs::M1CpCmdServiceResponse &res)
{
  // todo
  dobot_api::CPCmd cmd;
  cmd.cpMode = req.m1_cp_cmd.cpMode;
  cmd.x = req.m1_cp_cmd.x;
  cmd.y = req.m1_cp_cmd.y;
  cmd.z = req.m1_cp_cmd.z;

  uint8_t status;
  uint64_t last_index;
  std::string str;

  // Cp command only works on RightyArmOrientation
  // Set Arm Orientation
  status = dobot_api::SetArmOrientation(dobot_api::RightyArmOrientation, true, nullptr);
  dobot_m1_interface::CommunicationStatus2String(status, str);
  if (!dobot_m1_interface::CheckCommunication(status))
  {
    ROS_ERROR("%s", str.c_str());
    res.status.data = false;
    return false;
  }

  // Set Cp Cmd
  status = dobot_api::SetCPCmd(&cmd, true, &last_index);
  dobot_m1_interface::CommunicationStatus2String(status, str);
  if (!dobot_m1_interface::CheckCommunication(status))
  {
    ROS_ERROR("%s", str.c_str());
    res.status.data = false;
    return false;
  }

  // Check Error
  if (!DobotM1::TryCheckAlarm_())
  {
    res.status.data = false;
    return false;
  }

  // Wait Queued Cmd
  if (!dobot_m1_interface::TryWaitQueuedCmd(last_index))
  {
    ROS_ERROR("FAILED TO WAIT QUEUED CMD");
    res.status.data = false;
    return false;
  }

  res.status.data = true;
  return true;
}

void DobotM1::JogParams(float vel, float acc)
{
  float fixed_vel = CheckVelocity_(vel);
  float fixed_acc = CheckAcceleration_(acc);
  dobot_m1_interface::SetJogParams(fixed_vel, fixed_acc);
  DobotM1::CheckAlarm_();
}

bool DobotM1::TryJogParams(float vel, float acc)
{
  float fixed_vel = CheckVelocity_(vel);
  float fixed_acc = CheckAcceleration_(acc);
  if (!dobot_m1_interface::TrySetJogParams(fixed_vel, fixed_acc))
  {
    ROS_ERROR("FAILED TO SET JOG PARAMS");
    return false;
  }
  return DobotM1::TryCheckAlarm_();
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
  DobotM1::TryCheckAlarm_();
}

void DobotM1::JogParamsCallback_(const m1_msgs::M1JogParams &msg)
{
  DobotM1::TryJogParams(msg.vel, msg.acc);
}

bool DobotM1::JointCmdServiceCallback_(m1_msgs::M1JointCmdServiceRequest &req, m1_msgs::M1JointCmdServiceResponse &res)
{
  bool is_in_valid_range = false;
  uint8_t joint_idx;
  uint8_t positive_rotation_cmd;
  uint8_t negative_rotation_cmd;
  switch (req.m1_joint_cmd.jogCmd)
  {
    case dobot_m1_interface::REAR:
      is_in_valid_range = (std::abs(req.m1_joint_cmd.degree) < dobot_m1_interface::DOBOT_REAR_JOINT_HARDWARE_LIMIT);
      joint_idx = 0;
      positive_rotation_cmd = dobot_api::JogAPPressed;
      negative_rotation_cmd = dobot_api::JogANPressed;
      break;
    case dobot_m1_interface::FRONT:  // Fore arm rotate negative
      is_in_valid_range = (std::abs(req.m1_joint_cmd.degree) < dobot_m1_interface::DOBOT_FORE_JOINT_HARDWARE_LIMIT);
      joint_idx = 1;
      positive_rotation_cmd = dobot_api::JogBPPressed;
      negative_rotation_cmd = dobot_api::JogBNPressed;
      break;
    case dobot_m1_interface::END:
      is_in_valid_range = true;
      joint_idx = 3;
      positive_rotation_cmd = dobot_api::JogDPPressed;
      negative_rotation_cmd = dobot_api::JogDNPressed;
      break;
    default:
      ROS_ERROR("invalid jogCmd");
      res.status.data = false;
      return false;
  }

  if (!is_in_valid_range)
  {
    ROS_ERROR("degree is out of range");
    res.status.data = false;
    return false;
  }

  while (1)
  {
    dobot_api::Pose pose;
    dobot_api::JOGCmd jog_cmd;
    uint8_t status;

    status = dobot_api::GetPose(&pose);
    if (!dobot_m1_interface::CheckCommunication(status))
      continue;
    const float angle = (float)pose.jointAngle[joint_idx];
    const float diff = req.m1_joint_cmd.degree - angle;
    if (std::abs(diff) < dobot_m1_interface::ANGLE_EPSILON)
    {
      break;
    }

    jog_cmd.isJoint = true;
    if (diff > 0)
    {
      jog_cmd.cmd = positive_rotation_cmd;
    }
    else
    {
      jog_cmd.cmd = negative_rotation_cmd;
    }

    status = dobot_api::SetJOGCmd(&jog_cmd, false, nullptr);
    if (!dobot_m1_interface::CheckCommunication(status))
    {
      std::string str;
      dobot_m1_interface::CommunicationStatus2String(status, str);
      ROS_ERROR("%s", str.c_str());
      dobot_m1_interface::TryIdleJog();
      res.status.data = false;
      return false;
    }
  }
  dobot_m1_interface::TryIdleJog();
  res.status.data = true;
  return true;
}

void DobotM1::Homing()
{
  // clean queue
  dobot_m1_interface::SetQueuedCmdClear();
  dobot_m1_interface::SetQueuedCmdStartExec();

  uint64_t last_index;
  uint8_t status;

  status = dobot_api::SetHOMEWithSwitch(0, true, &last_index);
  dobot_m1_interface::CheckCommunicationWithException("SetHOMEWithSwitch", status);

  DobotM1::CheckAlarm_();
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
  dobot_m1_interface::GetAlarmCode(code);
  if (dobot_m1_interface::CheckAlarmCode(code))
    return;
  std::string str;
  dobot_m1_interface::AlarmCode2String(code, str);
  dobot_m1_interface::SetQueuedCmdClear();
  dobot_m1_interface::ClearAllAlarmsState();
  throw std::runtime_error(str);
}

bool DobotM1::TryCheckAlarm_()
{
  uint32_t code;
  if (!dobot_m1_interface::TryGetAlarmCode(code))
  {
    ROS_ERROR("FAILED TO GET ALARM CODE");
    return false;
  }

  if (dobot_m1_interface::CheckAlarmCode(code))
    // OK
    return true;

  std::string str;
  dobot_m1_interface::AlarmCode2String(code, str);
  ROS_ERROR("%s", str.c_str());

  if (!dobot_m1_interface::TrySetQueuedCmdClear())
  {
    ROS_ERROR("FAILED TO SET QUEUED CMD CLEAR");
    return false;
  }

  if (!dobot_m1_interface::TryClearAllAlarmsState())
  {
    ROS_ERROR("FAILED TO CLEAR ALL ALARMS STATE");
    return false;
  }
  return false;
}

}  // namespace dobot_m1
