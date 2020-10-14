#include "dobot_m1.h"

namespace dobot_m1 {
DobotM1::DobotM1() : nh_(), pnh_("~") {
  pnh_.getParam("port", port_);
  pnh_.getParam("vel", vel_default_);
  pnh_.getParam("acc", acc_default_);
  initDobot();
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
  timer_ = nh_.createTimer(ros::Duration(1), &DobotM1::timerCallback, this);
  ptp_sub_ = nh_.subscribe("ptp_cmd", 1, &DobotM1::ptpCallback, this);
  cp_sub_ = nh_.subscribe("cp_cmd", 1, &DobotM1::cpCallback, this);
  jog_sub_ = nh_.subscribe("jog_cmd", 1, &DobotM1::jogCallback, this);
}

DobotM1::~DobotM1() { dobot_api::DisconnectDobot(); }

void DobotM1::connectDobot() {
  while (!check_connection_(
      dobot_api::ConnectDobot(port_.c_str(), 115200, 0, 0))) {
  }
}

void DobotM1::initDobot() {
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

  while (
      !check_communication_(dobot_api::SetArmOrientation(
                                dobot_api::LeftyArmOrientation, true, nullptr),
                            "Set Arm Orientation")) {
  }
  return;
}

void DobotM1::timerCallback(const ros::TimerEvent &) {
  checkAlarm_();

  // Publish Position
  dobot_api::Pose pose;
  if (GetPose(&pose) != dobot_api::DobotCommunicate_NoError) {
    return;
  };

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

void DobotM1::ptpCallback(const m1_msgs::M1Ptp &msg) {
  uint64_t lastIndex;
  // set velocity
  float vel;
  if (msg.vel == 0.0) {
    vel = vel_default_;
  } else {
    vel = msg.vel;
  }
  vel = check_velocity_(vel);

  // set acceleration
  float acc;
  if (msg.acc == 0.0) {
    acc = acc_default_;
  } else {
    acc = msg.acc;
  }
  acc = check_acceleration_(acc);

  dobot_api::PTPCommonParams ptpCommonParams;
  ptpCommonParams.velocityRatio = vel;
  ptpCommonParams.accelerationRatio = acc;

  dobot_api::PTPCmd cmd;
  cmd.ptpMode = msg.ptpMode;
  cmd.x = msg.x;
  cmd.y = msg.y;
  cmd.z = msg.z;
  cmd.r = msg.r;

  // Start session with dobot
  while (
      !check_communication_(dobot_api::SetArmOrientation(
                                dobot_api::LeftyArmOrientation, true, nullptr),
                            "Set Arm Orientation")) {
  }
  while (!check_communication_(
      SetPTPCommonParams(&ptpCommonParams, true, nullptr), "Set PTP Param")) {
  }
  while (!check_communication_(dobot_api::SetPTPCmd(&cmd, true, nullptr),
                               "Set PTP Cmd")) {
  }
  checkAlarm_();
}

void DobotM1::cpCallback(const m1_msgs::M1Cp &msg) {
  uint64_t lastIndex;
  // set velocity
  float vel;
  if (msg.vel == 0.0) {
    vel = vel_default_;
  } else {
    vel = msg.vel;
  }
  vel = check_velocity_(vel);

  // set acceleration
  float acc;
  if (msg.acc == 0.0) {
    acc = acc_default_;
  } else {
    acc = msg.acc;
  }
  acc = check_acceleration_(acc);

  dobot_api::CPParams cpParams;
  cpParams.juncitionVel = vel;
  cpParams.planAcc = acc;

  dobot_api::CPCmd cmd;
  cmd.cpMode = msg.cpMode;
  cmd.x = msg.x;
  cmd.y = msg.y;
  cmd.z = msg.z;

  // Start session with dobot
  // Cp command only works on RightyArmOrientation
  while (
      !check_communication_(dobot_api::SetArmOrientation(
                                dobot_api::RightyArmOrientation, true, nullptr),
                            "Set Arm Orientation")) {
  }
  while (!check_communication_(SetCPParams(&cpParams, true, nullptr),
                               "Set CP Param")) {
  }
  while (!check_communication_(SetCPCmd(&cmd, true, nullptr), "Set CP Cmd")) {
  }
  checkAlarm_();
}

void DobotM1::jogCallback(const m1_msgs::M1Jog &msg) {
  // set velocity
  float vel;
  if (msg.vel == 0.0) {
    vel = vel_default_;
  } else {
    vel = msg.vel;
  }
  vel = check_velocity_(vel);

  // set acceleration
  float acc;
  if (msg.acc == 0.0) {
    acc = acc_default_;
  } else {
    acc = msg.acc;
  }
  acc = check_acceleration_(acc);

  dobot_api::JOGCommonParams jogCommonParams;
  jogCommonParams.velocityRatio = vel;
  jogCommonParams.accelerationRatio = acc;

  dobot_api::JOGCmd cmd;
  cmd.isJoint = msg.isJoint;
  cmd.cmd = msg.jogCmd;

  // Start session with dobot
  while (!check_communication_(
      SetJOGCommonParams(&jogCommonParams, true, nullptr), "Set JOG Param")) {
  }
  while (!check_communication_(dobot_api::SetJOGCmd(&cmd, false, nullptr),
                               "Set JOG Cmd")) {
  }
  checkAlarm_();
}

void DobotM1::homing() {
  // clean queue
  while (!check_communication_(dobot_api::SetQueuedCmdClear(), "Queue Clear")) {
  }

  while (!check_communication_(dobot_api::SetQueuedCmdStartExec(),
                               "Start Queue")) {
  }

  uint64_t lastIndex;
  uint64_t currentIndex;

  while (!check_communication_(
      dobot_api::SetHOMEWithSwitch(0, true, &lastIndex), "Homing")) {
  }
  while (1) {
    while (dobot_api::GetQueuedCmdCurrentIndex(&currentIndex) !=
           dobot_api::DobotCommunicate_NoError) {
    }
    if (lastIndex <= currentIndex)
      break;
  }

  ROS_INFO("Homing Successfully Finished!");
  return;
}

bool DobotM1::check_connection_(uint8_t status) {
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

bool DobotM1::check_communication_(uint8_t status, std::string process_name) {
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

float DobotM1::check_velocity_(float vel) {
  if (vel < vel_min_) {
    ROS_WARN("velocity modified to minimum value: %f", vel_min_);
    vel = vel_min_;
  } else if (vel > vel_max_) {
    ROS_WARN("velocity modified to maximum value: %f", vel_max_);
    vel = vel_max_;
  }
  return vel;
}

float DobotM1::check_acceleration_(float acc) {
  if (acc < acc_min_) {
    ROS_WARN("acceleration modified to minimum value: %f", acc_min_);
    acc = acc_min_;
  } else if (acc > acc_max_) {
    ROS_WARN("acceleration modified to maximum value: %f", acc_max_);
    acc = acc_max_;
  }
  return acc;
}

void DobotM1::checkAlarm_() {
  // Check Alarm
  dobot_api::alarmState alarmstate;
  uint32_t len, maxLen = 32;

  if (dobot_api::GetAlarmsState(alarmstate.value, &len, maxLen) ==
      dobot_api::DobotConnect_NoError) {
    publishAlarm_(alarmstate);
    dobot_api::SetQueuedCmdClear();
    dobot_api::ClearAllAlarmsState();
  }
}

void DobotM1::publishAlarm_(dobot_api::alarmState alarmstate) {
  uint32_t code = alarmStateToCode(alarmstate);
  if (code == 0)
    return;
  const char *s = dobot_api::getAlartCodeName(dobot_api::AlarmCode(code));
  ROS_ERROR("%s", s);
}

} // namespace dobot_m1
