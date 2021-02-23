#include <dobot_m1.hpp>

namespace dobot_m1
{
  DobotM1::DobotM1()
      : nh_(), pnh_("~")
  {
    pnh_.param<std::string>("port", port_, "192.168.100.159");

    InitDobot();
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
    timer_ = nh_.createTimer(ros::Duration(1), &DobotM1::TimerCallback_, this);
    ptp_sub_ = nh_.subscribe("ptp_cmd", 1, &DobotM1::PtpCallback_, this);
    cp_sub_ = nh_.subscribe("cp_cmd", 1, &DobotM1::CpCallback_, this);
    jog_sub_ = nh_.subscribe("jog_cmd", 1, &DobotM1::JogCallback_, this);
    ptp_service_ = nh_.advertiseService("ptp_service", &DobotM1::PtpServiceCallback_, this);
    cp_service_ = nh_.advertiseService("cp_service", &DobotM1::CpServiceCallback_, this);
  }

  DobotM1::~DobotM1() { dobot_api::DisconnectDobot(); }

  void DobotM1::InitDobot()
  {
    dobot_m1_interface::ConnectDobot(DobotM1::port_,  115200);
    ConnectDobot();

    // set timeout

    // set timeout
    uint32_t timeout = 50;
    dobot_m1_interface::SetCmdTimeout(timeout);
    dobot_m1_interface::SetQueuedCmdClear();

    // clean queue
    while (!CheckCommunication_(dobot_api::SetQueuedCmdClear(), "Queue Clear"))
    {
    }

    while (!CheckCommunication_(dobot_api::ClearAllAlarmsState(),
                                "Clear All Alarm"))
    {
    }

    while (!CheckCommunication_(dobot_api::SetQueuedCmdStartExec(),
                                "Start Queue"))
    {
    }

    return;
  }

  void DobotM1::TimerCallback_(const ros::TimerEvent &)
  {
    CheckAlarm_();

    // Publish Position
    dobot_api::Pose pose;
    if (GetPose(&pose) != dobot_api::DobotCommunicate_NoError)
    {
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

  void DobotM1::PtpCallback_(const m1_msgs::M1Ptp &msg)
  {
    uint64_t lastIndex;
    // set velocity acceleration
    float vel = CheckVelocity_(msg.vel);
    float acc = CheckAcceleration_(msg.acc);

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
        !CheckCommunication_(dobot_api::SetArmOrientation(
                                 dobot_api::LeftyArmOrientation, true, nullptr),
                             "Set Arm Orientation"))
    {
    }
    while (!CheckCommunication_(
        SetPTPCommonParams(&ptpCommonParams, true, nullptr), "Set PTP Param"))
    {
    }
    while (!CheckCommunication_(dobot_api::SetPTPCmd(&cmd, true, nullptr),
                                "Set PTP Cmd"))
    {
    }
    CheckAlarm_();
  }

  bool DobotM1::PtpServiceCallback_(m1_msgs::M1PtpServiceRequest &req, m1_msgs::M1PtpServiceResponse &res)
  {
    // todo
    return true;
  }

  void DobotM1::CpCallback_(const m1_msgs::M1Cp &msg)
  {
    uint64_t lastIndex;
    // set velocity acceleration
    float vel = CheckVelocity_(msg.vel);
    float acc = CheckAcceleration_(msg.acc);

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
        !CheckCommunication_(dobot_api::SetArmOrientation(
                                 dobot_api::RightyArmOrientation, true, nullptr),
                             "Set Arm Orientation"))
    {
    }
    while (!CheckCommunication_(SetCPParams(&cpParams, true, nullptr),
                                "Set CP Param"))
    {
    }
    while (!CheckCommunication_(SetCPCmd(&cmd, true, nullptr), "Set CP Cmd"))
    {
    }
    CheckAlarm_();
  }

  bool DobotM1::CpServiceCallback_(m1_msgs::M1CpServiceRequest &req, m1_msgs::M1CpServiceResponse &res)
  {
    // todo
    return true;
  }

  void DobotM1::JogCallback_(const m1_msgs::M1Jog &msg)
  {
    // set velocity and acceleration
    float vel = CheckVelocity_(msg.vel);
    float acc = CheckAcceleration_(msg.acc);

    dobot_api::JOGCommonParams jogCommonParams;
    jogCommonParams.velocityRatio = vel;
    jogCommonParams.accelerationRatio = acc;

    dobot_api::JOGCmd cmd;
    cmd.isJoint = msg.isJoint;
    cmd.cmd = msg.jogCmd;

    // Start session with dobot
    while (!CheckCommunication_(
        SetJOGCommonParams(&jogCommonParams, true, nullptr), "Set JOG Param"))
    {
    }
    while (!CheckCommunication_(dobot_api::SetJOGCmd(&cmd, false, nullptr),
                                "Set JOG Cmd"))
    {
    }
    CheckAlarm_();
  }

  void DobotM1::Homing()
  {
    // clean queue
    while (!CheckCommunication_(dobot_api::SetQueuedCmdClear(), "Queue Clear"))
    {
    }

    while (!CheckCommunication_(dobot_api::SetQueuedCmdStartExec(),
                                "Start Queue"))
    {
    }

    uint64_t lastIndex;
    uint64_t currentIndex;

    while (!CheckCommunication_(
        dobot_api::SetHOMEWithSwitch(0, true, &lastIndex), "Homing"))
    {
    }
    while (1)
    {
      while (dobot_api::GetQueuedCmdCurrentIndex(&currentIndex) !=
             dobot_api::DobotCommunicate_NoError)
      {
      }
      if (lastIndex <= currentIndex)
        break;
    }

    ROS_INFO("Homing Successfully Finished!");
    return;
  }

  bool DobotM1::CheckConnection_(uint8_t status)
  {
    switch (status)
    {
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

  bool DobotM1::CheckCommunication_(uint8_t status, std::string process_name)
  {
    switch (status)
    {
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
    // Check Alarm
    dobot_api::alarmState alarmstate;
    uint32_t len, maxLen = 32;

    if (dobot_api::GetAlarmsState(alarmstate.value, &len, maxLen) ==
        dobot_api::DobotConnect_NoError)
    {
      LogAlarm_(alarmstate);
      dobot_api::SetQueuedCmdClear();
      dobot_api::ClearAllAlarmsState();
    }
  }

  bool DobotM1::AssertAlarm_()
  {
    dobot_api::alarmState alarmstate;
    uint32_t len, maxLen = 32;
    bool state = false;
    if (dobot_api::GetAlarmsState(alarmstate.value, &len, maxLen) ==
        dobot_api::DobotConnect_NoError)
    {
      u_int32_t code = dobot_api::alarmStateToCode(alarmstate);

      if (code == 0)
        state = true;
    }
    return state;
  }

  void DobotM1::LogAlarm_(dobot_api::alarmState alarmstate)
  {
    uint32_t code = dobot_api::alarmStateToCode(alarmstate);
    if (code == 0)
      return;
    const char *s = dobot_api::getAlartCodeName(dobot_api::AlarmCode(code));
    ROS_ERROR("%s", s);
  }

} // namespace dobot_m1
