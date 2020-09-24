#include "DobotDll.h"
#include <m1_msgs/M1Cp.h>
#include <m1_msgs/M1Jog.h>
#include <m1_msgs/M1Ptp.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

class M1Controller {
public:
  M1Controller() : nh_(), pnh_("~") {
    pnh_.getParam("port", port_);
    pnh_.getParam("vel", vel_default_);
    pnh_.getParam("acc", acc_default_);
    initDobot();
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
    timer_ =
        nh_.createTimer(ros::Duration(1), &M1Controller::timerCallback, this);
    ptp_sub_ = nh_.subscribe("ptp_cmd", 1, &M1Controller::ptpCallback, this);
    cp_sub_ = nh_.subscribe("cp_cmd", 1, &M1Controller::cpCallback, this);
    jog_sub_ = nh_.subscribe("jog_cmd", 1, &M1Controller::jogCallback, this);
  }

  ~M1Controller() { DisconnectDobot(); }

  void connectDobot() {
    while (!check_connection_(ConnectDobot(port_.c_str(), 115200, 0, 0))) {
    }
  }

  void initDobot() {
    connectDobot();

    // set timeout
    uint32_t timeout = 50;
    while (!check_communication_(SetCmdTimeout(timeout), "Set Timeout")) {
    }
    // clean queue
    while (!check_communication_(SetQueuedCmdClear(), "Queue Clear")) {
    }

    while (!check_communication_(ClearAllAlarmsState(), "Clear All Alarm")) {
    }

    while (!check_communication_(SetQueuedCmdStartExec(), "Start Queue")) {
    }

    // FIXME: LeftyArmOrientation is not working
    while (!check_communication_(
        SetArmOrientation(RightyArmOrientation, true, nullptr),
        "Set Arm Orientation")) {
    }
    return;
  }

  void timerCallback(const ros::TimerEvent &) {
    checkAlarm_();

    // Publish Position
    Pose pose;
    if (GetPose(&pose) != DobotCommunicate_NoError) {
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

  void ptpCallback(const m1_msgs::M1Ptp &msg) {
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
    while (!check_communication_(
        SetPTPCommonParams(&ptpCommonParams, true, nullptr), "Set PTP Param")) {
    }
    while (
        !check_communication_(SetPTPCmd(&cmd, true, nullptr), "Set PTP Cmd")) {
    }
    checkAlarm_();
  }

  void cpCallback(const m1_msgs::M1Cp &msg) {
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

    CPParams cpParams;
    cpParams.juncitionVel = vel;
    cpParams.planAcc = acc;

    CPCmd cmd;
    cmd.cpMode = msg.cpMode;
    cmd.x = msg.x;
    cmd.y = msg.y;
    cmd.z = msg.z;

    // Start session with dobot
    while (!check_communication_(SetCPParams(&cpParams, true, nullptr),
                                 "Set CP Param")) {
    }
    while (!check_communication_(SetCPCmd(&cmd, true, nullptr), "Set CP Cmd")) {
    }
    checkAlarm_();
  }

  void jogCallback(const m1_msgs::M1Jog &msg) {
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

    JOGCommonParams jogCommonParams;
    jogCommonParams.velocityRatio = vel;
    jogCommonParams.accelerationRatio = acc;

    JOGCmd cmd;
    cmd.isJoint = msg.isJoint;
    cmd.cmd = msg.jogCmd;

    // Start session with dobot
    while (!check_communication_(
        SetJOGCommonParams(&jogCommonParams, true, nullptr), "Set JOG Param")) {
    }
    while (
        !check_communication_(SetJOGCmd(&cmd, false, nullptr), "Set JOG Cmd")) {
    }
    checkAlarm_();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher joint_pub_;
  ros::Subscriber ptp_sub_;
  ros::Subscriber cp_sub_;
  ros::Subscriber jog_sub_;
  ros::Timer timer_;

  std::string port_;
  // TODO: check limit
  float vel_default_;
  float vel_min_ = 5.0;
  float vel_max_ = 100.0;
  float acc_default_;
  float acc_min_ = 5.0;
  float acc_max_ = 100.0;

  bool check_connection_(uint8_t status) {
    switch (status) {
    case DobotConnect_NoError:
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

  bool check_communication_(uint8_t status, std::string process_name) {
    switch (status) {
    case DobotCommunicate_NoError:
      ROS_INFO("%s", process_name.c_str());
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

  float check_velocity_(float vel) {
    if (vel < vel_min_) {
      ROS_WARN("velocity modified to minimum value: %f", vel_min_);
      vel = vel_min_;
    } else if (vel > vel_max_) {
      ROS_WARN("velocity modified to maximum value: %f", vel_max_);
      vel = vel_max_;
    }
    return vel;
  }

  float check_acceleration_(float acc) {
    if (acc < acc_min_) {
      ROS_WARN("acceleration modified to minimum value: %f", acc_min_);
      acc = acc_min_;
    } else if (acc > acc_max_) {
      ROS_WARN("acceleration modified to maximum value: %f", acc_max_);
      acc = acc_max_;
    }
    return acc;
  }

  void checkAlarm_() {
    // Check Alarm
    alarmState alarmstate;
    uint32_t len, maxLen = 32;

    if (GetAlarmsState(alarmstate.value, &len, maxLen) ==
        DobotConnect_NoError) {
      publishAlarm_(alarmstate);
      SetQueuedCmdClear();
      ClearAllAlarmsState();
    }
  }

  void publishAlarm_(alarmState alarmstate) {
    uint32_t code = alarmStateToCode(alarmstate);
    if (code == 0)
      return;
    const char *s = getAlartCodeName(AlarmCode(code));
    ROS_ERROR("%s", s);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "m1_controller");
  M1Controller m1_controller;
  ros::spin();
  return 0;
}
