#pragma once

#include <m1_msgs/M1CpCmd.h>
#include <m1_msgs/M1CpParams.h>
#include <m1_msgs/M1CpCmdService.h>
#include <m1_msgs/M1JogCmd.h>
#include <m1_msgs/M1JogParams.h>
#include <m1_msgs/M1PtpCmd.h>
#include <m1_msgs/M1PtpParams.h>
#include <m1_msgs/M1PtpCmdService.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>

#include "dobot_m1_interface.hpp"

namespace dobot_m1
{
const float MIN_VEL = 5.0;
const float MAX_VEL = 100.0;

const float MIN_ACC = 5.0;
const float MAX_ACC = 100.0;

class DobotM1
{
public:
  DobotM1();
  ~DobotM1();

  void ConnectDobot();
  void InitDobot();
  void Homing();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber ptp_cmd_sub_;
  ros::Subscriber cp_cmd_sub_;
  ros::Subscriber jog_cmd_sub_;
  void PtpCmdCallback_(const m1_msgs::M1PtpCmd &msg);
  void PtpParamsCallback_(const m1_msgs::M1PtpParams &msg);
  void CpCmdCallback_(const m1_msgs::M1CpCmd &msg);
  void CpParamsCallback_(const m1_msgs::M1CpParams &msg);
  void JogCmdCallback_(const m1_msgs::M1JogCmd &msg);
  void JogParamsCallback_(const m1_msgs::M1JogParams &msg);

  // timer publisher for joint state
  ros::Timer timer_;
  ros::Publisher joint_pub_;
  void TimerCallback_(const ros::TimerEvent &);

  // services
  ros::ServiceServer ptp_service_;
  ros::ServiceServer cp_service_;
  bool PtpCmdServiceCallback_(m1_msgs::M1PtpCmdServiceRequest &req, m1_msgs::M1PtpCmdServiceResponse &res);
  bool CpCmdServiceCallback_(m1_msgs::M1CpCmdServiceRequest &req, m1_msgs::M1CpCmdServiceResponse &res);

  std::string port_;

  bool CheckConnection_(uint8_t status);
  bool CheckCommunication_(uint8_t status, std::string process_name);
  float CheckVelocity_(float vel);
  float CheckAcceleration_(float acc);
  void CheckAlarm_();
  bool AssertAlarm_();
  void LogAlarm_(dobot_api::alarmState alarmstate);
};
}  // namespace dobot_m1
