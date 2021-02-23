#pragma once

#include <m1_msgs/M1Cp.h>
#include <m1_msgs/M1CpService.h>
#include <m1_msgs/M1Jog.h>
#include <m1_msgs/M1Ptp.h>
#include <m1_msgs/M1PtpService.h>
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

  ros::Subscriber ptp_sub_;
  ros::Subscriber cp_sub_;
  ros::Subscriber jog_sub_;
  void PtpCallback_(const m1_msgs::M1Ptp &msg);
  void CpCallback_(const m1_msgs::M1Cp &msg);
  void JogCallback_(const m1_msgs::M1Jog &msg);

  // timer publisher for joint state
  ros::Timer timer_;
  ros::Publisher joint_pub_;
  void TimerCallback_(const ros::TimerEvent &);

  // services
  ros::ServiceServer ptp_service_;
  ros::ServiceServer cp_service_;
  bool PtpServiceCallback_(m1_msgs::M1PtpServiceRequest &req, m1_msgs::M1PtpServiceResponse &res);
  bool CpServiceCallback_(m1_msgs::M1CpServiceRequest &req, m1_msgs::M1CpServiceResponse &res);

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
