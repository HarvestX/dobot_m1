
#ifndef ROS_CONTROL__DOBOT_M1_H
#define ROS_CONTROL__DOBOT_M1_H

#include <actionlib/server/simple_action_server.h>
#include <m1_msgs/M1Cp.h>
#include <m1_msgs/M1Jog.h>
#include <m1_msgs/M1Ptp.h>
#include <m1_msgs/M1PtpAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

namespace dobot_api {
#include "DobotDll.h"
}

namespace dobot_m1 {
class DobotM1 {
public:
  DobotM1();
  ~DobotM1();

  void connectDobot();
  void initDobot();
  void timerCallback(const ros::TimerEvent &);
  void ptpCallback(const m1_msgs::M1Ptp &msg);
  void cpCallback(const m1_msgs::M1Cp &msg);
  void jogCallback(const m1_msgs::M1Jog &msg);
  void actionPtpCallback(const m1_msgs::M1PtpGoalConstPtr &goal);
  void homing();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher joint_pub_;
  ros::Subscriber ptp_sub_;
  ros::Subscriber cp_sub_;
  ros::Subscriber jog_sub_;
  ros::Timer timer_;
  actionlib::SimpleActionServer<m1_msgs::M1PtpAction> ptp_as_;

  // std::string port_ = "/dev/ttyUSB0";
  std::string port_ = "192.168.100.159";

  float vel_default_;
  const float vel_min_ = 5.0;
  const float vel_max_ = 100.0;
  float acc_default_;
  const float acc_min_ = 5.0;
  const float acc_max_ = 100.0;

  bool check_connection_(uint8_t status);
  bool check_communication_(uint8_t status, std::string process_name);
  float check_velocity_(float vel);
  float check_acceleration_(float acc);
  void checkAlarm_();
  bool assertAlarm_();
  void publishAlarm_(dobot_api::alarmState alarmstate);
};
} // namespace dobot_m1

#endif
