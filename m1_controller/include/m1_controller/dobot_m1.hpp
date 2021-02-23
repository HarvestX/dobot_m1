#pragma once

#include <m1_msgs/M1Cp.h>
#include <m1_msgs/M1Jog.h>
#include <m1_msgs/M1Ptp.h>
#include <m1_msgs/M1PtpService.h>
#include <m1_msgs/M1CpService.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

namespace dobot_api
{
#include "DobotDll.h"
}

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

    void connectDobot();
    void initDobot();
    void timerCallback(const ros::TimerEvent &);
    void ptpCallback(const m1_msgs::M1Ptp &msg);
    void cpCallback(const m1_msgs::M1Cp &msg);
    void jogCallback(const m1_msgs::M1Jog &msg);
    void homing();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher joint_pub_;
    ros::Subscriber ptp_sub_;
    ros::Subscriber cp_sub_;
    ros::Subscriber jog_sub_;
    ros::Timer timer_;

    // services
    ros::ServiceServer ptp_service_;
    ros::ServiceServer cp_service_;
    bool PtpServiceCallback_(m1_msgs::M1PtpServiceRequest &req, m1_msgs::M1PtpServiceResponse &res);
    bool CpServiceCallback_(m1_msgs::M1CpServiceRequest &req, m1_msgs::M1CpServiceResponse &res);

    std::string port_;

    bool check_connection_(uint8_t status);
    bool check_communication_(uint8_t status, std::string process_name);
    float check_velocity_(float vel);
    float check_acceleration_(float acc);
    void checkAlarm_();
    bool assertAlarm_();
    void publishAlarm_(dobot_api::alarmState alarmstate);
  };
} // namespace dobot_m1
