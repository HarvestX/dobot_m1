#pragma once

#include <cmath>
#include <limits>
#include <m1_msgs/M1CpCmd.h>
#include <m1_msgs/M1JogCmd.h>
#include <m1_msgs/M1PtpCmd.h>
#include <m1_msgs/M1PtpCmdService.h>
#include <ros/ros.h>

namespace dobot_m1
{
  class M1Validator
  {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::ServiceServer ptp_cmd_service_;
    bool PtpCmdServiceCallback_(m1_msgs::M1PtpCmdServiceRequest &req, m1_msgs::M1PtpCmdServiceResponse &res);

  public:
    M1Validator();
    ~M1Validator();
    bool IsValidPosition(float x, float y, float z);
    bool CheckPosition(float x, float y, float z);
    bool CheckJoint(float x, float y);
    float Deg2rad(float degree);
    bool is_left_hand = true;

    const float l1 = 200.0;
    const float l2 = 200.0;
  };
} // namespace dobot_m1
