#pragma once

#include <algorithm>
#include <m1_msgs/M1JogCmd.h>
#include <m1_msgs/M1JogParams.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace dobot_m1
{
  class JoyCommander
  {
  public:
    JoyCommander();
    void joyCallback(const sensor_msgs::Joy &joy_msg);
    void toggleJoyMode()
    {
      isJoint_ = !isJoint_;
      if (isJoint_)
      {
        ROS_WARN("Jog in Joint coordinate system");
      }
      else
      {
        ROS_WARN("Jog in Cartesian coordinate system");
      }
      return;
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher jog_cmd_pub_;
    ros::Publisher jog_params_pub_;
    ros::Subscriber joy_sub_;
    bool isJoint_ = true;
    m1_msgs::M1JogCmd current_jog_cmd_msg_;
    m1_msgs::M1JogParams current_jog_params_msg_;
  };
} // namespace dobot_m1