#ifndef ROS_CONTROL__M1_VALIDATOR_H
#define ROS_CONTROL__M1_VALIDATOR_H

#include<ros/ros.h>
#include <m1_msgs/M1Cp.h>
#include <m1_msgs/M1Jog.h>
#include <m1_msgs/M1Ptp.h>
#include <m1_msgs/M1PtpAction.h>
#include <actionlib/server/simple_action_server.h>

namespace dobot_m1 {
  class M1Validator
  {
  private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<m1_msgs::M1PtpAction> ptp_as_;

  public:
    M1Validator();
    ~M1Validator();
  void actionPtpCallback(const m1_msgs::M1PtpGoalConstPtr &goal);
  };
}
#endif