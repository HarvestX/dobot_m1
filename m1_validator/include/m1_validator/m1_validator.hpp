#ifndef ROS_CONTROL__M1_VALIDATOR_H
#define ROS_CONTROL__M1_VALIDATOR_H

#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <m1_msgs/M1Cp.h>
#include <m1_msgs/M1Jog.h>
#include <m1_msgs/M1Ptp.h>
#include <m1_msgs/M1PtpAction.h>
#include <ros/ros.h>

namespace dobot_m1 {
class M1Validator {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<m1_msgs::M1PtpAction> ptp_as_;

public:
  M1Validator();
  ~M1Validator();
  void actionPtpCallback(const m1_msgs::M1PtpGoalConstPtr &goal);
  bool isValidPosition(float x, float y, float z);
  bool checkPosition(float x, float y, float z);
  bool checkJoint(float x, float y);
  float deg2rad(float degree);
  bool is_left_hand = true;
  const float l1 = 200.0;
  const float l2 = 200.0;
};
} // namespace dobot_m1
#endif