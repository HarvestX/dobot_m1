#ifndef ROS_CONTROL__M1_VALIDATOR_H
#define ROS_CONTROL__M1_VALIDATOR_H

#include <cmath>
#include <m1_msgs/M1CpCmd.h>
#include <m1_msgs/M1JogCmd.h>
#include <m1_msgs/M1PtpCmd.h>
#include <ros/ros.h>

namespace dobot_m1 {
class M1Validator {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

public:
  M1Validator();
  ~M1Validator();
  // void actionPtpCallback(const m1_msgs::M1PtpGoalConstPtr &goal);
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