#include <algorithm>
#include <m1_msgs/M1Jog.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace dobot_m1 {
class JoyCommander {
public:
  JoyCommander();
  void joyCallback(const sensor_msgs::Joy &joy_msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher jog_pub_;
  ros::Subscriber joy_sub_;
  bool isJoint_ = true;
  m1_msgs::M1Jog current_jog_msg_;
};
} // namespace dobot_m1