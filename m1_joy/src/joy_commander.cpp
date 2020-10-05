#include "joy_commander.h"

namespace dobot_m1 {
JoyCommander::JoyCommander() : nh_(), pnh_() {
  pnh_.getParam("isJoint", isJoint_);
  jog_pub_ = nh_.advertise<m1_msgs::M1Jog>("jog_cmd", 10);
  joy_sub_ = nh_.subscribe("/joy", 1, &JoyCommander::joyCallback, this);
}

void JoyCommander::joyCallback(const sensor_msgs::Joy &joy_msg) {

  m1_msgs::M1Jog jog_msg;
  jog_msg.isJoint = isJoint_;
  jog_msg.jogCmd = 0;

  float x = joy_msg.axes[1]; // L stick horizontal
  float y = joy_msg.axes[0]; // L stick vertical
  float z = joy_msg.axes[4]; // R stick vertical
  float r = joy_msg.axes[3]; // R stick horizontal

  float x_abs = abs(x);
  float y_abs = abs(y);
  float z_abs = abs(z);
  float r_abs = abs(r);

  float vel = std::max({x_abs, y_abs, z_abs, r_abs});
  if (vel > 0.4) {
    if (x_abs >= vel) {
      if (x > 0) {
        jog_msg.jogCmd = 1; // X+
      } else {
        jog_msg.jogCmd = 2; // X-
      }
    } else if (y_abs >= vel) {
      if (y > 0) {
        jog_msg.jogCmd = 3; // Y+
      } else {
        jog_msg.jogCmd = 4; // Y-
      }
    } else if (z_abs >= vel) {
      // Move Joint directly to avoid axis limitation
      jog_msg.isJoint = 1;
      if (z > 0) {
        jog_msg.jogCmd = 5; // Z+
      } else {
        jog_msg.jogCmd = 6; // Z-
      }
    } else if (r_abs >= vel) {
      // Move Joint directly to avoid axis limitation
      jog_msg.isJoint = 1;
      if (r > 0) {
        jog_msg.jogCmd = 7; // R+

      } else {
        jog_msg.jogCmd = 8; // R-
      }
    }
    jog_msg.vel = round(vel * 100.0);
    jog_msg.acc = 100.0;
  }
  jog_pub_.publish(jog_msg);
}

} // namespace dobot_m1

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_commander");
  dobot_m1::JoyCommander joy_commander;
  ros::Rate loop_rate(60);
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}