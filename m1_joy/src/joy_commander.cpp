#include <algorithm>
#include <m1_msgs/M1Jog.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class JoyCommander {
public:
  JoyCommander() : nh_() {
    jog_pub_ = nh_.advertise<m1_msgs::M1Jog>("jog_cmd", 10);
    joy_sub_ = nh_.subscribe("/joy", 1, &JoyCommander::joyCallback, this);
  }

  void joyCallback(const sensor_msgs::Joy &joy_msg) {

    m1_msgs::M1Jog jog_msg;
    jog_msg.isJoint = 1;
    jog_msg.jogCmd = 0;

    float x = joy_msg.axes[0]; // L stick horizontal
    float y = joy_msg.axes[1]; // L stick vertical
    float z = joy_msg.axes[4]; // R stick vertical

    float x_abs = abs(x);
    float y_abs = abs(y);
    float z_abs = abs(z);

    float vel = std::max({x_abs, y_abs, z_abs});
    if (vel > 0.3) {
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
        if (z > 0) {
          jog_msg.jogCmd = 5; // Z+
        } else {
          jog_msg.jogCmd = 6; // Z-
        }
      }
      jog_msg.vel = vel * 100.0;
      jog_msg.acc = 50.0;
    }
    jog_pub_.publish(jog_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher jog_pub_;
  ros::Subscriber joy_sub_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_commander");
  JoyCommander joy_commander;
  ros::spin();
  return 0;
}