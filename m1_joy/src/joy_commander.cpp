#include "joy_commander.h"

namespace dobot_m1
{
  JoyCommander::JoyCommander() : nh_(), pnh_()
  {
    pnh_.getParam("isJoint", isJoint_);
    jog_cmd_pub_ = nh_.advertise<m1_msgs::M1JogCmd>("jog_cmd", 1);
    jog_params_pub_ = nh_.advertise<m1_msgs::M1JogParams>("jog_params", 1);
    joy_sub_ = nh_.subscribe("/joy", 1, &JoyCommander::joyCallback, this);

    // Initial Jog Command
    current_jog_cmd_msg_.isJoint = isJoint_;
    current_jog_cmd_msg_.jogCmd = 0;

    current_jog_params_msg_.vel = 0;
    current_jog_params_msg_.acc = 0;
  }

  void JoyCommander::joyCallback(const sensor_msgs::Joy &joy_msg)
  {

    m1_msgs::M1JogCmd target_jog_cmd_msg;
    m1_msgs::M1JogParams target_jog_params_msg;

    target_jog_cmd_msg.jogCmd = 0;

    float x = joy_msg.axes[1];          // L stick horizontal
    float y = joy_msg.axes[0];          // L stick vertical
    float z = joy_msg.axes[4];          // R stick vertical
    float r = joy_msg.axes[3];          // R stick horizontal
    int R1_buttom = joy_msg.buttons[4]; // R1 button

    if (R1_buttom)
    {
      toggleJoyMode();
    }

    float x_abs = abs(x);
    float y_abs = abs(y);
    float z_abs = abs(z);
    float r_abs = abs(r);

    float vel = std::max({x_abs, y_abs, z_abs, r_abs});
    if (vel > 0.4)
    { // ignore small input
      if (x_abs >= vel)
      {
        if (x > 0)
        {
          target_jog_cmd_msg.jogCmd = 1; // X+
        }
        else
        {
          target_jog_cmd_msg.jogCmd = 2; // X-
        }
      }
      else if (y_abs >= vel)
      {
        if (y > 0)
        {
          target_jog_cmd_msg.jogCmd = 3; // Y+
        }
        else
        {
          target_jog_cmd_msg.jogCmd = 4; // Y-
        }
      }
      else if (z_abs >= vel)
      {
        // Move Joint directly to avoid axis limitation
        target_jog_cmd_msg.isJoint = 1;
        if (z > 0)
        {
          target_jog_cmd_msg.jogCmd = 5; // Z+
        }
        else
        {
          target_jog_cmd_msg.jogCmd = 6; // Z-
        }
      }
      else if (r_abs >= vel)
      {
        // Move Joint directly to avoid axis limitation
        target_jog_cmd_msg.isJoint = 1;
        if (r > 0)
        {
          target_jog_cmd_msg.jogCmd = 7; // R+
        }
        else
        {
          target_jog_cmd_msg.jogCmd = 8; // R-
        }
      }
      target_jog_params_msg.vel = round(vel * 100.0);
      target_jog_params_msg.acc = 100.0;
    }

    // parameter
    if (target_jog_params_msg != current_jog_params_msg_)
    {
      // publish command
      current_jog_params_msg_ = target_jog_params_msg;

      jog_params_pub_.publish(target_jog_params_msg);
    }

    // cmd
    if (target_jog_cmd_msg != current_jog_cmd_msg_)
    {
      // publish command
      target_jog_cmd_msg.isJoint = isJoint_;
      current_jog_cmd_msg_ = target_jog_cmd_msg;

      jog_cmd_pub_.publish(target_jog_cmd_msg);
    }
  }
} // namespace dobot_m1

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_commander");
  dobot_m1::JoyCommander joy_commander;
  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}