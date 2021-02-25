#include <dobot_m1.hpp>

int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, "m1_homing");
    dobot_m1::DobotM1 my_dobot;
    ROS_INFO("Start Homing...");
    my_dobot.Homing();
    return EXIT_SUCCESS;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("%s", e.what());
    return EXIT_FAILURE;
  }
}
