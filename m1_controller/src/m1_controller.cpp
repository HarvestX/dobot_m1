#include <dobot_m1.hpp>

int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, "m1_controller");
    dobot_m1::DobotM1 my_dobot;
    ROS_INFO("Dobot M1 Ready");
    ros::spin();
    return EXIT_SUCCESS;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("%s", e.what());
    return EXIT_FAILURE;
  }
}
