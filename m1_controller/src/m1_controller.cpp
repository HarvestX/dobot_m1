#include <dobot_m1.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "m1_controller");
  dobot_m1::DobotM1 my_dobot;
  ros::spin();
  return 0;
}
