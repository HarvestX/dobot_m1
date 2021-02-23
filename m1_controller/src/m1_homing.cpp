#include <dobot_m1.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "m1_homing");
  dobot_m1::DobotM1 my_dobot;
  my_dobot.Homing();
  return 0;
}
