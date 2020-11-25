#include <m1_validator.hpp>

namespace dobot_m1 {

M1Validator::M1Validator()
    : ptp_as_(nh_, "m1_ptp_action",
              boost::bind(&M1Validator::actionPtpCallback, this, _1), false),
      nh_(), pnh_("~") {}

M1Validator::~M1Validator() {}

void M1Validator::actionPtpCallback(const m1_msgs::M1PtpGoalConstPtr &goal) {
  m1_msgs::M1PtpResult result;
  result.id = goal->id;

  M1Validator::ptp_as_.setSucceeded(result);
  return;
}

} // namespace dobot_m1

int main(int argc, char **argv) {
  ros::init(argc, argv, "m1_validator");
  dobot_m1::M1Validator my_validator;
  ros::spin();
  return 0;
}