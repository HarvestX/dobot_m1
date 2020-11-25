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
  if (goal->m1_ptp.ptpMode > 3) {
    // only check for absolute coodination
    ROS_ERROR("validator is not implemented for given ptpMode");
    result.status.data = false;
  } else {
    float x = goal->m1_ptp.x;
    float y = goal->m1_ptp.y;
    float z = goal->m1_ptp.z;
    result.status.data = M1Validator::isValidPosition(x, y, z);
  }
  M1Validator::ptp_as_.setSucceeded(result);
  return;
}

bool M1Validator::isValidPosition(float x, float y, float z) {
  bool result = true;
  if (!M1Validator::checkPosition(x, y, z)) {
    result = false;
  } else if (!M1Validator::checkJoint(x, y)) {
    result = false;
  }
  return result;
}

bool M1Validator::checkPosition(float x, float y, float z) {
  // max range is 400 sqare
  bool result = true;
  result &= ((std::pow(x, 2.0) + std::pow(y, 2.0)) < 160000.0);
  result &= ((z > 10.0) && (z < 235.0));
  return result;
}

bool M1Validator::checkJoint(float x, float y) {
  float arm_coeff;
  if (M1Validator::is_left_hand) {
    arm_coeff = 1.0;
  } else {
    arm_coeff = -1.0;
  }

  bool result = true;
  float acos_term =
      (std::pow(x, 2.0) + std::pow(y, 2.0) + std::pow(M1Validator::l1, 2.0) -
       std::pow(M1Validator::l2, 2.0)) /
      (2.0 * M1Validator::l1 * std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0)));
  float j1 = arm_coeff * std::acos(acos_term) + std::atan2(y, x);
  float j2 = std::atan2(y - M1Validator::l1 * std::sin(j1),
                        x - M1Validator::l1 * std::cos(j1)) -
             j1;
  result &= (std::abs(j1) < M1Validator::deg2rad(80.0));
  result &= (std::abs(j2) < M1Validator::deg2rad(135.0));
  return result;
}

float M1Validator::deg2rad(float degree) { return M_PI * degree / 180.0; }

} // namespace dobot_m1

int main(int argc, char **argv) {
  ros::init(argc, argv, "m1_validator");
  dobot_m1::M1Validator my_validator;
  ros::spin();
  return 0;
}