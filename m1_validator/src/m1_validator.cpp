#include "m1_validator.hpp"

namespace dobot_m1
{
  M1Validator::M1Validator() : nh_()
  {
    M1Validator::ptp_cmd_service_ = nh_.advertiseService("ptp_cmd_validator", &M1Validator::PtpCmdServiceCallback_, this);
  }

  M1Validator::~M1Validator() {}

  bool M1Validator::PtpCmdServiceCallback_(m1_msgs::M1PtpCmdServiceRequest &req, m1_msgs::M1PtpCmdServiceResponse &res)
  {
    if (req.m1_ptp_cmd.ptpMode > 3)
    {
      // only check for absolute coodination
      ROS_ERROR("validator is not implemented for given ptpMode");
      res.status.data = false;
    }
    else
    {
      float x = req.m1_ptp_cmd.x;
      float y = req.m1_ptp_cmd.y;
      float z = req.m1_ptp_cmd.z;
      res.status.data = M1Validator::IsValidPosition(x, y, z);
    }
    return true;
  }

  bool M1Validator::IsValidPosition(float x, float y, float z)
  {
    bool result = true;
    if (!M1Validator::CheckPosition(x, y, z))
    {
      ROS_ERROR("Invalid Position");
      result = false;
    }
    else if (!M1Validator::CheckJoint(x, y))
    {
      ROS_ERROR("Invalid Joint State");
      result = false;
    }
    return result;
  }

  bool M1Validator::CheckPosition(float x, float y, float z)
  {
    // max range is 400 sqare
    if (!(std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0)) - std::numeric_limits<float>::epsilon() < 400.0))
    {
      ROS_ERROR("r [%.3f] Out Of Range", std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0)));
      return false;
    };
    if (!((z > 10.0) && (z < 235.0)))
    {
      ROS_ERROR("z[%.3f] Out Of Range", z);
      return false;
    }
    return true;
  }

  bool M1Validator::CheckJoint(float x, float y)
  {
    float arm_coeff;
    if (M1Validator::is_left_hand)
    {
      arm_coeff = 1.0;
    }
    else
    {
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
    result &= (std::abs(j1) < M1Validator::Deg2rad(80.0));
    result &= (std::abs(j2) < M1Validator::Deg2rad(135.0));
    return result;
  }

  float M1Validator::Deg2rad(float degree) { return M_PI * degree / 180.0; }

} // namespace dobot_m1

int main(int argc, char **argv)
{
  ros::init(argc, argv, "m1_validator");
  dobot_m1::M1Validator my_validator;
  ros::spin();
  return 0;
}