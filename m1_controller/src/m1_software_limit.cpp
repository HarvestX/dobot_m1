#include "m1_software_limit.hpp"

namespace m1_software_limit
{
bool isValid(float abs_x, float abs_y, float abs_z)
{
  // limit for realsense
  if (abs_y < 0.0 && abs_x < 200.0)
  {
    return false;
  }

  // limit for xv1 wheel
  if (abs_z < 74.0 && abs_x < 280.0 && abs_y < -180.0)
  {
    return false;
  }
  // limit for body
  if (abs_z < 60.0 && abs_x < 280.0)
  {
    return false;
  }
  return true;
}
}  // namespace m1_software_limit
