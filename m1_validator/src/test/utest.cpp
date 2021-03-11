#include <gtest/gtest.h>
#include <m1_msgs/M1PtpCmd.h>
#include <m1_msgs/M1PtpCmdService.h>
#include <ros/ros.h>

m1_msgs::M1PtpCmd create_ptp_msg(float x, float y, float z)
{
  m1_msgs::M1PtpCmd msg;
  msg.ptpMode = 0;
  msg.x = x;
  msg.y = y;
  msg.z = z;
  return msg;
}

TEST(TestSuite, ActionServerEstablised)
{
  ros::NodeHandle nh;
  ros::ServiceClient ptp_sc = nh.serviceClient<m1_msgs::M1PtpCmdService>("ptp_cmd_validator");
  m1_msgs::M1PtpCmdService srv;

  srv.request.m1_ptp_cmd = create_ptp_msg(400.0, 0.0, 100.0);
  ptp_sc.call(srv);
  ASSERT_TRUE(srv.response.status.data);
}

TEST(TestSuite, OutOfRange)
{
  ros::NodeHandle nh;
  ros::ServiceClient ptp_sc = nh.serviceClient<m1_msgs::M1PtpCmdService>("ptp_cmd_validator");
  m1_msgs::M1PtpCmdService srv;
  srv.request.m1_ptp_cmd = create_ptp_msg(600.0, 0.0, 100.0); // invalid goal
  ptp_sc.call(srv);
  ASSERT_FALSE(srv.response.status.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unit_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
