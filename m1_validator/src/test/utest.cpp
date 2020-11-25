#include <actionlib/client/simple_action_client.h>
#include <gtest/gtest.h>
#include <m1_msgs/M1Ptp.h>
#include <m1_msgs/M1PtpAction.h>
#include <ros/ros.h>

m1_msgs::M1Ptp create_ptp_msg(float x, float y, float z) {
  m1_msgs::M1Ptp msg;
  msg.ptpMode = 0;
  msg.x = x;
  msg.y = y;
  msg.z = z;
  return msg;
}

TEST(TestSuite, ActionServerEstablised) {
  actionlib::SimpleActionClient<m1_msgs::M1PtpAction> ptp_ac(
      "/m1_ptp_validation", true);

  bool connection_state = false;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(1.0);
  while (ros::Time::now() - start_time < timeout) {
    if (ptp_ac.isServerConnected()) {
      connection_state = true;
      break;
    }
  }
  ASSERT_TRUE(connection_state);

  m1_msgs::M1PtpGoal goal;
  goal.id = 1;
  goal.m1_ptp = create_ptp_msg(400.0, 0.0, 100.0); // valid goal
  ptp_ac.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout = ptp_ac.waitForResult(ros::Duration(0.5));

  ASSERT_TRUE(finished_before_timeout);
  ASSERT_NE(ptp_ac.getState(), actionlib::SimpleClientGoalState::SUCCEEDED);

  m1_msgs::M1PtpResultConstPtr result = ptp_ac.getResult();
  ASSERT_EQ(result->id, 1);
  ASSERT_TRUE(result->status.data);
}

TEST(TestSuite, OutOfRange) {
  actionlib::SimpleActionClient<m1_msgs::M1PtpAction> ptp_ac(
      "m1_ptp_validation", true);

  bool connection_state = false;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(2.0); // Timeout of 2 seconds
  while (ros::Time::now() - start_time < timeout) {
    if (ptp_ac.isServerConnected()) {
      connection_state = true;
      break;
    }
  }
  ASSERT_TRUE(connection_state);

  m1_msgs::M1PtpGoal goal;
  goal.id = 1;
  goal.m1_ptp = create_ptp_msg(600.0, 0.0, 100.0); // invalid goal
  ptp_ac.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout = ptp_ac.waitForResult(ros::Duration(0.5));

  m1_msgs::M1PtpResultConstPtr result = ptp_ac.getResult();
  ASSERT_EQ(result->id, 1);
  ASSERT_FALSE(result->status.data);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "unit_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
