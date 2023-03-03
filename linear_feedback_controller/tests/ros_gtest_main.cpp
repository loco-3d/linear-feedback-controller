#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  std::string test_ros_node_name = std::string(TEST_ROS_NODE_NAME);
  ros::init(argc, argv, "tester");

  ros::NodeHandle nh;
  // Start the spinner here to be able to use callbacks.
  ros::AsyncSpinner spinner(4);  // Use 4 threads
  spinner.start();

  ::testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();

  ros::shutdown();

  return ret;
}
