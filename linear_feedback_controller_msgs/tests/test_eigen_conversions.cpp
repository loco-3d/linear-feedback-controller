#include <gtest/gtest.h>
#include "linear_feedback_controller_msgs/Sensor.h"
#include "linear_feedback_controller_msgs/Control.h"
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"

namespace lfc_msgs = linear_feedback_controller_msgs;

class LinearFeedbackControllerMsgsTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  void TearDown() override {}
};
class DISABLED_LinearFeedbackControllerMsgsTest : public LinearFeedbackControllerMsgsTest {};

TEST_F(LinearFeedbackControllerMsgsTest, checkEigenConstructors) {
  lfc_msgs::Eigen::Sensor s;
  lfc_msgs::Eigen::Control c;
}

TEST_F(LinearFeedbackControllerMsgsTest, checkRosConstructors) {
  lfc_msgs::Sensor s;
  lfc_msgs::Control c;
}

TEST_F(LinearFeedbackControllerMsgsTest, checkRosEigenMatrixConversion) {
  Eigen::MatrixXd eigen_mat = Eigen::MatrixXd::Random(5, 6);
  std_msgs::Float64MultiArray ros_mat;
  Eigen::MatrixXd eigen_mat_test;

  lfc_msgs::matrixEigenToMsg(eigen_mat, ros_mat);
  lfc_msgs::matrixMsgToEigen(ros_mat, eigen_mat_test);

  ASSERT_EQ(eigen_mat, eigen_mat_test);
}

TEST_F(LinearFeedbackControllerMsgsTest, checkRosEigenJointStateConversion) {
  lfc_msgs::Eigen::JointState eigen_joint_state;
  sensor_msgs::JointState ros_joint_state;
  lfc_msgs::Eigen::JointState eigen_joint_state_test;

  eigen_joint_state.name = {"1", "2", "3", "4", "5", "6"};
  eigen_joint_state.position = Eigen::VectorXd::Random(eigen_joint_state.name.size());
  eigen_joint_state.velocity = Eigen::VectorXd::Random(eigen_joint_state.name.size());
  eigen_joint_state.effort = Eigen::VectorXd::Random(eigen_joint_state.name.size());

  lfc_msgs::jointStateEigenToMsg(eigen_joint_state, ros_joint_state);
  lfc_msgs::jointStateMsgToEigen(ros_joint_state, eigen_joint_state_test);

  ASSERT_EQ(eigen_joint_state.name, eigen_joint_state_test.name);
  ASSERT_EQ(eigen_joint_state.position, eigen_joint_state_test.position);
  ASSERT_EQ(eigen_joint_state.velocity, eigen_joint_state_test.velocity);
  ASSERT_EQ(eigen_joint_state.effort, eigen_joint_state_test.effort);
}

TEST_F(LinearFeedbackControllerMsgsTest, checkRosEigenSensorConversion) {
  lfc_msgs::Eigen::Sensor e;
  lfc_msgs::Sensor m;
  lfc_msgs::Eigen::Sensor etest;

  e.base_pose = Eigen::Matrix<double, 7, 1>::Random();
  Eigen::Quaterniond q;
  q.coeffs() = Eigen::Vector4d::Random();
  q.normalize();
  e.base_pose.tail<4>() = q.coeffs();
  e.base_twist = Eigen::Matrix<double, 6, 1>::Random();
  e.joint_state.name = {"1", "2", "3", "4", "5", "6"};
  e.joint_state.position = Eigen::VectorXd::Random(e.joint_state.name.size());
  e.joint_state.velocity = Eigen::VectorXd::Random(e.joint_state.name.size());
  e.joint_state.effort = Eigen::VectorXd::Random(e.joint_state.name.size());

  lfc_msgs::sensorEigenToMsg(e, m);
  lfc_msgs::sensorMsgToEigen(m, etest);

  ASSERT_TRUE(e.base_pose.isApprox(etest.base_pose));
  ASSERT_EQ(e.base_twist, etest.base_twist);
  ASSERT_EQ(e.joint_state.name, etest.joint_state.name);
  ASSERT_EQ(e.joint_state.position, etest.joint_state.position);
  ASSERT_EQ(e.joint_state.velocity, etest.joint_state.velocity);
  ASSERT_EQ(e.joint_state.effort, etest.joint_state.effort);
}

TEST_F(LinearFeedbackControllerMsgsTest, checkRosEigenControlConversion) {
  lfc_msgs::Eigen::Control e;
  lfc_msgs::Control m;
  lfc_msgs::Eigen::Control etest;

  e.initial_state.base_pose = Eigen::Matrix<double, 7, 1>::Random();
  Eigen::Quaterniond q;
  q.coeffs() = Eigen::Vector4d::Random();
  q.normalize();
  e.initial_state.base_pose.tail<4>() = q.coeffs();
  e.initial_state.base_twist = Eigen::Matrix<double, 6, 1>::Random();
  e.initial_state.joint_state.name = {"1", "2", "3", "4", "5", "6"};
  e.initial_state.joint_state.position = Eigen::VectorXd::Random(e.initial_state.joint_state.name.size());
  e.initial_state.joint_state.velocity = Eigen::VectorXd::Random(e.initial_state.joint_state.name.size());
  e.initial_state.joint_state.effort = Eigen::VectorXd::Random(e.initial_state.joint_state.name.size());
  e.feedback_gain = Eigen::MatrixXd::Random(8, 4);

  lfc_msgs::controlEigenToMsg(e, m);
  lfc_msgs::controlMsgToEigen(m, etest);

  ASSERT_TRUE(e.initial_state.base_pose.isApprox(etest.initial_state.base_pose));
  ASSERT_EQ(e.initial_state.base_twist, etest.initial_state.base_twist);
  ASSERT_EQ(e.initial_state.joint_state.name, etest.initial_state.joint_state.name);
  ASSERT_EQ(e.initial_state.joint_state.position, etest.initial_state.joint_state.position);
  ASSERT_EQ(e.initial_state.joint_state.velocity, etest.initial_state.joint_state.velocity);
  ASSERT_EQ(e.initial_state.joint_state.effort, etest.initial_state.joint_state.effort);
  ASSERT_EQ(e.feedback_gain, etest.feedback_gain);
}
