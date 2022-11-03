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


// TEST_F(LinearFeedbackControllerMsgsTest, checkInitializeNoThrow) {
//   LinearFeedbackController obj;
//   GTEST_ASSERT_TRUE(obj.loadEtras(nh_));
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkControlledJointNamesSorted) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   GTEST_ASSERT_EQ(obj.get_controlled_joint_names(), sorted_controlled_joint_names_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkControlledJointNamesMixed) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", mixed_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   GTEST_ASSERT_EQ(obj.get_controlled_joint_names(), sorted_controlled_joint_names_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkControlledJointNamesWrong) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", wrong_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   sorted_controlled_joint_names_.pop_back();
//   GTEST_ASSERT_EQ(obj.get_controlled_joint_names(), sorted_controlled_joint_names_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkControlledJointNamesDuplicate) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", duplicate_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   GTEST_ASSERT_EQ(obj.get_controlled_joint_names(), sorted_controlled_joint_names_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkControlledJointIdsSorted) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> controlled_joint_ids = obj.get_controlled_joint_ids();
//   for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
//     GTEST_ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
//   }
//   GTEST_ASSERT_EQ(obj.get_controlled_joint_ids(), sorted_controlled_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkControlledJointIdsMixed) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", mixed_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> controlled_joint_ids = obj.get_controlled_joint_ids();
//   for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
//     GTEST_ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
//   }
//   GTEST_ASSERT_EQ(obj.get_controlled_joint_ids(), sorted_controlled_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkControlledJointIdsWrong) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", wrong_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   sorted_controlled_joint_names_.pop_back();

//   std::vector<long unsigned int> controlled_joint_ids = obj.get_controlled_joint_ids();
//   for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
//     GTEST_ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
//   }
//   sorted_controlled_joint_ids_.pop_back();
//   GTEST_ASSERT_EQ(obj.get_controlled_joint_ids(), sorted_controlled_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkControlledJointIdsDuplicate) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", duplicate_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> controlled_joint_ids = obj.get_controlled_joint_ids();
//   for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
//     GTEST_ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
//   }
//   GTEST_ASSERT_EQ(obj.get_controlled_joint_ids(), sorted_controlled_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkLockedJointIdsSorted) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
//   for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
//     GTEST_ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
//   }
//   GTEST_ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkLockedJointIdsMixed) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", mixed_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
//   for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
//     GTEST_ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
//   }
//   GTEST_ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkLockedJointIdsWrong) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", wrong_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   sorted_controlled_joint_names_.pop_back();

//   std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
//   for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
//     GTEST_ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
//   }
//   sorted_locked_joint_ids_.insert(sorted_locked_joint_ids_.begin(), sorted_controlled_joint_ids_.back());
//   GTEST_ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerMsgsTest, checkLockedJointIdsDuplicate) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", duplicate_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
//   for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
//     GTEST_ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
//   }
//   GTEST_ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
// }
