#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <example-robot-data/path.hpp>
#include <ros-wbmpc/wbmpc.hpp>

using namespace ros_wbmpc;

class RosWBMPCTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string urdf_file = std::string(EXAMPLE_ROBOT_DATA_MODEL_DIR) + "/talos_data/robots/talos_reduced.urdf";
    std::string srdf_file = std::string(EXAMPLE_ROBOT_DATA_MODEL_DIR) + "/talos_data/srdf/talos.srdf";
    std::ifstream urdf_stream(urdf_file);
    std::stringstream urdf;
    urdf << urdf_stream.rdbuf();
    std::ifstream srdf_stream(srdf_file);
    std::stringstream srdf;
    srdf << srdf_stream.rdbuf();

    // clang-format off
    // controlled_joint_names
    sorted_controlled_joint_names_ = {
        "root_joint",
        "leg_left_1_joint",
        "leg_left_2_joint",
        "leg_left_3_joint",
        "leg_left_4_joint",
        "leg_left_5_joint",
        "leg_left_6_joint",
        "leg_right_1_joint",
        "leg_right_2_joint",
        "leg_right_3_joint",
        "leg_right_4_joint",
        "leg_right_5_joint",
        "leg_right_6_joint",
        "torso_1_joint",
        "torso_2_joint",
    };
    mixed_controlled_joint_names_ = {
        "leg_right_1_joint",
        "leg_right_2_joint",
        "leg_right_3_joint",
        "leg_right_4_joint",
        "leg_right_5_joint",
        "leg_right_6_joint",
        "torso_1_joint",
        "torso_2_joint",
        "leg_left_1_joint",
        "leg_left_2_joint",
        "leg_left_3_joint",
        "leg_left_4_joint",
        "leg_left_5_joint",
        "leg_left_6_joint",
        "root_joint",
    };
    wrong_controlled_joint_names_ = {
        "root_joint",
        "leg_left_1_joint",
        "leg_left_2_joint",
        "leg_left_3_joint",
        "leg_left_4_joint",
        "leg_left_5_joint",
        "leg_left_6_joint",
        "leg_right_1_joint",
        "leg_right_2_joint",
        "leg_right_3_joint",
        "leg_right_4_joint",
        "leg_right_5_joint",
        "leg_right_6_joint",
        "torso_1_joint",
        "banana",
    };
    duplicate_controlled_joint_names_ = {
      "root_joint", "root_joint",
      "leg_left_1_joint", "leg_left_1_joint",
      "leg_left_2_joint", "leg_left_2_joint",
      "leg_left_3_joint", "leg_left_3_joint",
      "leg_left_4_joint", "leg_left_4_joint",
      "leg_left_5_joint", "leg_left_5_joint",
      "leg_left_6_joint", "leg_left_6_joint",
      "leg_right_1_joint", "leg_right_1_joint",
      "leg_right_2_joint", "leg_right_2_joint",
      "leg_right_3_joint", "leg_right_3_joint",
      "leg_right_4_joint", "leg_right_4_joint",
      "leg_right_5_joint", "leg_right_5_joint",
      "leg_right_6_joint", "leg_right_6_joint",
      "torso_1_joint", "torso_1_joint",
      "torso_2_joint", "torso_2_joint",
    };
    sorted_controlled_joint_ids_ =
      { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
    sorted_locked_joint_ids_ =
      { 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33 };
    // clang-format on
    bool robot_has_free_flyer = true;

    // Get the parameters of the node.
    nh_.setParam("robot_description", urdf.str());
    nh_.setParam("robot_description_semantic", srdf.str());
    nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
    nh_.setParam("robot_has_free_flyer", robot_has_free_flyer);
  }

  void TearDown() override {}

  ros::NodeHandle nh_;
  std::vector<long unsigned int> sorted_controlled_joint_ids_;
  std::vector<long unsigned int> sorted_locked_joint_ids_;
  std::vector<std::string> sorted_controlled_joint_names_;
  std::vector<std::string> mixed_controlled_joint_names_;
  std::vector<std::string> wrong_controlled_joint_names_;
  std::vector<std::string> duplicate_controlled_joint_names_;
};
class DISABLED_RosWBMPCTest : public RosWBMPCTest {};

TEST_F(RosWBMPCTest, checkConstructor) { RosWBMPC obj; }

TEST_F(RosWBMPCTest, checkInitializeNoThrow) {
  RosWBMPC obj;
  obj.initialize(nh_);
}

TEST_F(RosWBMPCTest, checkControlledJointNamesSorted) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
  obj.initialize(nh_);
  GTEST_ASSERT_EQ(obj.get_controlled_joint_names(), sorted_controlled_joint_names_);
}

TEST_F(RosWBMPCTest, checkControlledJointNamesMixed) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", mixed_controlled_joint_names_);
  obj.initialize(nh_);
  GTEST_ASSERT_EQ(obj.get_controlled_joint_names(), sorted_controlled_joint_names_);
}

TEST_F(RosWBMPCTest, checkControlledJointNamesWrong) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", wrong_controlled_joint_names_);
  obj.initialize(nh_);
  sorted_controlled_joint_names_.pop_back();
  GTEST_ASSERT_EQ(obj.get_controlled_joint_names(), sorted_controlled_joint_names_);
}

TEST_F(RosWBMPCTest, checkControlledJointNamesDuplicate) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", duplicate_controlled_joint_names_);
  obj.initialize(nh_);
  GTEST_ASSERT_EQ(obj.get_controlled_joint_names(), sorted_controlled_joint_names_);
}

TEST_F(RosWBMPCTest, checkControlledJointIdsSorted) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
  obj.initialize(nh_);

  std::vector<long unsigned int> controlled_joint_ids = obj.get_controlled_joint_ids();
  for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
    GTEST_ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
  }
  GTEST_ASSERT_EQ(obj.get_controlled_joint_ids(), sorted_controlled_joint_ids_);
}

TEST_F(RosWBMPCTest, checkControlledJointIdsMixed) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", mixed_controlled_joint_names_);
  obj.initialize(nh_);

  std::vector<long unsigned int> controlled_joint_ids = obj.get_controlled_joint_ids();
  for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
    GTEST_ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
  }
  GTEST_ASSERT_EQ(obj.get_controlled_joint_ids(), sorted_controlled_joint_ids_);
}

TEST_F(RosWBMPCTest, checkControlledJointIdsWrong) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", wrong_controlled_joint_names_);
  obj.initialize(nh_);
  sorted_controlled_joint_names_.pop_back();

  std::vector<long unsigned int> controlled_joint_ids = obj.get_controlled_joint_ids();
  for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
    GTEST_ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
  }
  sorted_controlled_joint_ids_.pop_back();
  GTEST_ASSERT_EQ(obj.get_controlled_joint_ids(), sorted_controlled_joint_ids_);
}

TEST_F(RosWBMPCTest, checkControlledJointIdsDuplicate) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", duplicate_controlled_joint_names_);
  obj.initialize(nh_);

  std::vector<long unsigned int> controlled_joint_ids = obj.get_controlled_joint_ids();
  for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
    GTEST_ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
  }
  GTEST_ASSERT_EQ(obj.get_controlled_joint_ids(), sorted_controlled_joint_ids_);
}

TEST_F(RosWBMPCTest, checkLockedJointIdsSorted) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
  obj.initialize(nh_);

  std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    GTEST_ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  GTEST_ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
}

TEST_F(RosWBMPCTest, checkLockedJointIdsMixed) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", mixed_controlled_joint_names_);
  obj.initialize(nh_);

  std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    GTEST_ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  GTEST_ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
}

TEST_F(RosWBMPCTest, checkLockedJointIdsWrong) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", wrong_controlled_joint_names_);
  obj.initialize(nh_);
  sorted_controlled_joint_names_.pop_back();

  std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    GTEST_ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  sorted_locked_joint_ids_.insert(sorted_locked_joint_ids_.begin(), sorted_controlled_joint_ids_.back());
  GTEST_ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
}

TEST_F(RosWBMPCTest, checkLockedJointIdsDuplicate) {
  RosWBMPC obj;
  nh_.setParam("controlled_joint_names", duplicate_controlled_joint_names_);
  obj.initialize(nh_);

  std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    GTEST_ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  GTEST_ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
}
