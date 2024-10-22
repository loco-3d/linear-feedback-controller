#include <gtest/gtest.h>

#include <fstream>

#include "linear_feedback_controller/robot_model_builder.hpp"

using namespace linear_feedback_controller;

class RobotModelBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string talos_urdf_path = std::string(EXAMPLE_ROBOT_DATA_MODEL_DIR) +
                                  "/talos_data/robots/talos_reduced.urdf";
    std::string talos_srdf_path = std::string(EXAMPLE_ROBOT_DATA_MODEL_DIR) +
                                  "/talos_data/srdf/talos.srdf";

    talos_urdf_ = ReadFile(talos_urdf_path);
    talos_srdf_ = ReadFile(talos_srdf_path);

    default_configuration_name_ = "half_sitting";

    // moving_joint_names
    sorted_moving_joint_names_ = {
        "root_joint",        "leg_left_1_joint",  "leg_left_2_joint",
        "leg_left_3_joint",  "leg_left_4_joint",  "leg_left_5_joint",
        "leg_left_6_joint",  "leg_right_1_joint", "leg_right_2_joint",
        "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint",
        "leg_right_6_joint", "torso_1_joint",     "torso_2_joint",
    };
    // remove the root_joint which is an artifact of Pinocchio.
    test_sorted_moving_joint_names_ = {
        "leg_left_1_joint",  "leg_left_2_joint",  "leg_left_3_joint",
        "leg_left_4_joint",  "leg_left_5_joint",  "leg_left_6_joint",
        "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint",
        "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
        "torso_1_joint",     "torso_2_joint",
    };
    mixed_moving_joint_names_ = {
        "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint",
        "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
        "torso_1_joint",     "torso_2_joint",     "leg_left_1_joint",
        "leg_left_2_joint",  "leg_left_3_joint",  "leg_left_4_joint",
        "leg_left_5_joint",  "leg_left_6_joint",  "root_joint",
    };
    wrong_moving_joint_names_ = {
        "root_joint",        "leg_left_1_joint",  "leg_left_2_joint",
        "leg_left_3_joint",  "leg_left_4_joint",  "leg_left_5_joint",
        "leg_left_6_joint",  "leg_right_1_joint", "leg_right_2_joint",
        "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint",
        "leg_right_6_joint", "torso_1_joint",     "banana",
    };
    duplicate_moving_joint_names_ = {
        "root_joint",        "root_joint",        "leg_left_1_joint",
        "leg_left_1_joint",  "leg_left_2_joint",  "leg_left_2_joint",
        "leg_left_3_joint",  "leg_left_3_joint",  "leg_left_4_joint",
        "leg_left_4_joint",  "leg_left_5_joint",  "leg_left_5_joint",
        "leg_left_6_joint",  "leg_left_6_joint",  "leg_right_1_joint",
        "leg_right_1_joint", "leg_right_2_joint", "leg_right_2_joint",
        "leg_right_3_joint", "leg_right_3_joint", "leg_right_4_joint",
        "leg_right_4_joint", "leg_right_5_joint", "leg_right_5_joint",
        "leg_right_6_joint", "leg_right_6_joint", "torso_1_joint",
        "torso_1_joint",     "torso_2_joint",     "torso_2_joint",
    };
    controlled_joint_names_ = {
        {"root_joint",        "leg_left_1_joint",  "leg_left_2_joint",
         "leg_left_3_joint",  "leg_left_4_joint",  "leg_left_5_joint",
         "leg_left_6_joint",  "leg_right_1_joint", "leg_right_2_joint",
         "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint",
         "leg_right_6_joint", "torso_1_joint",     "torso_2_joint",
         "arm_left_1_joint",  "arm_left_2_joint",  "arm_left_3_joint",
         "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint",
         "arm_left_7_joint",  "arm_right_1_joint", "arm_right_2_joint",
         "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint",
         "arm_right_6_joint", "arm_right_7_joint"}};
    sorted_moving_joint_ids_ = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    sorted_locked_joint_ids_ = {16, 17, 18, 19, 20, 21, 22, 23, 24,
                                25, 26, 27, 28, 29, 30, 31, 32, 33};
    has_free_flyer_ = true;
  }

  void TearDown() override {}

  std::string ReadFile(std::string filePath) {
    // Open the file using ifstream.
    std::ifstream file(filePath);
    // Confirm file opening.
    if (!file.is_open()) {
      // print error message and return
      throw std::runtime_error("Failed to open file: " + filePath);
    }
    // Read file.
    std::stringstream buffer;
    buffer << file.rdbuf();
    // Close the file
    file.close();
    // Return the string.
    return buffer.str();
  }

  std::string talos_urdf_;
  std::string talos_srdf_;
  std::string default_configuration_name_;
  std::vector<std::string> moving_joint_names_;
  bool has_free_flyer_;
  std::vector<std::string> controlled_joint_names_;
  std::vector<std::string> sorted_moving_joint_names_;
  std::vector<std::string> test_sorted_moving_joint_names_;
  std::vector<std::string> mixed_moving_joint_names_;
  std::vector<std::string> wrong_moving_joint_names_;
  std::vector<std::string> duplicate_moving_joint_names_;
  std::vector<long unsigned int> sorted_moving_joint_ids_;
  std::vector<long unsigned int> sorted_locked_joint_ids_;
};
class DISABLED_MinJerkTest : public RobotModelBuilderTest {};

TEST_F(RobotModelBuilderTest, checkConstructor) { RobotModelBuilder obj; }

TEST_F(RobotModelBuilderTest, checkMovingJointNames) {
  RobotModelBuilder obj;

  obj.build_model(talos_urdf_, talos_srdf_, sorted_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);

  ASSERT_EQ(obj.get_moving_joint_names(), test_sorted_moving_joint_names_);
}

TEST_F(RobotModelBuilderTest, checkMovingJointNamesMixed) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, mixed_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);
  ASSERT_EQ(obj.get_moving_joint_names(), test_sorted_moving_joint_names_);
}

TEST_F(RobotModelBuilderTest, checkMovingJointNamesWrong) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, wrong_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);
  test_sorted_moving_joint_names_.pop_back();
  ASSERT_EQ(obj.get_moving_joint_names(), test_sorted_moving_joint_names_);
}

TEST_F(RobotModelBuilderTest, checkMovingJointNamesDuplicate) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, duplicate_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);
  ASSERT_EQ(obj.get_moving_joint_names(), test_sorted_moving_joint_names_);
}

TEST_F(RobotModelBuilderTest, checkMovingJointIdsSorted) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, sorted_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);

  std::vector<long unsigned int> moving_joint_ids = obj.get_moving_joint_ids();
  for (std::size_t i = 1; i < moving_joint_ids.size(); ++i) {
    ASSERT_LE(moving_joint_ids[i - 1], moving_joint_ids[i]);
  }
  ASSERT_EQ(obj.get_moving_joint_ids(), sorted_moving_joint_ids_);
}

TEST_F(RobotModelBuilderTest, checkMovingJointIdsMixed) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, mixed_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);

  std::vector<long unsigned int> moving_joint_ids = obj.get_moving_joint_ids();
  for (std::size_t i = 1; i < moving_joint_ids.size(); ++i) {
    ASSERT_LE(moving_joint_ids[i - 1], moving_joint_ids[i]);
  }
  ASSERT_EQ(obj.get_moving_joint_ids(), sorted_moving_joint_ids_);
}

TEST_F(RobotModelBuilderTest, checkMovingJointIdsWrong) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, wrong_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);
  sorted_moving_joint_names_.pop_back();

  std::vector<long unsigned int> moving_joint_ids = obj.get_moving_joint_ids();
  for (std::size_t i = 1; i < moving_joint_ids.size(); ++i) {
    ASSERT_LE(moving_joint_ids[i - 1], moving_joint_ids[i]);
  }
  sorted_moving_joint_ids_.pop_back();
  ASSERT_EQ(obj.get_moving_joint_ids(), sorted_moving_joint_ids_);
}

TEST_F(RobotModelBuilderTest, checkMovingJointIdsDuplicate) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, duplicate_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);

  std::vector<long unsigned int> moving_joint_ids = obj.get_moving_joint_ids();
  for (std::size_t i = 1; i < moving_joint_ids.size(); ++i) {
    ASSERT_LE(moving_joint_ids[i - 1], moving_joint_ids[i]);
  }
  ASSERT_EQ(obj.get_moving_joint_ids(), sorted_moving_joint_ids_);
}

TEST_F(RobotModelBuilderTest, checkLockedJointIdsSorted) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, sorted_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);

  std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
}

TEST_F(RobotModelBuilderTest, checkLockedJointIdsMixed) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, mixed_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);

  std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
}

TEST_F(RobotModelBuilderTest, checkLockedJointIdsWrong) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, wrong_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);
  sorted_moving_joint_names_.pop_back();

  std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  sorted_locked_joint_ids_.insert(sorted_locked_joint_ids_.begin(),
                                  sorted_moving_joint_ids_.back());
  ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
}

TEST_F(RobotModelBuilderTest, checkLockedJointIdsDuplicate) {
  RobotModelBuilder obj;
  obj.build_model(talos_urdf_, talos_srdf_, duplicate_moving_joint_names_,
                  controlled_joint_names_, default_configuration_name_,
                  has_free_flyer_);

  std::vector<long unsigned int> locked_joint_ids = obj.get_locked_joint_ids();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  ASSERT_EQ(obj.get_locked_joint_ids(), sorted_locked_joint_ids_);
}

TEST_F(RobotModelBuilderTest, checkSharedPtr) {
  RobotModelBuilder::SharedPtr obj = std::make_shared<RobotModelBuilder>();
  obj->build_model(talos_urdf_, talos_srdf_, duplicate_moving_joint_names_,
                   controlled_joint_names_, default_configuration_name_,
                   has_free_flyer_);

  std::vector<long unsigned int> locked_joint_ids = obj->get_locked_joint_ids();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  ASSERT_EQ(obj->get_locked_joint_ids(), sorted_locked_joint_ids_);
}
