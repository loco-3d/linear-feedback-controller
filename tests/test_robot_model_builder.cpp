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

    moving_joint_names_ = {{
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
    }};
    has_free_flyer_ = true;
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
  std::vector<std::string> moving_joint_names_;
  bool has_free_flyer_;
  std::vector<std::string> controlled_joint_names_;
};
class DISABLED_MinJerkTest : public RobotModelBuilderTest {};

TEST_F(RobotModelBuilderTest, checkConstructor) { RobotModelBuilder obj; }

TEST_F(RobotModelBuilderTest, checkBuildModel) {
  RobotModelBuilder obj;

  obj.build_model(talos_urdf_, talos_srdf_, moving_joint_names_,
                  has_free_flyer_, controlled_joint_names_);

  // obj.setParameters(end_time_, start_pos_,
  //                   /*start_speed*/ 0.0,
  //                   /*start_acc*/ 0.0, end_pos_,
  //                   /*end_speed*/ 0.0,
  //                   /*end_acc*/ 0.0);

  // // Test below min.
  // for (double t = start_time_ - range_time_; t < start_time_; t += dt_) {
  //   ASSERT_NEAR(obj.compute(t), start_pos_, 1e-8);
  //   ASSERT_NEAR(obj.computeDerivative(t), 0.0, 1e-8);
  //   ASSERT_NEAR(obj.computeDerivative(t), 0.0, 1e-8);
  //   ASSERT_NEAR(obj.computeSecDerivative(t), 0.0, 1e-8);
  //   ASSERT_NEAR(obj.computeSecDerivative(t), 0.0, 1e-8);
  // }
  // // Test below max.
  // for (double t = end_time_; t < end_time_ + range_time_; t += dt_) {
  //   ASSERT_NEAR(obj.compute(t), end_pos_, 1e-8);
  //   ASSERT_NEAR(obj.computeDerivative(t), 0.0, 1e-8);
  //   ASSERT_NEAR(obj.computeDerivative(t), 0.0, 1e-8);
  //   ASSERT_NEAR(obj.computeSecDerivative(t), 0.0, 1e-8);
  //   ASSERT_NEAR(obj.computeSecDerivative(t), 0.0, 1e-8);
  // }
  // // Test extremities
  // ASSERT_NEAR(obj.compute(start_time_), start_pos_, 1e-8);
  // ASSERT_NEAR(obj.compute(end_time_), end_pos_, 1e-8);
  // ASSERT_NEAR(obj.computeDerivative(start_time_), 0.0, 1e-8);
  // ASSERT_NEAR(obj.computeDerivative(end_time_), 0.0, 1e-8);
  // ASSERT_NEAR(obj.computeSecDerivative(start_time_), 0.0, 1e-8);
  // ASSERT_NEAR(obj.computeSecDerivative(end_time_), 0.0, 1e-8);
}
