#include <filesystem>
#include <system_error>

#include "utils/file_operation.hpp"
using tests::utils::FileOpen;
using tests::utils::FileToString;

#include "utils/robot_model.hpp"
using tests::utils::JointType;
using tests::utils::MakeBuilderFrom;

#include "linear_feedback_controller/robot_model_builder.hpp"
using linear_feedback_controller::RobotModelBuilder;

#include "linear_feedback_controller/lf_controller.hpp"
using linear_feedback_controller::LFController;

#include "linear_feedback_controller_msgs/eigen_conversions.hpp"
using linear_feedback_controller_msgs::Eigen::Control;
using linear_feedback_controller_msgs::Eigen::Sensor;

#include "example-robot-data/path.hpp"  // EXAMPLE_ROBOT_DATA_MODEL_DIR
#include "gtest/gtest.h"

struct LfControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  void TearDown() override {}

  static auto GetTalosUrdf() -> const std::string& {
    static const auto urdf =
        FileToString(std::filesystem::path(EXAMPLE_ROBOT_DATA_MODEL_DIR) /
                     "talos_data" / "robots" / "talos_reduced.urdf");
    return urdf;
  }
};

TEST_F(LfControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LFController(); });
}

TEST_F(LfControllerTest, InitializeNullptr) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ ctrl.initialize(nullptr); });
}

TEST_F(LfControllerTest, InitializeEmptyModel) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ ctrl.initialize(std::make_shared<RobotModelBuilder>()); });
}

TEST_F(LfControllerTest, Initialize) {
  const auto talos_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = GetTalosUrdf(),
          .joints =
              {
                  // TBD
                  {.name = "root_joint"},
                  {.name = "leg_left_1_joint", .type = JointType::Controlled},
                  {.name = "leg_left_2_joint", .type = JointType::Both},
                  // {.name = "leg_left_3_joint"},
                  // {.name = "leg_left_4_joint"},
                  // {.name = "leg_left_5_joint"},
                  // {.name = "leg_left_6_joint"},
                  // {.name = "leg_right_1_joint"},
                  // {.name = "leg_right_2_joint"},
                  // {.name = "leg_right_3_joint"},
                  // {.name = "leg_right_4_joint"},
                  // {.name = "leg_right_5_joint"},
                  // {.name = "leg_right_6_joint"},
                  // {.name = "torso_1_joint"},
                  // {.name = "torso_2_joint"},
                  // {.name = "arm_left_1_joint"},
                  // {.name = "arm_left_2_joint"},
                  // {.name = "arm_left_3_joint"},
                  // {.name = "arm_left_4_joint"},
                  // {.name = "arm_left_5_joint"},
                  // {.name = "arm_left_6_joint"},
                  // {.name = "arm_left_7_joint"},
                  // {.name = "arm_right_1_joint"},
                  // {.name = "arm_right_2_joint"},
                  // {.name = "arm_right_3_joint"},
                  // {.name = "arm_right_4_joint"},
                  // {.name = "arm_right_5_joint"},
                  // {.name = "arm_right_6_joint"},
                  // {.name = "arm_right_7_joint"},
              },
          .has_free_flyer = true,
      }),
  };
  ASSERT_NE(talos_model_ptr, nullptr);

  auto ctrl = LFController();
  EXPECT_NO_THROW({ ctrl.initialize(talos_model_ptr); });
}

TEST_F(LfControllerTest, ComputeControlNotInitialized) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ auto _ = ctrl.compute_control({}, {}); });
}

TEST_F(LfControllerTest, ComputeControlNoInput) {
  const auto talos_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = GetTalosUrdf(),
          .joints =
              {
                  // TBD
                  {.name = "root_joint"},        {.name = "leg_left_1_joint"},
                  {.name = "leg_left_2_joint"},  {.name = "leg_left_3_joint"},
                  {.name = "leg_left_4_joint"},  {.name = "leg_left_5_joint"},
                  {.name = "leg_left_6_joint"},  {.name = "leg_right_1_joint"},
                  {.name = "leg_right_2_joint"}, {.name = "leg_right_3_joint"},
                  {.name = "leg_right_4_joint"}, {.name = "leg_right_5_joint"},
                  {.name = "leg_right_6_joint"}, {.name = "torso_1_joint"},
                  {.name = "torso_2_joint"},     {.name = "arm_left_1_joint"},
                  {.name = "arm_left_2_joint"},  {.name = "arm_left_3_joint"},
                  {.name = "arm_left_4_joint"},  {.name = "arm_left_5_joint"},
                  {.name = "arm_left_6_joint"},  {.name = "arm_left_7_joint"},
                  {.name = "arm_right_1_joint"}, {.name = "arm_right_2_joint"},
                  {.name = "arm_right_3_joint"}, {.name = "arm_right_4_joint"},
                  {.name = "arm_right_5_joint"}, {.name = "arm_right_6_joint"},
                  {.name = "arm_right_7_joint"},
              },
          .has_free_flyer = true,
      }),
  };
  ASSERT_NE(talos_model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(talos_model_ptr);
  EXPECT_ANY_THROW({ auto _ = ctrl.compute_control({}, {}); });
}

TEST_F(LfControllerTest, ComputeControlUnknownJoints) {
  const auto talos_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = GetTalosUrdf(),
          .joints =
              {
                  {.name = "root_joint"},
                  {.name = "leg_left_1_joint"},
              },
          .has_free_flyer = true,
      }),
  };
  ASSERT_NE(talos_model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(talos_model_ptr);

  // TODO: Create function arguments with an unknown joint name
  //  (not within the defined model)
  EXPECT_ANY_THROW({
    auto _ = ctrl.compute_control(
        Sensor{
            // TODO
        },
        Control{
            // TODO
        });
  });
}

TEST_F(LfControllerTest, ComputeControlSizeMismatch) {
  const auto talos_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = GetTalosUrdf(),
          .joints =
              {
                  {.name = "root_joint"},
                  {.name = "leg_left_1_joint"},
              },
          .has_free_flyer = true,
      }),
  };
  ASSERT_NE(talos_model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(talos_model_ptr);

  // TODO: Create function arguments with an matrices/vectors size mismatch ?
  //  (not within the defined model)
  EXPECT_ANY_THROW({
    auto _ = ctrl.compute_control(
        Sensor{
            // TODO
        },
        Control{
            // TODO
        });
  });
}

TEST_F(LfControllerTest, ComputeControlValid) {
  const auto talos_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = GetTalosUrdf(),
          .joints =
              {
                  {.name = "root_joint"},
                  {.name = "leg_left_1_joint"},
              },
          .has_free_flyer = true,
      }),
  };
  ASSERT_NE(talos_model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(talos_model_ptr);

  // TODO: How to test it worked ?
  EXPECT_EQ(
      Eigen::VectorXd{
          // TODO
      },
      ctrl.compute_control(
          Sensor{
              // TODO
          },
          Control{
              // TODO
          }));
}
