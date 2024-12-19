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

#include "gtest/gtest.h"

static auto GetTalosFilePath() noexcept -> std::filesystem::path {
  return std::filesystem::path(EXAMPLE_ROBOT_DATA_MODEL_DIR) / "talos_data" /
         "robots" / "talos_reduced.urdf";
}

TEST(LfControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LFController(); });
}

TEST(LfControllerTest, InitializeNullptr) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ ctrl.initialize(nullptr); });
}

TEST(LfControllerTest, InitializeEmptyModel) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ ctrl.initialize(std::make_shared<RobotModelBuilder>()); });
}

TEST(LfControllerTest, Initialize) {
  const auto talos_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = FileToString(GetTalosFilePath()),
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
