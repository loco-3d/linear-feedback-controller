#include <string_view>

#include "utils/robot_model.hpp"
using tests::utils::JointType;
using tests::utils::MakeBuilderFrom;

#include "utils/eigen_conversions.hpp"
using tests::utils::MakeRandomControlForJoints;
using tests::utils::MakeRandomSensorForJoints;

#include "linear_feedback_controller/robot_model_builder.hpp"
using linear_feedback_controller::RobotModelBuilder;

#include "linear_feedback_controller/lf_controller.hpp"
using linear_feedback_controller::LFController;

#include "gtest/gtest.h"

using namespace std::literals::string_view_literals;

constexpr auto dummy_urdf =
    "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
    "<robot name=\"test\">"
    "  <link name=\"link_0\"/>"
    "  "
    "  <joint name=\"l0_to_l1\" type=\"revolute\">"
    "    <parent link=\"link_0\"/>"
    "    <child link=\"link_1\"/>"
    "    <origin xyz=\"0 0 1\" rpy=\"0 0 1\"/>"
    "    <axis xyz=\"0 0 1\"/>"
    "    <limit lower=\"0\" upper=\"3.14\" velocity=\"100\" effort=\"100\"/>"
    "  </joint>"
    "  "
    "  <link name=\"link_1\"/>"
    "  "
    "  <joint name=\"l1_to_l2\" type=\"revolute\">"
    "    <parent link=\"link_1\"/>"
    "    <child link=\"link_2\"/>"
    "    <origin xyz=\"0 1 0\" rpy=\"1 0 0\"/>"
    "    <axis xyz=\"0 1 0\"/>"
    "    <limit lower=\"-3.14\" upper=\"3.14\" velocity=\"100\" effort=\"10\"/>"
    "  </joint>"
    "  "
    "  <link name=\"link_2\"/>"
    "</robot>"sv;

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
  const auto dummy_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = std::string{dummy_urdf},
          .joints =
              {
                  {.name = "l0_to_l1"},
                  {.name = "l1_to_l2"},
              },
          .has_free_flyer = false,
      }),
  };
  ASSERT_NE(dummy_model_ptr, nullptr);

  auto ctrl = LFController();
  EXPECT_NO_THROW({ ctrl.initialize(dummy_model_ptr); });
}

TEST(LfControllerTest, ComputeControlNotInitialized) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ auto _ = ctrl.compute_control({}, {}); });
}

TEST(LfControllerTest, ComputeControlNoInput) {
  const auto dummy_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = std::string{dummy_urdf},
          .joints =
              {
                  {.name = "l0_to_l1"},
                  {.name = "l1_to_l2"},
              },
          .has_free_flyer = false,
      }),
  };
  ASSERT_NE(dummy_model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(dummy_model_ptr);
  EXPECT_ANY_THROW({ auto _ = ctrl.compute_control({}, {}); });
}

TEST(LfControllerTest, ComputeControlUnknownJoints) {
  const auto dummy_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = std::string{dummy_urdf},
          .joints =
              {
                  {.name = "l0_to_l1"},
                  {.name = "l1_to_l2"},
              },
          .has_free_flyer = false,
      }),
  };
  ASSERT_NE(dummy_model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(dummy_model_ptr);

  const auto name_begin =
      std::cbegin(dummy_model_ptr->get_moving_joint_names());
  const auto name_end = std::cend(dummy_model_ptr->get_moving_joint_names());

  auto sensor = MakeRandomSensorForJoints(name_begin, name_end);
  const auto control = MakeRandomControlForJoints(name_begin, name_end);

  EXPECT_ANY_THROW({
    sensor.joint_state.name[0] = "this joint doesn't exist";
    auto _ = ctrl.compute_control(sensor, control);
  });
}

TEST(LfControllerTest, ComputeControlSizeMismatch) {
  const auto dummy_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = std::string{dummy_urdf},
          .joints =
              {
                  {.name = "l0_to_l1"},
                  {.name = "l1_to_l2"},
              },
          .has_free_flyer = false,
      }),
  };
  ASSERT_NE(dummy_model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(dummy_model_ptr);

  // TODO: Create function arguments with an matrices/vectors size mismatch ?
  //  (not within the defined model)
  EXPECT_ANY_THROW({
    auto _ = ctrl.compute_control(
        {
            // TODO
        },
        {
            // TODO
        });
  });
}

TEST(LfControllerTest, ComputeControl) {
  const auto dummy_model_ptr = std::shared_ptr{
      MakeBuilderFrom({
          .urdf = std::string{dummy_urdf},
          .joints =
              {
                  {.name = "l0_to_l1"},
                  {.name = "l1_to_l2"},
              },
          .has_free_flyer = false,
      }),
  };
  ASSERT_NE(dummy_model_ptr, nullptr);

  const auto name_begin =
      std::cbegin(dummy_model_ptr->get_moving_joint_names());
  const auto name_end = std::cend(dummy_model_ptr->get_moving_joint_names());

  const auto sensor = MakeRandomSensorForJoints(name_begin, name_end);
  const auto control = MakeRandomControlForJoints(name_begin, name_end);

  // FIXME: Replace Random with the expected stuff...
  const Eigen::VectorXd expected_control =
      Eigen::VectorXd::Random(control.feedforward.size());

  auto ctrl = LFController();
  ctrl.initialize(dummy_model_ptr);
  EXPECT_EQ(expected_control, ctrl.compute_control(sensor, control));
}
