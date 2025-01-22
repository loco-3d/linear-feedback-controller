#include <chrono>
using namespace std::literals::chrono_literals;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

#include <string_view>
using namespace std::literals::string_view_literals;

#include "utils/pd_controller.hpp"
using tests::utils::Gains;
// using tests::utils::References;

#include "utils/robot_model.hpp"
using tests::utils::JointType;
using tests::utils::MakeAllModelDescriptionsFor;
using tests::utils::MakePairOfJointNamesFrom;
using tests::utils::ModelDescription;

#include "linear_feedback_controller/linear_feedback_controller.hpp"
using linear_feedback_controller::ControllerParameters;
using linear_feedback_controller::Duration;
using linear_feedback_controller::LinearFeedbackController;

#include "gtest/gtest.h"

namespace {

using LinearFeedbackControllerTestParams =
    std::tuple<ModelDescription, Duration>;

auto MakeValidParamsFrom(const LinearFeedbackControllerTestParams& test_params)
    -> ControllerParameters {
  const auto& [model, duration] = test_params;

  ControllerParameters out;

  out.urdf = std::string{model.urdf};
  out.robot_has_free_flyer = model.has_free_flyer;

  {
    const auto joint_list = MakePairOfJointNamesFrom(model.joint_list);
    out.controlled_joint_names = std::move(joint_list.controlled);
    out.moving_joint_names = std::move(joint_list.moving);
  }

  {
    // FIXME: Size ? number of moving joints ?
    const auto gains = Gains::Random(out.moving_joint_names.size());

    out.d_gains.resize(gains.d.size());
    Eigen::Map<Eigen::VectorXd>(out.d_gains.data(), out.d_gains.size()) =
        gains.d;

    out.p_gains.resize(gains.p.size());
    Eigen::Map<Eigen::VectorXd>(out.p_gains.data(), out.p_gains.size()) =
        gains.p;
  }

  out.pd_to_lf_transition_duration = duration;

  return out;
}

struct LinearFeedbackControllerTest
    : public ::testing::TestWithParam<LinearFeedbackControllerTestParams> {};

TEST(LinearFeedbackControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LinearFeedbackController{}; });
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadEmptyParams) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_FALSE(ctrl.load({}));
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadNoURDF) {
  auto ctrl = LinearFeedbackController{};
  auto params = ControllerParameters{};
  // TODO
  EXPECT_FALSE(ctrl.load(params));
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadSizeMismatch) {
  auto ctrl = LinearFeedbackController{};
  auto params = ControllerParameters{};
  // TODO
  EXPECT_FALSE(ctrl.load(params));
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadNegativeDuration) {
  auto ctrl = LinearFeedbackController{};
  auto params = ControllerParameters{};
  // TODO
  EXPECT_FALSE(ctrl.load(params));
}

TEST_P(LinearFeedbackControllerTest, Load) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_TRUE(ctrl.load(MakeValidParamsFrom(GetParam())));
}

constexpr std::string_view dummy_urdf =
    "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
    "<robot name=\"dummy\">"
    "  <link name=\"l0\"/>"
    "  "
    "  <joint name=\"l01\" type=\"revolute\">"
    "    <parent link=\"l0\"/>"
    "    <child link=\"l1\"/>"
    "    <origin xyz=\"0 0 1\" rpy=\"0 0 1\"/>"
    "    <axis xyz=\"0 0 1\"/>"
    "    <limit lower=\"0\" upper=\"3.14\" velocity=\"100\" effort=\"100\"/>"
    "  </joint>"
    "  "
    "  <link name=\"l1\"/>"
    "  "
    "  <joint name=\"l12\" type=\"revolute\">"
    "    <parent link=\"l1\"/>"
    "    <child link=\"l2\"/>"
    "    <origin xyz=\"0 1 0\" rpy=\"1 0 0\"/>"
    "    <axis xyz=\"0 1 0\"/>"
    "    <limit lower=\"-3.14\" upper=\"3.14\" velocity=\"100\" "
    "effort=\"10\"/>"
    "  </joint>"
    "  "
    "  <link name=\"l2\"/>"
    "</robot>";

INSTANTIATE_TEST_SUITE_P(
    DummyUrdf, LinearFeedbackControllerTest,
    ::testing::Combine(
        ::testing::ValuesIn(MakeAllModelDescriptionsFor(
            dummy_urdf,
            {
                {
                    {.name = "l01"},
                },
                {
                    {.name = "l02", .type = JointType::Controlled},
                },
                {
                    {.name = "l01"},
                    {.name = "l12"},
                },
            })),
        ::testing::ValuesIn(std::vector<Duration>{500ms, 1s})),
    [](const auto& info) {
      // NOTE: Can't use structured binding inside GTest macros
      const auto& model = std::get<ModelDescription>(info.param);
      const auto& duration = std::get<Duration>(info.param);

      std::string str;
      if (model.has_free_flyer) {
        str.append("FreeFlyer_");
      }

      str.append(std::to_string(size(model.joint_list)));
      str.append("_Joints");

      for (const auto& [name, type] : model.joint_list) {
        str.append("_");
        str.append(name);
        str.append("_");
        str.append(ToString(type));
      }

      str.append("_Duration_");
      str.append(std::to_string(duration_cast<milliseconds>(duration).count()));
      str.append("_ms");

      return str;
    });

}  // namespace
