#include <chrono>
using namespace std::literals::chrono_literals;

#include <string_view>
using namespace std::literals::string_view_literals;

#include <sstream>
#include <vector>

#include "utils/pd_controller.hpp"
using tests::utils::Gains;
// using tests::utils::References;

#include "utils/robot_model.hpp"
using tests::utils::MakeAllModelDescriptionsFor;
using tests::utils::MakePairOfJointNamesFrom;
using tests::utils::ModelDescription;

#include "linear_feedback_controller/linear_feedback_controller.hpp"
using linear_feedback_controller::ControllerParameters;
using linear_feedback_controller::Duration;
using linear_feedback_controller::LinearFeedbackController;

#include "gtest/gtest.h"

namespace {

struct TestParams {
  ModelDescription model;
  Gains gains;
  Duration pd_to_lf_duration;

  struct PrintFormat;
};

struct TestParams::PrintFormat {
  ModelDescription::PrintFormat model = {};
  bool as_param_name = false;
};

auto PrintTo(const TestParams& params, std::ostream* os,
             TestParams::PrintFormat fmt = {}) -> void {
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;

  if (os == nullptr) return;

  if (not fmt.as_param_name) {
    *os << "TestParams{";

    *os << ".model = ";
    PrintTo(params.model, os, fmt.model);
    *os << ", ";

    *os << ".gains = ";
    PrintTo(params.gains, os);
    *os << ", ";

    *os << ".pd_to_lf_duration = "
        << duration_cast<milliseconds>(params.pd_to_lf_duration).count()
        << "ms ";

    *os << "}";
  } else {
    fmt.model.as_param_name = true;
    PrintTo(params.model, os, fmt.model);
    *os << "_" << duration_cast<milliseconds>(params.pd_to_lf_duration).count()
        << "ms";
  }
}

auto MakeAllValidTestParamsFrom(const std::vector<ModelDescription>& models,
                                std::initializer_list<Duration> durations)
    -> std::vector<TestParams> {
  std::vector<TestParams> out;
  out.reserve(models.size() * durations.size());

  for (const auto& model : models) {
    const auto gains = Gains::Random(model.joint_list.size());
    for (const auto& duration : durations) {
      out.emplace_back(TestParams{
          .model = model,
          .gains = gains,
          .pd_to_lf_duration = duration,
      });
    }
  }

  return out;
}

auto MakeParamsFrom(const TestParams& test_params) -> ControllerParameters {
  const auto& [model, gains, duration] = test_params;

  ControllerParameters out;
  out.urdf = std::string{model.urdf};
  out.robot_has_free_flyer = model.has_free_flyer;

  {
    const auto joint_list = MakePairOfJointNamesFrom(model.joint_list);
    out.controlled_joint_names = std::move(joint_list.controlled);
    out.moving_joint_names = std::move(joint_list.moving);
  }

  {
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
    : public ::testing::TestWithParam<TestParams> {};

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
  EXPECT_TRUE(ctrl.load(MakeParamsFrom(GetParam())));
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
    "    <limit lower=\"-3.14\" upper=\"3.14\" velocity=\"100\" effort=\"10\"/>"
    "  </joint>"
    "  "
    "  <link name=\"l2\"/>"
    "</robot>";

INSTANTIATE_TEST_SUITE_P(
    DummyUrdf, LinearFeedbackControllerTest,
    ::testing::ValuesIn(MakeAllValidTestParamsFrom(
        MakeAllModelDescriptionsFor(dummy_urdf,
                                    {
                                        {{.name = "l01"}},
                                        {{.name = "l01"}, {.name = "l12"}},
                                    }),
        {500ms, 1s})),
    [](const auto& info) {
      std::stringstream stream;
      PrintTo(info.param, &stream, {.as_param_name = true});
      return stream.str();
    }

);

}  // namespace
