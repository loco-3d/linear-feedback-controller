#include <sstream>
#include <string_view>

#include "utils/linear_feedback_controller.hpp"
using tests::utils::MakeAllControllerParametersFrom;
using tests::utils::References;

#include "linear_feedback_controller/linear_feedback_controller.hpp"
using linear_feedback_controller::ControllerParameters;
using linear_feedback_controller::Duration;
using linear_feedback_controller::LinearFeedbackController;

#include "gtest/gtest.h"

namespace {

struct LinearFeedbackControllerTest
    : public ::testing::TestWithParam<ControllerParameters> {};

template <typename Pred>
constexpr auto Not(Pred&& pred) {
  return [&pred](auto&&... val) {
    return not pred(std::forward<decltype(val)>(val)...);
  };
}

constexpr auto CorrectlyLoads(LinearFeedbackController& ctrl) {
  return [&](const ControllerParameters& params) { return ctrl.load(params); };
}

constexpr auto FailsToLoad(LinearFeedbackController& ctrl) {
  return Not(CorrectlyLoads(ctrl));
}

constexpr auto CorrectlySetInitialState(LinearFeedbackController& ctrl) {
  return [&](const References& refs) {
    return ctrl.set_initial_state(refs.tau, refs.q);
  };
}

constexpr auto FailsToSetInitialState(LinearFeedbackController& ctrl) {
  return Not(CorrectlySetInitialState(ctrl));
}

TEST(LinearFeedbackControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LinearFeedbackController{}; });
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadEmptyParams) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_PRED1(Not(CorrectlyLoads(ctrl)), ControllerParameters{});
}

TEST_P(LinearFeedbackControllerTest, DISABLED_LoadNoURDF) {
  auto ctrl = LinearFeedbackController{};
  auto no_urdf_param = GetParam();
  no_urdf_param.urdf.clear();
  EXPECT_PRED1(FailsToLoad(ctrl), no_urdf_param);
}

TEST_P(LinearFeedbackControllerTest, DISABLED_LoadSizeMismatch) {
  auto ctrl = LinearFeedbackController{};
  auto params = GetParam();
  // TODO
  EXPECT_FALSE(ctrl.load(params));
}

TEST_P(LinearFeedbackControllerTest, DISABLED_LoadNegativeDuration) {
  auto ctrl = LinearFeedbackController{};

  auto negative_duration_params = GetParam();
  negative_duration_params.pd_to_lf_transition_duration =
      -negative_duration_params.pd_to_lf_transition_duration;

  EXPECT_PRED1(FailsToLoad(ctrl), negative_duration_params);
}

TEST_P(LinearFeedbackControllerTest, Load) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_PRED1(CorrectlyLoads(ctrl), GetParam());
}

TEST_P(LinearFeedbackControllerTest, SetInitialStateEmpty) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_PRED1(CorrectlyLoads(ctrl), GetParam());
  EXPECT_PRED1(FailsToSetInitialState(ctrl), References{});
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

using namespace std::literals::chrono_literals;
INSTANTIATE_TEST_SUITE_P(DummyUrdf, LinearFeedbackControllerTest,
                         ::testing::ValuesIn(MakeAllControllerParametersFrom(
                             dummy_urdf,
                             {
                                 {{.name = "l01"}},
                                 {{.name = "l01"}, {.name = "l12"}},
                             },
                             {500ms, 1s})),
                         [](const auto& info) {
                           std::stringstream stream;
                           PrintTo(info.param, &stream,
                                   {.as_param_name = true});
                           return stream.str();
                         }

);

}  // namespace
