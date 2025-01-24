#include <chrono>
using namespace std::literals::chrono_literals;

#include <string_view>
using namespace std::literals::string_view_literals;

#include <sstream>

#include "utils/linear_feedback_controller.hpp"
using tests::utils::Gains;
using tests::utils::MakeAllControllerParametersFrom;

#include "linear_feedback_controller/linear_feedback_controller.hpp"
using linear_feedback_controller::ControllerParameters;
using linear_feedback_controller::Duration;
using linear_feedback_controller::LinearFeedbackController;

#include "gtest/gtest.h"

namespace {

struct LinearFeedbackControllerTest
    : public ::testing::TestWithParam<ControllerParameters> {};

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
  EXPECT_TRUE(ctrl.load(GetParam()));
}

TEST_P(LinearFeedbackControllerTest, SetInitialStateEmpty) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_TRUE(ctrl.load(GetParam()));
  EXPECT_FALSE(ctrl.set_initial_state({}, {}));
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
