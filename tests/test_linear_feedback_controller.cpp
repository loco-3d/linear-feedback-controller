#include <sstream>
#include <string_view>

#include "utils/core.hpp"
using tests::utils::DoNot;

#include "utils/mutation.hpp"
using tests::utils::TemporaryMutate;

#include "utils/pd_controller.hpp"
using tests::utils::ExpectedPDControlFrom;
using tests::utils::Gains;
using tests::utils::References;

#include "utils/eigen_conversions.hpp"
using linear_feedback_controller_msgs::Eigen::Control;
using linear_feedback_controller_msgs::Eigen::Sensor;

#include "utils/lf_controller.hpp"
using tests::utils::ExpectedLFControlFrom;
using tests::utils::MakeValidRandomControlFor;
using tests::utils::MakeValidRandomSensorFor;

#include "utils/linear_feedback_controller.hpp"
using tests::utils::MakeAllControllerParametersFrom;

#include "linear_feedback_controller/linear_feedback_controller.hpp"
using linear_feedback_controller::ControllerParameters;
using linear_feedback_controller::Duration;
using linear_feedback_controller::LinearFeedbackController;

#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace {

struct LinearFeedbackControllerTest
    : public ::testing::TestWithParam<ControllerParameters> {};

constexpr auto Load(LinearFeedbackController& ctrl) {
  return [&](const ControllerParameters& params) { return ctrl.load(params); };
}

constexpr auto SetInitialState(LinearFeedbackController& ctrl) {
  return [&](const References& refs) {
    return ctrl.set_initial_state(refs.tau, refs.q);
  };
}

constexpr auto SuccesfullyInitialized(LinearFeedbackController& ctrl) {
  return [&](const ControllerParameters& params, const References& refs) {
    return ctrl.load(params) and ctrl.set_initial_state(refs.tau, refs.q);
  };
}

TEST(LinearFeedbackControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LinearFeedbackController{}; });
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadEmptyParams) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_PRED1(DoNot(Load(ctrl)), ControllerParameters{});
  EXPECT_EQ(ctrl.get_robot_model(), nullptr);
}

TEST_P(LinearFeedbackControllerTest, DISABLED_LoadNoURDF) {
  auto ctrl = LinearFeedbackController{};
  auto no_urdf_param = GetParam();
  no_urdf_param.urdf.clear();
  EXPECT_PRED1(DoNot(Load(ctrl)), no_urdf_param);
  EXPECT_EQ(ctrl.get_robot_model(), nullptr);
}

TEST_P(LinearFeedbackControllerTest, DISABLED_LoadSizeMismatch) {
  auto ctrl = LinearFeedbackController{};
  auto param_size_mismatch = GetParam();
  // TODO
  EXPECT_PRED1(DoNot(Load(ctrl)), param_size_mismatch);
  EXPECT_EQ(ctrl.get_robot_model(), nullptr);
}

TEST_P(LinearFeedbackControllerTest, DISABLED_LoadNegativeDuration) {
  auto ctrl = LinearFeedbackController{};

  auto negative_duration_params = GetParam();
  negative_duration_params.pd_to_lf_transition_duration =
      -negative_duration_params.pd_to_lf_transition_duration;

  EXPECT_PRED1(DoNot(Load(ctrl)), negative_duration_params);
  EXPECT_EQ(ctrl.get_robot_model(), nullptr);
}

TEST_P(LinearFeedbackControllerTest, Load) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_PRED1(Load(ctrl), GetParam());
  EXPECT_NE(ctrl.get_robot_model(), nullptr);
}

TEST_P(LinearFeedbackControllerTest, DISABLED_SetInitialStateEmpty) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_PRED1(Load(ctrl), GetParam());
  EXPECT_PRED1(DoNot(SetInitialState(ctrl)), References{});
}

TEST_P(LinearFeedbackControllerTest, DISABLED_SetInitialStateSizeMismatch) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_PRED1(Load(ctrl), GetParam());
  const auto good_refs = References::Random(GetParam().d_gains.size());

  {
    auto tau_bigger = good_refs;
    tau_bigger.tau << tau_bigger.tau, 1.0;
    EXPECT_PRED1(DoNot(SetInitialState(ctrl)), tau_bigger);
  }

  {
    auto tau_smaller = good_refs;
    tau_smaller.tau.conservativeResize(tau_smaller.tau.size() - 1);
    EXPECT_PRED1(DoNot(SetInitialState(ctrl)), tau_smaller);
  }

  {
    auto q_bigger = good_refs;
    q_bigger.q << q_bigger.q, 1.0;
    EXPECT_PRED1(DoNot(SetInitialState(ctrl)), q_bigger);
  }

  {
    auto q_smaller = good_refs;
    q_smaller.q.conservativeResize(q_smaller.q.size() - 1);
    EXPECT_PRED1(DoNot(SetInitialState(ctrl)), q_smaller);
  }
}

#define MakeRefOf(val) std::make_tuple(std::ref((val)), std::string_view{#val})

TEST_P(LinearFeedbackControllerTest, DISABLED_SetInitialStateSpecialDouble) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_PRED1(Load(ctrl), GetParam());
  auto refs = References::Random(GetParam().d_gains.size());

  for (const auto& [ref, str] : {
           MakeRefOf(refs.q(0)),
           MakeRefOf(refs.q.tail<1>()[0]),
           MakeRefOf(refs.tau(0)),
           MakeRefOf(refs.tau.tail<1>()[0]),
       }) {
    for (auto tmp_value : {
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::signaling_NaN(),
         }) {
      const auto mutation = TemporaryMutate(ref, tmp_value);
      EXPECT_PRED1(DoNot(SetInitialState(ctrl)), refs)
          << str << " = " << ref << " (was " << mutation.OldValue() << ")";
    }
  }
}

TEST_P(LinearFeedbackControllerTest, SetInitialState) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_PRED1(Load(ctrl), GetParam());
  EXPECT_PRED1(SetInitialState(ctrl),
               References::Random(GetParam().d_gains.size()));

  ASSERT_NE(ctrl.get_robot_model(), nullptr);
  EXPECT_EQ(ctrl.get_robot_model()->get_moving_joint_names(),
            GetParam().moving_joint_names);

  EXPECT_EQ(ctrl.get_robot_model()->get_robot_has_free_flyer(),
            GetParam().robot_has_free_flyer);

  // Other verifications based on RMB ? ...
}

TEST_P(LinearFeedbackControllerTest, ComputeControl) {
  auto ctrl = LinearFeedbackController{};
  const auto refs = References::Random(GetParam().d_gains.size());

  ASSERT_PRED2(SuccesfullyInitialized(ctrl), GetParam(), refs);

  constexpr auto ToEigen = [](const std::vector<double>& v) {
    return Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
  };

  const auto gains = Gains{
      .p = ToEigen(GetParam().p_gains),
      .d = ToEigen(GetParam().d_gains),
  };

  const auto control = MakeValidRandomControlFor(*ctrl.get_robot_model());
  const auto sensor = MakeValidRandomSensorFor(*ctrl.get_robot_model());

  // First call always calls PDController
  const auto first_call = std::chrono::high_resolution_clock::now();
  EXPECT_EQ(ctrl.compute_control(first_call, sensor, control, false),
            ExpectedPDControlFrom(gains, refs, sensor.joint_state.position,
                                  sensor.joint_state.velocity));

  EXPECT_EQ(
      ctrl.compute_control(
          (first_call + GetParam().pd_to_lf_transition_duration + 1us), sensor,
          control, false),
      ExpectedLFControlFrom(sensor, control, GetParam().robot_has_free_flyer));

  // This test that time::now() is not used inside the controller an only
  // depends on the first_call
  EXPECT_EQ(ctrl.compute_control(first_call, sensor, control, false),
            ExpectedPDControlFrom(gains, refs, sensor.joint_state.position,
                                  sensor.joint_state.velocity));

  // TODO: Gravity compensation ????
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
