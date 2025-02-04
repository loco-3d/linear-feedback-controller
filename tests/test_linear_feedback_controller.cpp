#include <sstream>
#include <string_view>

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
using linear_feedback_controller::RobotModelBuilder;
using tests::utils::ExpectedLFControlFrom;
using tests::utils::MakeValidRandomControlFor;
using tests::utils::MakeValidRandomSensorFor;

#include "utils/linear_feedback_controller.hpp"
using tests::utils::MakeAllControllerParametersFrom;

#include "linear_feedback_controller/linear_feedback_controller.hpp"
using linear_feedback_controller::ControllerParameters;
using linear_feedback_controller::LinearFeedbackController;
using linear_feedback_controller::TimePoint;

#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;
using namespace std::literals::string_view_literals;

#define MakeNamedValueOf(val) std::make_tuple(std::string_view{#val}, (val))
#define MakeNamedRefOf(val) \
  std::make_tuple(std::string_view{#val}, std::ref((val)))

namespace {

/**
 *  @return A predicate used to compare 2 double Eigen::Matrix returning true
 *          when all elements follows |lhs - rhs| <= eps
 *
 *  @param[in] abs_error The max absolute error allowed when comparing doubles
 */
constexpr auto AreAlmostEquals(double abs_error) {
  return [=](const auto& lhs, const auto& rhs) -> bool {
    return ((lhs.array() - rhs.array()).abs() <= abs_error).all();
  };
}

struct PDParams {
  Gains gains;
  References refs;

  static auto From(const ControllerParameters& params) -> PDParams {
    constexpr auto ToEigen = [](const std::vector<double>& v) {
      return Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
    };

    PDParams out;
    out.gains.p = ToEigen(params.p_gains);
    out.gains.d = ToEigen(params.d_gains);
    out.refs = References::Random(params.d_gains.size());
    return out;
  }
};

struct Timestamps {
  TimePoint first_call;
  TimePoint pd_timeout;

  static auto From(const ControllerParameters& params) -> Timestamps {
    Timestamps out;
    out.first_call = TimePoint::clock::now();
    out.pd_timeout = out.first_call + params.pd_to_lf_transition_duration;
    return out;
  }
};

struct ControllerInputs {
  Sensor sensor;
  Control control;

  static auto From(const RobotModelBuilder& model) -> ControllerInputs {
    ControllerInputs out;
    out.sensor = MakeValidRandomSensorFor(model);
    out.control = MakeValidRandomControlFor(model);
    return out;
  }
};

struct Expectations {
  Eigen::VectorXd pd;
  Eigen::VectorXd lf;

  static auto From(const RobotModelBuilder& model, const Gains& gains,
                   const References& refs, const Sensor& sensor,
                   const Control& control) -> Expectations {
    Expectations out;
    out.pd = ExpectedPDControlFrom(gains, refs, sensor.joint_state.position,
                                   sensor.joint_state.velocity);
    out.lf = ExpectedLFControlFrom(model, sensor, control);
    return out;
  }
};

struct LinearFeedbackControllerTest
    : public ::testing::TestWithParam<ControllerParameters> {};

TEST(LinearFeedbackControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LinearFeedbackController{}; });
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadEmptyParams) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_FALSE(ctrl.load(ControllerParameters{}));
  EXPECT_EQ(ctrl.get_robot_model(), nullptr);
}

TEST_P(LinearFeedbackControllerTest, DISABLED_LoadNoURDF) {
  auto ctrl = LinearFeedbackController{};
  auto no_urdf_param = GetParam();
  no_urdf_param.urdf.clear();

  EXPECT_FALSE(ctrl.load(no_urdf_param));
  EXPECT_EQ(ctrl.get_robot_model(), nullptr);
}

TEST_P(LinearFeedbackControllerTest, DISABLED_LoadSizeMismatch) {
  auto ctrl = LinearFeedbackController{};
  auto good_params = GetParam();

  {
    auto p_gains_too_big = good_params;
    p_gains_too_big.p_gains.push_back(3.141592);

    EXPECT_FALSE(ctrl.load(p_gains_too_big));
    EXPECT_EQ(ctrl.get_robot_model(), nullptr);
  }

  {
    auto p_gains_smaller = good_params;
    p_gains_smaller.p_gains.pop_back();

    EXPECT_FALSE(ctrl.load(p_gains_smaller));
    EXPECT_EQ(ctrl.get_robot_model(), nullptr);
  }

  {
    auto d_gains_too_big = good_params;
    d_gains_too_big.d_gains.push_back(3.141592);

    EXPECT_FALSE(ctrl.load(d_gains_too_big));
    EXPECT_EQ(ctrl.get_robot_model(), nullptr);
  }

  {
    auto d_gains_smaller = good_params;
    d_gains_smaller.d_gains.pop_back();

    EXPECT_FALSE(ctrl.load(d_gains_smaller));
    EXPECT_EQ(ctrl.get_robot_model(), nullptr);
  }

  {
    auto moving_joint_cleared = good_params;
    moving_joint_cleared.moving_joint_names.clear();

    EXPECT_FALSE(ctrl.load(moving_joint_cleared));
    EXPECT_EQ(ctrl.get_robot_model(), nullptr);
  }
}

TEST_P(LinearFeedbackControllerTest, DISABLED_LoadNegativeDuration) {
  auto ctrl = LinearFeedbackController{};

  auto negative_duration_params = GetParam();
  negative_duration_params.pd_to_lf_transition_duration =
      -negative_duration_params.pd_to_lf_transition_duration;

  EXPECT_FALSE(ctrl.load(negative_duration_params));
  EXPECT_EQ(ctrl.get_robot_model(), nullptr);
}

TEST_P(LinearFeedbackControllerTest, Load) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_TRUE(ctrl.load(GetParam()));
  EXPECT_NE(ctrl.get_robot_model(), nullptr);
}

TEST_P(LinearFeedbackControllerTest, DISABLED_SetInitialStateEmpty) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_TRUE(ctrl.load(GetParam()));
  EXPECT_FALSE(ctrl.set_initial_state({}, {}));
}

TEST_P(LinearFeedbackControllerTest, DISABLED_SetInitialStateSizeMismatch) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_TRUE(ctrl.load(GetParam()));

  const auto good_refs = References::Random(GetParam().d_gains.size());

  {
    auto tau_bigger = good_refs;
    tau_bigger.tau << tau_bigger.tau, 1.0;
    EXPECT_FALSE(ctrl.set_initial_state(tau_bigger.tau, tau_bigger.q));
  }

  {
    auto tau_smaller = good_refs;
    tau_smaller.tau.conservativeResize(tau_smaller.tau.size() - 1);
    EXPECT_FALSE(ctrl.set_initial_state(tau_smaller.tau, tau_smaller.q));
  }

  {
    auto q_bigger = good_refs;
    q_bigger.q << q_bigger.q, 1.0;
    EXPECT_FALSE(ctrl.set_initial_state(q_bigger.tau, q_bigger.q));
  }

  {
    auto q_smaller = good_refs;
    q_smaller.q.conservativeResize(q_smaller.q.size() - 1);
    EXPECT_FALSE(ctrl.set_initial_state(q_smaller.tau, q_smaller.q));
  }
}

TEST_P(LinearFeedbackControllerTest, DISABLED_SetInitialStateSpecialDouble) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_TRUE(ctrl.load(GetParam()));

  auto refs = References::Random(GetParam().d_gains.size());

  for (const auto& [str, ref] : {
           MakeNamedRefOf(refs.q(0)),
           MakeNamedRefOf(refs.q.tail<1>()[0]),
           MakeNamedRefOf(refs.tau(0)),
           MakeNamedRefOf(refs.tau.tail<1>()[0]),
       }) {
    for (auto tmp_value : {
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::signaling_NaN(),
         }) {
      const auto mutation = TemporaryMutate(ref, tmp_value);
      EXPECT_FALSE(ctrl.set_initial_state(refs.tau, refs.q))
          << str << " = " << ref << " (was " << mutation.OldValue() << ")";
    }
  }
}

TEST_P(LinearFeedbackControllerTest, SetInitialState) {
  auto ctrl = LinearFeedbackController{};
  ASSERT_TRUE(ctrl.load(GetParam()));

  const auto refs = References::Random(GetParam().d_gains.size());
  EXPECT_TRUE(ctrl.set_initial_state(refs.tau, refs.q));

  ASSERT_NE(ctrl.get_robot_model(), nullptr);
  EXPECT_EQ(ctrl.get_robot_model()->get_moving_joint_names(),
            GetParam().moving_joint_names);

  EXPECT_EQ(ctrl.get_robot_model()->get_robot_has_free_flyer(),
            GetParam().robot_has_free_flyer);
}

TEST_P(LinearFeedbackControllerTest, ComputeControl) {
  auto ctrl = LinearFeedbackController{};
  const auto [gains, refs] = PDParams::From(GetParam());
  ASSERT_TRUE(ctrl.load(GetParam()) and
              ctrl.set_initial_state(refs.tau, refs.q));

  const auto [first_call, pd_timeout] = Timestamps::From(GetParam());

  const auto& model = *ctrl.get_robot_model();
  const auto [sensor, control] = ControllerInputs::From(model);
  const auto expected = Expectations::From(model, gains, refs, sensor, control);

  for (auto gravity_compensation : {
           false,
           true,
       }) {
    SCOPED_TRACE(::testing::Message()
                 << "\n gravity_compensation = "
                 << (gravity_compensation ? "TRUE"sv : "FALSE"sv));

    // First call always calls PDController
    EXPECT_EQ(
        ctrl.compute_control(first_call, sensor, control, gravity_compensation),
        expected.pd);

    // When duration expired, always calls LFController
    EXPECT_EQ(ctrl.compute_control(pd_timeout + 1ms, sensor, control,
                                   gravity_compensation),
              expected.lf);

    // In between, compute both and apply a weight based on the time elapsed
    // from the first call and the expected transition
    for (const auto& [str, when] : {
             MakeNamedValueOf(first_call + 1ms),
             MakeNamedValueOf(first_call + 5ms),
             MakeNamedValueOf(pd_timeout - 50ms),
             MakeNamedValueOf(pd_timeout - 1ms),
         }) {
      const double ratio =
          ((when - first_call) / (GetParam().pd_to_lf_transition_duration));

      EXPECT_PRED2(
          AreAlmostEquals(5e-6),
          ctrl.compute_control(when, sensor, control, gravity_compensation),
          (((1.0 - ratio) * expected.pd) + (ratio * expected.lf)))
          << "when = " << std::quoted(str) << " | ratio = " << ratio;
    }

    // This test that time::now() is not used inside the controller an only
    // depends on the first_call
    EXPECT_EQ(
        ctrl.compute_control(first_call, sensor, control, gravity_compensation),
        expected.pd);
  }
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
    "  "
    "  <joint name=\"l23\" type=\"revolute\">"
    "    <parent link=\"l2\"/>"
    "    <child link=\"l3\"/>"
    "    <origin xyz=\"0 1 0\" rpy=\"1 0 0\"/>"
    "    <axis xyz=\"0 1 0\"/>"
    "    <limit lower=\"-3.14\" upper=\"3.14\" velocity=\"10\" effort=\"100\"/>"
    "  </joint>"
    "  "
    "  <link name=\"l3\"/>"
    "</robot>";

INSTANTIATE_TEST_SUITE_P(
    DummyUrdf, LinearFeedbackControllerTest,
    ::testing::ValuesIn(MakeAllControllerParametersFrom(
        dummy_urdf,
        {
            {{.name = "l01"}},
            {{.name = "l01"}, {.name = "l12"}},
            {{.name = "l01"}, {.name = "l23"}},
            {{.name = "l01"}, {.name = "l12"}, {.name = "l23"}},
        },
        {500ms, 1s})),
    [](const auto& info) {
      std::stringstream stream;
      PrintTo(info.param, &stream, {.as_param_name = true});
      return stream.str();
    });

}  // namespace
