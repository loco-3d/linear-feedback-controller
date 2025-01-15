#include <string_view>
#include <tuple>

#include "utils/mutation.hpp"
using tests::utils::TemporaryMutate;

#include "utils/robot_model.hpp"
using linear_feedback_controller::RobotModelBuilder;
using tests::utils::JointDescription;
using tests::utils::MakeRobotModelBuilderFrom;
using tests::utils::ModelDescription;

#include "utils/eigen_conversions.hpp"
using tests::utils::MakeRandomControlForJoints;
using tests::utils::MakeRandomSensorForJoints;
using tests::utils::PushNewJointStateTo;

#include "linear_feedback_controller/lf_controller.hpp"
using linear_feedback_controller::LFController;

#include "gtest/gtest.h"

using namespace std::literals::string_view_literals;

namespace {

template <typename InputIt>
auto MakeLFControllerFrom(ModelDescription model, InputIt first, InputIt last)
    -> std::optional<LFController> {
  std::optional<LFController> output = std::nullopt;

  if (auto rmb = MakeRobotModelBuilderFrom(model, first, last);
      rmb.has_value()) {
    output = LFController{};
    output->initialize(std::make_shared<RobotModelBuilder>(std::move(*rmb)));
  }

  return output;
}

template <typename Range>
auto MakeLFControllerFrom(ModelDescription model, Range&& range)
    -> std::optional<LFController> {
  return MakeLFControllerFrom(model, std::cbegin(range), std::cend(range));
}

using UrdfType = std::string_view;
using FreeFlyerType = bool;
using JointListType = std::vector<JointDescription>;

using LFControllerParams = std::tuple<UrdfType, FreeFlyerType, JointListType>;

auto MakeLFControllerFrom(const LFControllerParams& params)
    -> std::optional<LFController> {
  const auto& [urdf, has_free_flyer, joint_list] = params;
  return MakeLFControllerFrom(
      {
          .urdf = urdf,
          .has_free_flyer = has_free_flyer,
      },
      joint_list);
}

auto MakeRandomSensorFrom(const LFControllerParams& params) {
  const auto& joint_list = std::get<JointListType>(params);
  // TODO: take into account free flyer
  return MakeRandomSensorForJoints(
      std::cbegin(joint_list), std::cend(joint_list),
      [](const auto& joint) { return joint.name; });
}

auto MakeRandomControlFrom(const LFControllerParams& params) {
  const auto& joint_list = std::get<JointListType>(params);
  // TODO: take into account free flyer
  return MakeRandomControlForJoints(
      std::cbegin(joint_list), std::cend(joint_list),
      [](const auto& joint) { return joint.name; });
}

struct LFControllerTest : public ::testing::TestWithParam<LFControllerParams> {
};

TEST(LFControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LFController(); });
}

TEST(LFControllerTest, DISABLED_InitializeNullptr) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ ctrl.initialize(nullptr); });
}

TEST(LFControllerTest, DISABLED_InitializeEmptyModel) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ ctrl.initialize(std::make_shared<RobotModelBuilder>()); });
}

TEST_P(LFControllerTest, Initialize) {
  // .value() will throw if any failure occured
  const auto& [urdf, free_flyer, joint_list] = GetParam();
  const auto dummy_model = std::make_shared<RobotModelBuilder>(
      MakeRobotModelBuilderFrom({.urdf = urdf, .has_free_flyer = free_flyer},
                                joint_list)
          .value());

  auto ctrl = LFController();
  EXPECT_NO_THROW({ ctrl.initialize(dummy_model); });
}

TEST(LFControllerTest, DISABLED_ComputeControlNotInitialized) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ auto _ = ctrl.compute_control({}, {}); });
}

TEST_P(LFControllerTest, DISABLED_ComputeControlNoInput) {
  auto ctrl = MakeLFControllerFrom(GetParam());
  ASSERT_TRUE(ctrl);
  EXPECT_ANY_THROW({ auto _ = ctrl->compute_control({}, {}); });
}

TEST_P(LFControllerTest, DISABLED_ComputeControlUnknownJoints) {
  auto ctrl = MakeLFControllerFrom(GetParam());
  ASSERT_TRUE(ctrl);

  const auto sensor = MakeRandomSensorFrom(GetParam());
  const auto control = MakeRandomControlFrom(GetParam());

  EXPECT_ANY_THROW({
    auto wrong_sensor = sensor;
    wrong_sensor.joint_state.name[0] = "this joint doesn't exist";
    auto _ = ctrl->compute_control(wrong_sensor, control);
  });
}

TEST_P(LFControllerTest, DISABLED_ComputeControlSizeMismatch) {
  auto ctrl = MakeLFControllerFrom(GetParam());
  ASSERT_TRUE(ctrl);

  const auto sensor = MakeRandomSensorFrom(GetParam());
  const auto control = MakeRandomControlFrom(GetParam());

  EXPECT_ANY_THROW({
    auto wrong_sensor = sensor;

    // One more unknown joint inside sensor
    PushNewJointStateTo(
        wrong_sensor.joint_state,
        {.name = "foo", .position = 0.0, .velocity = 0.0, .effort = 0.0});

    auto _ = ctrl->compute_control(wrong_sensor, control);
  });

  EXPECT_ANY_THROW({
    auto wrong_control = control;

    // One more feed forward term
    tests::utils::Grow(wrong_control.feedforward, 1);
    wrong_control.feedforward.tail<1>()[0] = 0.0;

    auto _ = ctrl->compute_control(sensor, wrong_control);
  });

  EXPECT_ANY_THROW({
    auto wrong_control = control;

    // One more row/col to feedback gain
    tests::utils::Grow(wrong_control.feedback_gain, 1);

    wrong_control.feedforward.bottomRows<1>() =
        ::Eigen::VectorXd::Random(wrong_control.feedforward.cols());

    wrong_control.feedforward.rightCols<1>() =
        ::Eigen::VectorXd::Random(wrong_control.feedforward.rows());

    auto _ = ctrl->compute_control(sensor, wrong_control);
  });

  // TODO: Other size mutation... ?
}

/// Create a std::tuple<T&, string_view> with the ref expression as string
#define MakeRefOf(val) std::make_tuple(std::ref((val)), std::string_view{#val})

TEST_P(LFControllerTest, ComputeControlSpecialDouble) {
  auto ctrl = MakeLFControllerFrom(GetParam());
  ASSERT_TRUE(ctrl);

  auto sensor = MakeRandomSensorFrom(GetParam());
  auto control = MakeRandomControlFrom(GetParam());

  // Test for special double values acceptance or not ?
  for (const auto& [ref, str] : {
           MakeRefOf(sensor.base_pose(0)),
           MakeRefOf(sensor.base_pose(3)),
           MakeRefOf(sensor.base_twist(4)),
           MakeRefOf(sensor.joint_state.position(0)),
           MakeRefOf(sensor.joint_state.velocity(0)),
           MakeRefOf(control.feedforward(0)),
           MakeRefOf(control.feedback_gain(0, 0)),
       }) {
    for (auto tmp_value : {
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::signaling_NaN(),
         }) {
      const auto mutation = TemporaryMutate(ref, tmp_value);
      EXPECT_ANY_THROW({ auto _ = ctrl->compute_control(sensor, control); })
          << str << " = " << tmp_value << " (was " << mutation.OldValue()
          << ")";
    }
  }
}

TEST_P(LFControllerTest, ComputeControl) {
  auto ctrl = MakeLFControllerFrom(GetParam());
  ASSERT_TRUE(ctrl);

  const auto sensor = MakeRandomSensorFrom(GetParam());
  const auto control = MakeRandomControlFrom(GetParam());

  // FIXME: Replace Random with the expected stuff...
  const Eigen::VectorXd expected_control =
      Eigen::VectorXd::Random(control.feedforward.size());

  EXPECT_EQ(expected_control, ctrl->compute_control(sensor, control));
}

constexpr auto dummy_urdf =
    "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
    "<robot name=\"dummy\">"
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

INSTANTIATE_TEST_SUITE_P(DummyUrdf, LFControllerTest,
                         ::testing::Combine(::testing::Values(dummy_urdf),
                                            ::testing::Bool(),
                                            ::testing::Values(
                                                // JointListType{
                                                //     {.name = "l1_to_l2"},
                                                // },
                                                JointListType{
                                                    {.name = "l0_to_l1"},
                                                    {.name = "l1_to_l2"},
                                                })),
                         [](auto&& info) {
                           auto str = std::to_string(
                               std::size(std::get<JointListType>(info.param)));
                           str.append("_Joints");
                           if (std::get<FreeFlyerType>(info.param)) {
                             str.append("_FreeFlyer");
                           }

                           return str;
                         });

}  // namespace
