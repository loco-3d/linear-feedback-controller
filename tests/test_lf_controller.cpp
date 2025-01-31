#include <string_view>
#include <tuple>

#include "utils/mutation.hpp"
using tests::utils::TemporaryMutate;

#include "utils/lf_controller.hpp"
using tests::utils::ExpectedLFControlFrom;
using tests::utils::MakeValidRandomControlFor;
using tests::utils::MakeValidRandomSensorFor;

#include "utils/robot_model.hpp"
using tests::utils::JointType;
using tests::utils::MakeAllModelDescriptionsFor;
using tests::utils::ModelDescription;

#include "utils/eigen_conversions.hpp"
using tests::utils::PushNewJointStateTo;

#include "linear_feedback_controller/lf_controller.hpp"
using linear_feedback_controller::LFController;

#include "gtest/gtest.h"

using namespace std::literals::string_view_literals;

namespace {

struct LFControllerTest : public ::testing::TestWithParam<ModelDescription> {};

TEST(LFControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LFController(); });
}

TEST(LFControllerTest, DISABLED_InitializeNullptr) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ ctrl.initialize(nullptr); });
}

TEST(LFControllerTest, DISABLED_InitializeEmptyModel) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({
    ctrl.initialize(
        std::make_shared<linear_feedback_controller::RobotModelBuilder>());
  });
}

TEST_P(LFControllerTest, Initialize) {
  const auto model_ptr = std::shared_ptr{MakeRobotModelBuilderFrom(GetParam())};
  ASSERT_NE(model_ptr, nullptr);

  auto ctrl = LFController();
  EXPECT_NO_THROW({ ctrl.initialize(model_ptr); });
}

TEST(LFControllerTest, DISABLED_ComputeControlNotInitialized) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({ auto _ = ctrl.compute_control({}, {}); });
}

TEST_P(LFControllerTest, DISABLED_ComputeControlNoInput) {
  const auto model_ptr = std::shared_ptr{MakeRobotModelBuilderFrom(GetParam())};
  ASSERT_NE(model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(model_ptr);

  EXPECT_ANY_THROW({ auto _ = ctrl.compute_control({}, {}); });
}

TEST_P(LFControllerTest, DISABLED_ComputeControlUnknownJoints) {
  const auto model_ptr = std::shared_ptr{MakeRobotModelBuilderFrom(GetParam())};
  ASSERT_NE(model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(model_ptr);

  const auto sensor = MakeValidRandomSensorFor(*model_ptr);
  const auto control = MakeValidRandomControlFor(*model_ptr);

  EXPECT_ANY_THROW({
    auto wrong_sensor = sensor;
    wrong_sensor.joint_state.name[0] = "this joint doesn't exist";
    auto _ = ctrl.compute_control(wrong_sensor, control);
  });
}

TEST_P(LFControllerTest, DISABLED_ComputeControlSizeMismatch) {
  const auto model_ptr = std::shared_ptr{MakeRobotModelBuilderFrom(GetParam())};
  ASSERT_NE(model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(model_ptr);

  const auto sensor = MakeValidRandomSensorFor(*model_ptr);
  const auto control = MakeValidRandomControlFor(*model_ptr);

  EXPECT_ANY_THROW({
    auto wrong_sensor = sensor;

    // One more unknown joint inside sensor
    PushNewJointStateTo(
        wrong_sensor.joint_state,
        {.name = "foo", .position = 0.0, .velocity = 0.0, .effort = 0.0});

    auto _ = ctrl.compute_control(wrong_sensor, control);
  });

  EXPECT_ANY_THROW({
    auto wrong_control = control;

    // One more unknown joint inside control.initial_state
    PushNewJointStateTo(
        wrong_control.initial_state.joint_state,
        {.name = "foo", .position = 0.0, .velocity = 0.0, .effort = 0.0});

    auto _ = ctrl.compute_control(sensor, wrong_control);
  });

  EXPECT_ANY_THROW({
    auto wrong_control = control;

    // One more feed forward term
    tests::utils::Grow(wrong_control.feedforward, 1);
    wrong_control.feedforward.tail<1>()[0] = 0.0;

    auto _ = ctrl.compute_control(sensor, wrong_control);
  });

  EXPECT_ANY_THROW({
    auto wrong_control = control;

    // One more row/col to feedback gain
    tests::utils::Grow(wrong_control.feedback_gain, 1);

    wrong_control.feedback_gain.bottomRows<1>() =
        ::Eigen::VectorXd::Random(wrong_control.feedback_gain.cols());

    wrong_control.feedback_gain.rightCols<1>() =
        ::Eigen::VectorXd::Random(wrong_control.feedback_gain.rows());

    auto _ = ctrl.compute_control(sensor, wrong_control);
  });

  // TODO: Other size mutation... ?
}

/// Create a std::tuple<T&, string_view> with the ref expression as string
#define MakeRefOf(val) std::make_tuple(std::ref((val)), std::string_view{#val})

TEST_P(LFControllerTest, DISABLED_ComputeControlSpecialDouble) {
  const auto model_ptr = std::shared_ptr{MakeRobotModelBuilderFrom(GetParam())};
  ASSERT_NE(model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(model_ptr);

  auto sensor = MakeValidRandomSensorFor(*model_ptr);
  auto control = MakeValidRandomControlFor(*model_ptr);

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
      EXPECT_ANY_THROW({ auto _ = ctrl.compute_control(sensor, control); })
          << str << " = " << tmp_value << " (was " << mutation.OldValue()
          << ")";
    }
  }
}

TEST_P(LFControllerTest, ComputeControl) {
  const auto model_ptr = std::shared_ptr{MakeRobotModelBuilderFrom(GetParam())};
  ASSERT_NE(model_ptr, nullptr);

  auto ctrl = LFController();
  ctrl.initialize(model_ptr);

  const auto sensor = MakeValidRandomSensorFor(*model_ptr);
  const auto control = MakeValidRandomControlFor(*model_ptr);
  EXPECT_EQ(ExpectedLFControlFrom(*model_ptr, sensor, control),
            ctrl.compute_control(sensor, control));
}

constexpr auto dummy_urdf =
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
    "</robot>"sv;

INSTANTIATE_TEST_SUITE_P(
    DummyUrdf, LFControllerTest,
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
    [](auto&& info) {
      std::string str;
      if (info.param.has_free_flyer) {
        str.append("FreeFlyer_");
      }

      str.append(std::to_string(size(info.param.joint_list)));
      str.append("_Joints");

      for (const auto& [name, type] : info.param.joint_list) {
        str.append("_");
        str.append(name);
        str.append("_");
        str.append(ToString(type));
      }

      return str;
    });

}  // namespace
