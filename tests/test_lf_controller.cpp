#include <string_view>
#include <tuple>

#include "utils/mutation.hpp"
using tests::utils::TemporaryMutate;

#include "utils/robot_model.hpp"
using linear_feedback_controller::RobotModelBuilder;
using tests::utils::JointDescription;
using tests::utils::JointType;
using tests::utils::MakeAllModelDescriptionsFor;
using tests::utils::ModelDescription;

#include "utils/eigen_conversions.hpp"
using linear_feedback_controller_msgs::Eigen::Control;
using linear_feedback_controller_msgs::Eigen::JointState;
using linear_feedback_controller_msgs::Eigen::Sensor;
using tests::utils::GetCompleteStateFrom;
using tests::utils::PushNewJointStateTo;

#include "linear_feedback_controller/lf_controller.hpp"
using linear_feedback_controller::LFController;

#include "gtest/gtest.h"

using namespace std::literals::string_view_literals;

namespace {

/**
 *  @brief Create a randomized JointState struct for each names inside a model
 *
 *  @param[in] model RobotModelBuilder use to create a valid set of Joint state
 *
 *  @return linear_feedback_controller_msgs::Eigen::JointState Randomized
 */
auto MakeValidRandomJointStateFor(const RobotModelBuilder& model)
    -> JointState {
  JointState joint_state;

  // deep copy
  joint_state.name = model.get_moving_joint_names();

  // NOTE: Since ::Random() doesn't return a VectorXd, but an operation (eigen
  // stuff), using `auto` and assigning doesn't mean it copies the same vector
  // but it generates a new randomized vector data everytime with assign it.
  const auto generate_random_values =
      Eigen::VectorXd::Random(joint_state.name.size());

  joint_state.position = generate_random_values;
  joint_state.velocity = generate_random_values;
  joint_state.effort = generate_random_values;

  return joint_state;
}

/**
 *  @brief Create a randomized Sensor struct for a given model
 *
 *  @param[in] model RobotModelBuilder use to create a valid set of values
 *
 *  @return linear_feedback_controller_msgs::Eigen::Sensor Randomized
 */
auto MakeValidRandomSensorFor(const RobotModelBuilder& model) -> Sensor {
  Sensor sensor;

  // base_pose is composed of a 3D (x,y,z) vector followed by a normalized
  // quaternion
  sensor.base_pose.head<3>() = Eigen::Vector3d::Random();
  sensor.base_pose.tail<4>() = Eigen::Quaterniond::UnitRandom().coeffs();
  sensor.base_twist = decltype(sensor.base_twist)::Random();
  sensor.joint_state = MakeValidRandomJointStateFor(model);

  // TODO: Contacts ???
  return sensor;
}

/**
 *  @brief Create a randomized Sensor struct for each names provided
 *
 *  @tparam InputIt Input iterator containing the joints name
 *  @tparam UnaryOp Unary operation transforming InputIt into a
 *                  std::string_view compatible value
 *
 *  @param[in] [first, last) Range of names used as follow:
 *                           std::string{std::string_view{get_name(*first)}}
 *  @param[in] get_name Functor use to get a name from the dereferenced iterator
 *
 *  @return linear_feedback_controller_msgs::Eigen::Control Randomized
 */
auto MakeValidRandomControlFor(const RobotModelBuilder& model) -> Control {
  Control control;

  // get_n* function take into account the free flyer stuff
  control.feedforward = Eigen::VectorXd::Random(model.get_nv());
  control.feedback_gain = Eigen::MatrixXd::Random(
      /* rows = */ model.get_nv(),
      /* cols = */ model.get_nv() + model.get_nq());

  control.initial_state = MakeValidRandomSensorFor(model);

  return control;
}

using JointListType = std::vector<JointDescription>;
using LFControllerParams = ModelDescription<JointListType>;

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

    // One more feed forward term
    tests::utils::Grow(wrong_control.feedforward, 1);
    wrong_control.feedforward.tail<1>()[0] = 0.0;

    auto _ = ctrl.compute_control(sensor, wrong_control);
  });

  EXPECT_ANY_THROW({
    auto wrong_control = control;

    // One more row/col to feedback gain
    tests::utils::Grow(wrong_control.feedback_gain, 1);

    wrong_control.feedforward.bottomRows<1>() =
        ::Eigen::VectorXd::Random(wrong_control.feedforward.cols());

    wrong_control.feedforward.rightCols<1>() =
        ::Eigen::VectorXd::Random(wrong_control.feedforward.rows());

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

  const auto x =
      GetCompleteStateFrom(sensor, model_ptr->get_robot_has_free_flyer());

  const auto x_0 = GetCompleteStateFrom(control.initial_state,
                                        model_ptr->get_robot_has_free_flyer());

  // FIXME: Compute the error differently based on free flyer ?
  const auto error = x_0 - x;
  EXPECT_EQ((control.feedforward + (control.feedback_gain * error)),
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
    ::testing::ValuesIn(MakeAllModelDescriptionsFor<JointListType>(
        dummy_urdf,
        JointListType{
            {.name = "l01", .type = JointType::Controlled},
        },
        // JointListType{
        //     {.name = "l01", .type = JointType::Moving},
        // },
        JointListType{
            {.name = "l01"},
            {.name = "l12"},
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
