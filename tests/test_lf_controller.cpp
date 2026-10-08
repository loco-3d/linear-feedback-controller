#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "linear_feedback_controller/lf_controller.hpp"
#include "utils/mock_robot_model_builder.hpp"
#include "utils/mock_robot_model_builder_smart.hpp"

using namespace linear_feedback_controller;
using ::testing::_;
using ::testing::Invoke;

namespace {
// Mirrors the real (non-free-flyer) construct_robot_state: passes the joint
// state straight through. Lets a test call compute_control an arbitrary
// number of times with arbitrary inputs without pre-choreographing WillOnce
// sequences.
void PassThroughConstructRobotState(
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor,
    Eigen::VectorXd& robot_configuration, Eigen::VectorXd& robot_velocity) {
  robot_configuration = sensor.joint_state.position;
  robot_velocity = sensor.joint_state.velocity;
}
}  // namespace

// basic fixture "happy path"
class LFControllerTest : public ::testing::Test {
 protected:
  std::shared_ptr<SmartMockRobotModelBuilder> mock_robot_builder_;
  std::unique_ptr<LFController> controller_;

  void SetUp() override {
    mock_robot_builder_ = std::make_shared<SmartMockRobotModelBuilder>();
    controller_ = std::make_unique<LFController>();
    controller_->initialize(mock_robot_builder_);
  }
};

TEST_F(LFControllerTest, CanBeConstructed) {
  ASSERT_NE(controller_, nullptr);
  ASSERT_NE(mock_robot_builder_, nullptr);
}

TEST_F(LFControllerTest, ComputesCorrectControlSignal) {
  // we use the robotModel inside the smart mocked class (nv = nq = 2)
  int nq = 2;
  int nv = 2;

  // We define measured and desired states
  Eigen::VectorXd q_measured(nq);
  q_measured << 0.1, -0.1;
  Eigen::VectorXd v_measured(nv);
  v_measured << 0.2, 0.0;

  Eigen::VectorXd q_desired(nq);
  q_desired << 0.0, 0.0;
  Eigen::VectorXd v_desired(nv);
  v_desired << 0.0, 0.0;

  // We define feedforward and gains (these are supposed to be an output of
  // crocoddyl)
  Eigen::VectorXd feedforward_input(nv);
  feedforward_input << 9.5, 1.4;
  Eigen::MatrixXd feedback_gain_input(nv, 2 * nv);
  // K = [Kp | Kd]
  // Kp = diag(100, 120)
  // Kd = diag(20, 25)
  feedback_gain_input << 100, 0, 20, 0, 0, 120, 0, 25;

  // We put all the generated data on expected format (ROS messages)
  linear_feedback_controller_msgs::Eigen::Sensor sensor_msg;
  sensor_msg.joint_state.position = q_measured;
  sensor_msg.joint_state.velocity = v_measured;

  linear_feedback_controller_msgs::Eigen::Control control_msg;
  control_msg.initial_state.joint_state.position = q_desired;
  control_msg.initial_state.joint_state.velocity = v_desired;
  control_msg.feedforward = feedforward_input;
  control_msg.feedback_gain = feedback_gain_input;

  // Compute error state
  Eigen::VectorXd diff_state_expected(2 * nv);
  // Error on position is not a simple soustraction
  pinocchio::difference(mock_robot_builder_->get_model(), q_measured, q_desired,
                        diff_state_expected.head(nv));

  // Error on speed is more simple
  diff_state_expected.tail(nv) = v_desired - v_measured;

  // Compute command law
  // τ = τ_ff + K * x_err
  Eigen::VectorXd expected_control(nv);
  expected_control =
      feedforward_input + feedback_gain_input * diff_state_expected;

  // Precise how the mock function should react
  EXPECT_CALL(*mock_robot_builder_,
              construct_robot_state(testing::_, testing::_, testing::_))
      .WillOnce(testing::DoAll(testing::SetArgReferee<1>(q_desired),
                               testing::SetArgReferee<2>(v_desired)))
      .WillOnce(testing::DoAll(testing::SetArgReferee<1>(q_measured),
                               testing::SetArgReferee<2>(v_measured)));

  const Eigen::VectorXd& actual_control =
      controller_->compute_control(sensor_msg, control_msg);

  // Debug
  std::cout << "Actual control:   " << actual_control.transpose() << std::endl;
  std::cout << "Expected control: " << expected_control.transpose()
            << std::endl;

  ASSERT_TRUE(actual_control.isApprox(expected_control, 1e-9));
}

TEST_F(LFControllerTest, FeedbackGainScaleAppliesToFeedbackTorqueOnly) {
  EXPECT_CALL(*mock_robot_builder_, construct_robot_state(_, _, _))
      .WillRepeatedly(Invoke(&PassThroughConstructRobotState));

  int nq = 2;
  int nv = 2;

  Eigen::VectorXd q_measured(nq);
  q_measured << 0.1, -0.1;
  Eigen::VectorXd v_measured(nv);
  v_measured << 0.2, 0.0;
  Eigen::VectorXd q_desired(nq);
  q_desired << 0.0, 0.0;
  Eigen::VectorXd v_desired(nv);
  v_desired << 0.0, 0.0;
  Eigen::VectorXd feedforward_input(nv);
  feedforward_input << 9.5, 1.4;
  Eigen::MatrixXd feedback_gain_input(nv, 2 * nv);
  feedback_gain_input << 100, 0, 20, 0, 0, 120, 0, 25;

  linear_feedback_controller_msgs::Eigen::Sensor sensor_msg;
  sensor_msg.joint_state.position = q_measured;
  sensor_msg.joint_state.velocity = v_measured;

  linear_feedback_controller_msgs::Eigen::Control control_msg;
  control_msg.initial_state.joint_state.position = q_desired;
  control_msg.initial_state.joint_state.velocity = v_desired;
  control_msg.feedforward = feedforward_input;
  control_msg.feedback_gain = feedback_gain_input;

  Eigen::VectorXd diff_state_expected(2 * nv);
  pinocchio::difference(mock_robot_builder_->get_model(), q_measured, q_desired,
                        diff_state_expected.head(nv));
  diff_state_expected.tail(nv) = v_desired - v_measured;
  const Eigen::VectorXd u_fb_raw_expected =
      feedback_gain_input * diff_state_expected;

  constexpr double kScale = 0.5;
  controller_->set_feedback_gain_scale(kScale);
  const Eigen::VectorXd& actual_control =
      controller_->compute_control(sensor_msg, control_msg);

  ASSERT_TRUE(actual_control.isApprox(
      feedforward_input + kScale * u_fb_raw_expected, 1e-9));
  // u_fb_raw stays the unscaled K*diff_state (debug introspection).
  ASSERT_TRUE(
      controller_->get_feedback_torque_raw().isApprox(u_fb_raw_expected, 1e-9));
  ASSERT_TRUE(controller_->get_feedback_torque().isApprox(
      kScale * u_fb_raw_expected, 1e-9));
}

TEST_F(LFControllerTest, FeedbackGainScaleZeroIsFeedforwardOnly) {
  EXPECT_CALL(*mock_robot_builder_, construct_robot_state(_, _, _))
      .WillRepeatedly(Invoke(&PassThroughConstructRobotState));

  linear_feedback_controller_msgs::Eigen::Sensor sensor_msg;
  sensor_msg.joint_state.position = Eigen::Vector2d(0.1, -0.1);
  sensor_msg.joint_state.velocity = Eigen::Vector2d(0.2, 0.0);

  linear_feedback_controller_msgs::Eigen::Control control_msg;
  control_msg.initial_state.joint_state.position = Eigen::Vector2d(0.0, 0.0);
  control_msg.initial_state.joint_state.velocity = Eigen::Vector2d(0.0, 0.0);
  control_msg.feedforward = Eigen::Vector2d(9.5, 1.4);
  Eigen::MatrixXd feedback_gain_input(2, 4);
  feedback_gain_input << 100, 0, 20, 0, 0, 120, 0, 25;
  control_msg.feedback_gain = feedback_gain_input;

  controller_->set_feedback_gain_scale(0.0);
  const Eigen::VectorXd& actual_control =
      controller_->compute_control(sensor_msg, control_msg);

  ASSERT_TRUE(actual_control.isApprox(control_msg.feedforward, 1e-9));
  ASSERT_TRUE(controller_->get_feedback_torque().isApprox(
      Eigen::VectorXd::Zero(2), 1e-9));
  // The debug (unscaled) raw torque still reflects the real K*diff_state.
  ASSERT_FALSE(controller_->get_feedback_torque_raw().isApprox(
      Eigen::VectorXd::Zero(2), 1e-9));
}

TEST_F(LFControllerTest, IntrospectionGettersExposeReferenceState) {
  EXPECT_CALL(*mock_robot_builder_, construct_robot_state(_, _, _))
      .WillRepeatedly(Invoke(&PassThroughConstructRobotState));

  linear_feedback_controller_msgs::Eigen::Sensor sensor_msg;
  sensor_msg.joint_state.position = Eigen::Vector2d(0.1, -0.1);
  sensor_msg.joint_state.velocity = Eigen::Vector2d(0.2, 0.0);

  linear_feedback_controller_msgs::Eigen::Control control_msg;
  control_msg.initial_state.joint_state.position = Eigen::Vector2d(0.3, 0.4);
  control_msg.initial_state.joint_state.velocity = Eigen::Vector2d(0.5, 0.6);
  control_msg.feedforward = Eigen::Vector2d(9.5, 1.4);
  control_msg.feedback_gain = Eigen::MatrixXd::Zero(2, 4);

  controller_->compute_control(sensor_msg, control_msg);

  ASSERT_TRUE(controller_->get_desired_configuration().isApprox(
      control_msg.initial_state.joint_state.position, 1e-9));
  ASSERT_TRUE(controller_->get_desired_velocity().isApprox(
      control_msg.initial_state.joint_state.velocity, 1e-9));
}

TEST_F(LFControllerTest, FeedbackLowpassDisabledIsANoOp) {
  EXPECT_CALL(*mock_robot_builder_, construct_robot_state(_, _, _))
      .WillRepeatedly(Invoke(&PassThroughConstructRobotState));

  linear_feedback_controller_msgs::Eigen::Sensor sensor_msg;
  sensor_msg.joint_state.position = Eigen::Vector2d(0.1, -0.1);
  sensor_msg.joint_state.velocity = Eigen::Vector2d(0.2, 0.0);

  linear_feedback_controller_msgs::Eigen::Control control_msg;
  control_msg.initial_state.joint_state.position = Eigen::Vector2d(0.0, 0.0);
  control_msg.initial_state.joint_state.velocity = Eigen::Vector2d(0.0, 0.0);
  control_msg.feedforward = Eigen::Vector2d(9.5, 1.4);
  Eigen::MatrixXd feedback_gain_input(2, 4);
  feedback_gain_input << 100, 0, 20, 0, 0, 120, 0, 25;
  control_msg.feedback_gain = feedback_gain_input;

  // cutoff_hz <= 0.
  controller_->configure_feedback_lowpass(0.0, 1000.0);
  controller_->compute_control(sensor_msg, control_msg);
  ASSERT_TRUE(controller_->get_feedback_torque().isApprox(
      controller_->get_feedback_torque_raw(), 1e-9));

  // sample_rate_hz <= 0.
  controller_->configure_feedback_lowpass(10.0, 0.0);
  controller_->compute_control(sensor_msg, control_msg);
  ASSERT_TRUE(controller_->get_feedback_torque().isApprox(
      controller_->get_feedback_torque_raw(), 1e-9));

  // cutoff_hz >= Nyquist (0.5 * sample_rate_hz).
  controller_->configure_feedback_lowpass(600.0, 1000.0);
  controller_->compute_control(sensor_msg, control_msg);
  ASSERT_TRUE(controller_->get_feedback_torque().isApprox(
      controller_->get_feedback_torque_raw(), 1e-9));
}

TEST_F(LFControllerTest, FeedbackLowpassPrimesOnFirstSampleThenFilters) {
  EXPECT_CALL(*mock_robot_builder_, construct_robot_state(_, _, _))
      .WillRepeatedly(Invoke(&PassThroughConstructRobotState));

  linear_feedback_controller_msgs::Eigen::Control control_msg;
  control_msg.initial_state.joint_state.position = Eigen::Vector2d(0.0, 0.0);
  control_msg.initial_state.joint_state.velocity = Eigen::Vector2d(0.0, 0.0);
  control_msg.feedforward = Eigen::Vector2d(9.5, 1.4);
  Eigen::MatrixXd feedback_gain_input(2, 4);
  feedback_gain_input << 100, 0, 20, 0, 0, 120, 0, 25;
  control_msg.feedback_gain = feedback_gain_input;

  controller_->configure_feedback_lowpass(/*cutoff_hz=*/10.0,
                                          /*sample_rate_hz=*/1000.0);

  linear_feedback_controller_msgs::Eigen::Sensor sensor_msg_1;
  sensor_msg_1.joint_state.position = Eigen::Vector2d(0.1, -0.1);
  sensor_msg_1.joint_state.velocity = Eigen::Vector2d(0.2, 0.0);
  controller_->compute_control(sensor_msg_1, control_msg);
  // The filter state is seeded on the very first sample: no startup
  // transient, the filtered output equals the raw feedback torque exactly.
  ASSERT_TRUE(controller_->get_feedback_torque().isApprox(
      controller_->get_feedback_torque_raw(), 1e-9));

  linear_feedback_controller_msgs::Eigen::Sensor sensor_msg_2;
  sensor_msg_2.joint_state.position = Eigen::Vector2d(-0.4, 0.6);
  sensor_msg_2.joint_state.velocity = Eigen::Vector2d(-0.7, 0.3);
  controller_->compute_control(sensor_msg_2, control_msg);
  // A different sample now shows the filter has memory: the filtered output
  // lags behind (differs from) the new raw feedback torque.
  ASSERT_FALSE(controller_->get_feedback_torque().isApprox(
      controller_->get_feedback_torque_raw(), 1e-6));
}

// Robustness fixture
class LFControllerRobustnessTest : public ::testing::Test {
 protected:
  std::shared_ptr<MockRobotModelBuilder> mock_robot_builder_;
  std::unique_ptr<LFController> controller_;

  void SetUp() override { controller_ = std::make_unique<LFController>(); }
};

// initialisation with nullptr model
TEST_F(LFControllerRobustnessTest, InitializeWithNullModelThrows) {
  EXPECT_THROW(controller_->initialize(nullptr), std::invalid_argument);
}

// call of compute_control on a non-initialized controler
TEST_F(LFControllerRobustnessTest, ComputeControlThrowsIfNotInitialized) {
  linear_feedback_controller_msgs::Eigen::Sensor sensor_msg;
  linear_feedback_controller_msgs::Eigen::Control control_msg;

  EXPECT_THROW(controller_->compute_control(sensor_msg, control_msg),
               std::runtime_error);
}
