#ifndef LINEAR_FEEDBACK_CONTROLLER_TESTS__LF_CONTROLLER_HPP_
#define LINEAR_FEEDBACK_CONTROLLER_TESTS__LF_CONTROLLER_HPP_

#include "eigen_conversions.hpp"
#include "linear_feedback_controller/lf_controller.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"  // pinocchio::difference
#include "robot_model.hpp"

namespace tests::utils {
/**
 *  @brief Create a randomized JointState struct for each names inside a model
 *
 *  @param[in] model RobotModelBuilder use to create a valid set of Joint state
 *
 *  @return linear_feedback_controller_msgs::Eigen::JointState Randomized
 */
inline auto MakeValidRandomJointStateFor(
    const linear_feedback_controller::RobotModelBuilder& model)
    -> linear_feedback_controller_msgs::Eigen::JointState {
  linear_feedback_controller_msgs::Eigen::JointState joint_state;

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
inline auto MakeValidRandomSensorFor(
    const linear_feedback_controller::RobotModelBuilder& model)
    -> linear_feedback_controller_msgs::Eigen::Sensor {
  linear_feedback_controller_msgs::Eigen::Sensor sensor;

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
 *  @brief Create a randomized Control struct for each names provided
 *
 *  @param[in] model RobotModelBuilder use to create a valid set of values
 *
 *  @return linear_feedback_controller_msgs::Eigen::Control Randomized
 */
inline auto MakeValidRandomControlFor(
    const linear_feedback_controller::RobotModelBuilder& model)
    -> linear_feedback_controller_msgs::Eigen::Control {
  linear_feedback_controller_msgs::Eigen::Control control;

  // get_n* function take into account the free flyer stuff
  control.feedforward = Eigen::VectorXd::Random(model.get_joint_nv());
  control.feedback_gain = Eigen::MatrixXd::Random(
      /* rows = */ model.get_joint_nv(),
      /* cols = */ model.get_nv() * 2);

  control.initial_state = MakeValidRandomSensorFor(model);

  return control;
}

/**
 *  @brief Compute the expected LFController output
 *
 *  @param[in] model RobotModelBuilder use to create a valid set of values
 *  @param[in] sensor, control Set of values forwarded to the LFController
 *
 *  @return Eigen::VectorXd The expected set of values
 */
inline auto ExpectedLFControlFrom(
    const linear_feedback_controller::RobotModelBuilder& model,
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor,
    const linear_feedback_controller_msgs::Eigen::Control& control)
    -> Eigen::VectorXd {
  Eigen::VectorXd out;

  const auto x = RobotState::From(sensor, model.get_robot_has_free_flyer());
  const auto x_0 =
      RobotState::From(control.initial_state, model.get_robot_has_free_flyer());

  auto error = Eigen::VectorXd{control.feedback_gain.cols()};
  pinocchio::difference(model.get_model(), x.position, x_0.position,
                        error.head(model.get_model().nv));
  error.tail(model.get_model().nv) = x_0.velocity - x.velocity;

  out = control.feedforward + (control.feedback_gain * error);
  return out;
}

}  // namespace tests::utils

#endif  // LINEAR_FEEDBACK_CONTROLLER_TESTS__LF_CONTROLLER_HPP_
