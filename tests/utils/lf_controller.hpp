#ifndef LINEAR_FEEDBACK_CONTROLLER_TESTS__LF_CONTROLLER_HPP_
#define LINEAR_FEEDBACK_CONTROLLER_TESTS__LF_CONTROLLER_HPP_

#include "linear_feedback_controller/lf_controller.hpp"
#include "linear_feedback_controller/robot_model_builder.hpp"
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"

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
inline auto MakeValidRandomControlFor(
    const linear_feedback_controller::RobotModelBuilder& model)
    -> linear_feedback_controller_msgs::Eigen::Control {
  linear_feedback_controller_msgs::Eigen::Control control;

  // get_n* function take into account the free flyer stuff
  control.feedforward = Eigen::VectorXd::Random(model.get_nv());
  control.feedback_gain = Eigen::MatrixXd::Random(
      /* rows = */ model.get_nv(),
      /* cols = */ model.get_nv() + model.get_nq());

  control.initial_state = MakeValidRandomSensorFor(model);

  return control;
}

}  // namespace tests::utils

#endif  // LINEAR_FEEDBACK_CONTROLLER_TESTS__LF_CONTROLLER_HPP_
