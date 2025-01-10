#pragma once

#include <iomanip>  // std::quoted
#include <memory>   // unique_ptr
#include <ostream>
#include <string_view>

#include "linear_feedback_controller/robot_model_builder.hpp"
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"

namespace tests::utils {

/// Represents the type of joints accepted by the robot model builder
enum class JointType {
  Controlled,
  Moving,
  Both,
};

/**
 *  @return String repr of the given JointType enum ("<UNKNOWN>" if invalid)
 */
constexpr auto ToString(JointType type) noexcept -> std::string_view {
  switch (type) {
    case JointType::Controlled:
      return "Controlled";
    case JointType::Moving:
      return "Moving";
    case JointType::Both:
      return "Both";
  }

  return "<UNKNOWN>";
}

/**
 *  @brief Print a given joint type \a type into \a os
 *
 *  @param[in] type The JointType we wish to print
 *  @param[inout] os The output stream ptr we wish to print to
 */
constexpr auto PrintTo(JointType type, std::ostream *os) noexcept -> void {
  if (os == nullptr) return;
  *os << "JointType{ " << std::quoted(ToString(type)) << " }";
}

/**
 *  @return True if the given type is 'Controlled'
 */
constexpr auto IsControlled(JointType type) noexcept -> bool {
  switch (type) {
    case JointType::Controlled:
    case JointType::Both:
      return true;
    default:
      return false;
  }
}

/**
 *  @return True if the given type is 'Moving'
 */
constexpr auto IsMoving(JointType type) noexcept -> bool {
  switch (type) {
    case JointType::Moving:
    case JointType::Both:
      return true;
    default:
      return false;
  }
}

/// Store info about the joint description used by RobotModelBuilder
struct Joint {
  std::string_view name;            /*!< Name of the controlled joint */
  JointType type = JointType::Both; /*!< Type of the join  */
};

/**
 *  @brief Print a given \a joint into \a os
 *
 *  @param[in] joint The jont we wish to print
 *  @param[inout] os The output stream ptr we wish to print to
 */
constexpr auto PrintTo(const Joint &joint, std::ostream *os) noexcept -> void {
  if (os == nullptr) return;

  *os << "Joint{";
  *os << ".type = " << std::quoted(ToString(joint.type)) << ", ";
  *os << ".name = " << std::quoted(joint.name) << ", ";
  *os << "}";
}

/// Global information about the model we wish to create
struct Model {
  std::string urdf;          /*!< Complete Robot URDF description */
  std::vector<Joint> joints; /*!< List of joints */
  bool has_free_flyer;       /*!< Indicates if the model uses free flyer */
};

/// PrintFormat used by PrintTo to format a Model
struct ModelPrintFormat {
  bool full_urdf = false; /*!< Print the full URDF string */
};

/**
 *  @brief Print a given \a model into \a os
 *
 *  @param[in] model The model we wish to print
 *  @param[inout] os The output stream ptr we wish to print to
 *  @param[in] fmt The format specifier use to print the model
 */
constexpr auto PrintTo(const Model &model, std::ostream *os,
                       ModelPrintFormat fmt = {}) noexcept -> void {
  if (os == nullptr) return;

  *os << "Model{";

  *os << ".urdf = ";
  if (fmt.full_urdf) {
    *os << std::quoted(model.urdf);
  } else {
    *os << "str{.size() = " << model.urdf.size() << "}";
  }
  *os << ", ";

  *os << ".has_free_flyer = " << model.has_free_flyer << ", ";

  *os << ".joints = {";
  for (const auto &joint : model.joints) {
    PrintTo(joint, os);
    *os << ", ";
  }
  *os << "}, ";

  *os << "}";
}

/**
 *  @brief Helper function to create a RobotModelBuilder using a given \a model
 *
 *  @param[in] model The model we wish to build
 *  @return std::unique_ptr<RobotModelBuilder> A valid RobotModelBuilder (i.e.
 *          build_model() returned true), nullptr otherwise
 */
inline auto MakeBuilderFrom(const Model &model) noexcept
    -> std::unique_ptr<linear_feedback_controller::RobotModelBuilder> {
  auto controlled_joints = std::vector<std::string>{};
  controlled_joints.reserve(model.joints.size());

  auto moving_joints = std::vector<std::string>{};
  moving_joints.reserve(model.joints.size());

  for (const auto &joint : model.joints) {
    if (IsControlled(joint.type)) {
      controlled_joints.emplace_back(joint.name);
    }

    if (IsMoving(joint.type)) {
      moving_joints.emplace_back(joint.name);
    }
  }

  auto rmb = std::make_unique<linear_feedback_controller::RobotModelBuilder>();
  if (rmb->build_model(model.urdf, moving_joints, controlled_joints,
                       model.has_free_flyer)) {
    return rmb;
  } else {
    return nullptr;
  }
}

/**
 *  @brief Generate a Sensor data struct filled with Random value
 *
 *  @param[in] rmb The model used to generate valid (i.e. with same joints)
 *  @return linear_feedback_controller_msgs::Eigen::Sensor Randomized
 */
inline auto MakeValidRandomSensorFor(
    const linear_feedback_controller::RobotModelBuilder &rmb)
    -> linear_feedback_controller_msgs::Eigen::Sensor {
  linear_feedback_controller_msgs::Eigen::Sensor sensor;

  sensor.base_pose = decltype(sensor.base_pose)::Random();
  sensor.base_twist = decltype(sensor.base_twist)::Random();

  sensor.joint_state.name = rmb.get_moving_joint_names();

  const auto number_of_joints = sensor.joint_state.name.size();
  sensor.joint_state.position = Eigen::VectorXd::Random(number_of_joints);
  sensor.joint_state.velocity = Eigen::VectorXd::Random(number_of_joints);
  sensor.joint_state.effort = Eigen::VectorXd::Random(number_of_joints);

  return sensor;
}

/**
 *  @brief TODO
 *
 *  @param[in] rmb TODO
 *  @return linear_feedback_controller_msgs::Eigen::Control TODO
 */
inline auto MakeValidRandomControlFor(
    const linear_feedback_controller::RobotModelBuilder &rmb)
    -> linear_feedback_controller_msgs::Eigen::Control {
  linear_feedback_controller_msgs::Eigen::Control control;

  // FIXME: Size ???
  control.feedforward = Eigen::VectorXd::Random(rmb.get_joint_nv());
  control.feedback_gain =
      Eigen::MatrixXd::Random(rmb.get_joint_nv(), rmb.get_joint_nq());

  control.initial_state = MakeValidRandomSensorFor(rmb);
  return control;
}

}  // namespace tests::utils
