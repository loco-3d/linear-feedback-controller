#pragma once

#include <ostream>
#include <string_view>

#include "eigen.hpp"  // PrintTo(Eigen::Vector), Grow
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"

// Using this namesapce for ADL
namespace linear_feedback_controller_msgs::Eigen {

inline auto PrintTo(const JointState& joint_state, std::ostream* os) noexcept
    -> void {
  if (os == nullptr) return;

  *os << "JointState{";

  const auto number_of_joints = joint_state.name.size();
  *os << number_of_joints << " joints: ";

  for (auto i = 0; i < number_of_joints; ++i) {
    *os << std::quoted(joint_state.name[i]) << " = {";
    using tpl = std::tuple<std::string_view, const ::Eigen::VectorXd&>;
    for (auto&& [name, range] : {
             tpl{".pos", joint_state.position},
             tpl{".vel", joint_state.velocity},
             tpl{".eff", joint_state.effort},
         }) {
      *os << name << " = ";

      // JointState (from ROS) may contains empty ranges/vectors,
      // hence why doing the 'if' below
      if (i < range.size()) {
        *os << range[i];
      } else {
        *os << "N/A";
      }

      *os << ", ";
    }
    *os << "}, ";
  }

  *os << '}';
}

inline auto PrintTo(const Sensor& sensor, std::ostream* os) noexcept -> void {
  if (os == nullptr) return;

  *os << "Sensor{";

  *os << ".base_pose = ";
  PrintTo(sensor.base_pose, os, {.with_size = false});
  *os << ", ";

  *os << ".base_twist = ";
  PrintTo(sensor.base_twist, os, {.with_size = false});
  *os << ", ";

  *os << ".joint_state = ";
  PrintTo(sensor.joint_state, os);

  *os << '}';
}

inline auto PrintTo(const Control& ctrl, std::ostream* os) noexcept -> void {
  if (os == nullptr) return;
  *os << "Control{";

  *os << "feedback_gain = ";
  // TODO
  // PrintTo(ctrl.feedback_gain, os, {.with_size = false});
  *os << ctrl.feedback_gain.format({
      ::Eigen::StreamPrecision,  // precision
      0,                         // flags
      " ",                       // coeffSeparator
      " ",                       // rowSeparator
      "[",                       // rowPrefix
      "]",                       // rowSuffix
      "",                        // matPrefix
      "",                        // matSuffix
      ' ',                       // fill
  });
  *os << ", ";

  *os << ".feedforward = ";
  PrintTo(ctrl.feedforward, os,
          {
              // .strip = {.head = 2, .tail = 2},
              .with_size = true,
          });
  *os << ", ";

  *os << ".initial_state = ";
  PrintTo(ctrl.initial_state, os);

  *os << '}';
}

}  // namespace linear_feedback_controller_msgs::Eigen

namespace tests::utils {

/// Used as argument of the PushNewJointStateTo() function below
struct SingleJointState {
  std::string_view name = "";
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
};

/**
 *  @brief Add a new JointState value into the provided JointState list
 *
 *  @param[inout] joint_state JointState list we wish to modify
 *  @param[in] new_joint_state New data we wish to push into \a joint_state
 */
inline auto PushNewJointStateTo(
    linear_feedback_controller_msgs::Eigen::JointState& joint_state,
    const SingleJointState& new_joint_state) -> void {
  joint_state.name.emplace_back(new_joint_state.name);
  joint_state.position << joint_state.position, new_joint_state.position;
  joint_state.velocity << joint_state.velocity, new_joint_state.velocity;
  joint_state.effort << joint_state.effort, new_joint_state.effort;
}

/**
 *  @brief Reconstruct the expected state X (= [q, v]) from the given sensor
 *
 *  @param[in] sensor The complete sensor data
 *  @param[in] with_free_flyer Indicates if we have a free flyer or not
 *
 *  @return Eigen::VectorXd Containing the complete state X
 */
inline auto GetCompleteStateFrom(
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor,
    bool with_free_flyer) -> Eigen::VectorXd {
  if (with_free_flyer) {
    return ConcatenateAs<Eigen::VectorXd>(
        sensor.base_pose, sensor.joint_state.position, sensor.base_twist,
        sensor.joint_state.velocity);
  } else {
    return ConcatenateAs<Eigen::VectorXd>(sensor.joint_state.position,
                                          sensor.joint_state.velocity);
  }
}

}  // namespace tests::utils
