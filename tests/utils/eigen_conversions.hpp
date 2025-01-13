#pragma once

#include <iterator>  // std::distance
#include <ostream>
#include <string_view>

#include "eigen.hpp"
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
    // *os << '[' << i << "] ";

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

/**
 *  @brief Create a randomized JointState struct for each names provided
 *
 *  @tparam InputIt Input iterator of T used to construct a std::string
 *  @param[in] [first, last) Iterator range of joints name
 *  @return linear_feedback_controller_msgs::Eigen::JointState Randomized
 */
template <typename InputIt>
auto MakeRandomJointStateWithNames(InputIt first, InputIt last)
    -> linear_feedback_controller_msgs::Eigen::JointState {
  // TODO: static_assert() on InputIt to provide meaningfull error msg

  linear_feedback_controller_msgs::Eigen::JointState joint_state;
  if (const auto number_of_joints = std::distance(first, last);
      number_of_joints > 0) {
    // NOTE: Since ::Random() doesn't return a VectorXd, but an operation (eigen
    // stuff), using `auto` and assigning doesn't mean it copies the same vector
    // but it generates a new randomized vector data everytime with assign it.
    const auto generate_random_values =
        Eigen::VectorXd::Random(number_of_joints);

    joint_state.position = generate_random_values;
    joint_state.velocity = generate_random_values;
    joint_state.effort = generate_random_values;

    joint_state.name.reserve(number_of_joints);
    for (; first != last; ++first) {
      joint_state.name.emplace_back(*first);
    }
  }

  return joint_state;
}

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

  Grow(joint_state.position, 1);
  joint_state.position.tail<1>()[0] = new_joint_state.position;

  Grow(joint_state.velocity, 1);
  joint_state.velocity.tail<1>()[0] = new_joint_state.velocity;

  Grow(joint_state.effort, 1);
  joint_state.effort.tail<1>()[0] = new_joint_state.effort;
};

/**
 *  @brief Create a randomized Sensor struct for each joints name
 *
 *  @tparam InputIt Input iterator of std::string compatible elements
 *
 *  @param[in] first, last Range of joints name
 *  @return linear_feedback_controller_msgs::Eigen::Sensor Randomized
 */
template <typename InputIt>
auto MakeRandomSensorForJoints(InputIt first, InputIt last)
    -> linear_feedback_controller_msgs::Eigen::Sensor {
  linear_feedback_controller_msgs::Eigen::Sensor sensor;
  sensor.base_pose = decltype(sensor.base_pose)::Random();
  sensor.base_twist = decltype(sensor.base_twist)::Random();
  sensor.joint_state = MakeRandomJointStateWithNames(first, last);
  return sensor;
}

/**
 *  @brief TODO
 *
 *  @param[in] rmb TODO
 *  @return linear_feedback_controller_msgs::Eigen::Control TODO
 */
template <typename InputIt>
auto MakeRandomControlForJoints(InputIt first, InputIt last)
    -> linear_feedback_controller_msgs::Eigen::Control {
  linear_feedback_controller_msgs::Eigen::Control control;

  control.initial_state = MakeRandomSensorForJoints(first, last);

  const auto number_of_joints = control.initial_state.joint_state.name.size();
  control.feedforward = Eigen::VectorXd::Random(number_of_joints);
  control.feedback_gain =
      Eigen::MatrixXd::Random(number_of_joints, number_of_joints * 2);

  return control;
}
}  // namespace tests::utils
