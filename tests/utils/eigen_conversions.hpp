#pragma once

#include <iterator>  // std::distance, std::iterator_traits
#include <ostream>
#include <string_view>
#include <utility>  // std::forward

#include "Eigen/Geometry"  // Eigen::Quaternions
#include "eigen.hpp"       // PrintTo(Eigen::Vector)
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
 *  @tparam InputIt Input iterator containing the joints name
 *  @tparam UnaryOp Unary operation transforming InputIt into a
 *                  std::string_view compatible value
 *
 *  @param[in] [first, last) Range of names used as follow:
 *                           std::string{std::string_view{get_name(*first)}}
 *  @param[in] get_name Functor use to get a name from the dereferenced iterator
 *
 *  @return linear_feedback_controller_msgs::Eigen::JointState Randomized
 */
template <typename InputIt, typename UnaryOp>
auto MakeRandomJointStateWithNames(InputIt first, InputIt last,
                                   UnaryOp&& get_name)
    -> linear_feedback_controller_msgs::Eigen::JointState {
  // TODO: static_assert()

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
      joint_state.name.emplace_back(std::string_view{get_name(*first)});
    }
  }

  return joint_state;
}

/**
 *  @brief Create a randomized JointState struct for each names provided
 *
 *  Basically call MakeRandomJointStateWithNames(first, last, identity).
 *
 *  @tparam InputIt Input iterator containing values convertible to
 *                  std::string_view
 *
 *  @param[in] [first, last) Iterator range used to construct as follow:
 *                           std::string{std::string_view{*first}}
 *
 *  @return linear_feedback_controller_msgs::Eigen::JointState Randomized
 */
template <typename InputIt>
auto MakeRandomJointStateWithNames(InputIt first, InputIt last)
    -> linear_feedback_controller_msgs::Eigen::JointState {
  // Default identity UnaryOp -> forward the value / does nothing
  // Same as std::identity (c++20)
  return MakeRandomJointStateWithNames(
      first, last, [](auto&& value) noexcept -> decltype(auto) {
        return std::forward<decltype(value)>(value);
      });
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
 *  @return linear_feedback_controller_msgs::Eigen::Sensor Randomized
 */
template <typename InputIt, typename UnaryOp>
auto MakeRandomSensorForJoints(InputIt first, InputIt last, UnaryOp&& get_name)
    -> linear_feedback_controller_msgs::Eigen::Sensor {
  linear_feedback_controller_msgs::Eigen::Sensor sensor;

  // base_pose is composed of a 3D (x,y,z) vector followed by a normalized
  // quaternion
  sensor.base_pose.head<3>() = Eigen::Vector3d::Random();
  sensor.base_pose.tail<4>() = Eigen::Quaterniond::UnitRandom().coeffs();

  sensor.base_twist = decltype(sensor.base_twist)::Random();

  sensor.joint_state = MakeRandomJointStateWithNames(
      first, last, std::forward<UnaryOp>(get_name));

  // TODO: Contacts ???
  return sensor;
}

/**
 *  @brief Create a randomized Sensor struct for each joints name
 *
 *  Basically call MakeRandomSensorForJoints(first, last, identity).
 *
 *  @tparam InputIt Input iterator containing values convertible to
 *                  std::string_view
 *
 *  @param[in] [first, last) Range of names used as follow:
 *                           std::string{std::string_view{*first}}
 *
 *  @return linear_feedback_controller_msgs::Eigen::Sensor Randomized
 */
template <typename InputIt>
auto MakeRandomSensorForJoints(InputIt first, InputIt last)
    -> linear_feedback_controller_msgs::Eigen::Sensor {
  // Default identity UnaryOp -> forward the value / does nothing
  // Same as std::identity (c++20)
  return MakeRandomSensorForJoints(
      first, last, [](auto&& value) noexcept -> decltype(auto) {
        return std::forward<decltype(value)>(value);
      });
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
template <typename InputIt, typename UnaryOp>
auto MakeRandomControlForJoints(InputIt first, InputIt last, UnaryOp&& get_name)
    -> linear_feedback_controller_msgs::Eigen::Control {
  linear_feedback_controller_msgs::Eigen::Control control;

  control.initial_state =
      MakeRandomSensorForJoints(first, last, std::forward<UnaryOp>(get_name));

  const auto number_of_joints = control.initial_state.joint_state.name.size();
  control.feedforward = Eigen::VectorXd::Random(number_of_joints);
  control.feedback_gain =
      Eigen::MatrixXd::Random(number_of_joints, number_of_joints * 2);

  return control;
}

/**
 *  @brief Create a randomized JointState struct for each names provided
 *
 *  Basically call MakeRandomControlForJoints(first, last, identity).
 *
 *  @tparam InputIt Input iterator containing values convertible to
 *                  std::string_view
 *
 *  @param[in] [first, last) Iterator range used to construct as follow:
 *                           std::string{std::string_view{*first}}
 *
 *  @return linear_feedback_controller_msgs::Eigen::Control Randomized
 */
template <typename InputIt>
auto MakeRandomControlForJoints(InputIt first, InputIt last)
    -> linear_feedback_controller_msgs::Eigen::Control {
  // Default identity UnaryOp -> forward the value / does nothing
  // Same as std::identity (c++20)
  return MakeRandomControlForJoints(
      first, last, [](auto&& value) noexcept -> decltype(auto) {
        return std::forward<decltype(value)>(value);
      });
}

}  // namespace tests::utils
