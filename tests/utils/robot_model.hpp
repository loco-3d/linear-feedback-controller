#pragma once

#include <iomanip>   // std::quoted
#include <iterator>  // std::distance
#include <memory>
#include <optional>
#include <ostream>
#include <string_view>
#include <type_traits>

#include "linear_feedback_controller/robot_model_builder.hpp"

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
/// NOTE: Easy to copy
struct JointDescription {
  std::string_view name;            /*!< Name of the controlled joint */
  JointType type = JointType::Both; /*!< Type of the join */
};

/**
 *  @brief Print a given \a joint into \a os
 *
 *  @param[in] joint The jont we wish to print
 *  @param[inout] os The output stream ptr we wish to print to
 */
constexpr auto PrintTo(JointDescription joint, std::ostream *os) noexcept
    -> void {
  if (os == nullptr) return;

  *os << "JointDescription{";
  *os << ".name = " << std::quoted(joint.name) << ", ";
  *os << ".type = " << std::quoted(ToString(joint.type)) << ", ";
  *os << "}";
}

/// Global information about the model we wish to create
/// NOTE: Easy to copy
struct ModelDescription {
  std::string_view urdf; /*!< Complete Robot URDF description */
  bool has_free_flyer;   /*!< Indicates if the model uses free flyer */
};

/// PrintFormat used by PrintTo to format a Model
struct ModelDescriptionPrintFormat {
  bool full_urdf = false; /*!< Print the full URDF string */
};

constexpr auto GetRobotNameFromURDF(std::string_view urdf)
    -> std::optional<std::string_view> {
  std::optional<std::string_view> res = std::nullopt;

  constexpr std::string_view urdf_robot_name_tag = "<robot name=\"";
  if (const auto begin_pos = urdf.find(urdf_robot_name_tag);
      begin_pos != std::string_view::npos) {
    urdf = urdf.substr(begin_pos + urdf_robot_name_tag.size());

    if (const auto end_pos = urdf.find_first_of("\">");
        (end_pos != std::string_view::npos) and (urdf[end_pos] != '>')) {
      res = urdf.substr(0, end_pos);
    }
  }

  return res;
}

/**
 *  @brief Print a given \a model into \a os
 *
 *  @param[in] model The model we wish to print
 *  @param[inout] os The output stream ptr we wish to print to
 *  @param[in] fmt The format specifier use to print the model
 */
constexpr auto PrintTo(ModelDescription model, std::ostream *os,
                       ModelDescriptionPrintFormat fmt = {}) noexcept -> void {
  if (os == nullptr) return;

  *os << "ModelDescription{";

  *os << ".urdf = ";
  if (fmt.full_urdf) {
    *os << std::quoted(model.urdf);
  } else if (const auto robot_name = GetRobotNameFromURDF(model.urdf);
             robot_name.has_value()) {
    *os << "{Robot = \"" << *robot_name << "\"}";
  } else {
    *os << "str{";
    *os << ".data() = @ " << (void const *)model.urdf.data() << ", ";
    *os << ".size() = " << model.urdf.size() << ", ";
    *os << "}";
  }
  *os << ", ";

  *os << ".has_free_flyer = " << model.has_free_flyer;

  *os << "}";
}

/**
 *  @brief Helper function to create a RobotModelBuilder using a given \a model
 *         and a list of Joints to control
 *
 *  @param[in] model The model description (URDF) we wish to build
 *  @param[in] [first, last) A valid range of JointDescription
 *
 *  @return std::optional<RobotModelBuilder> A valid RobotModelBuilder (i.e.
 *          build_model() returned true), nullopt otherwise
 */
template <typename InputIt>
auto MakeRobotModelBuilderFrom(ModelDescription model, InputIt first,
                               InputIt last)
    -> std::unique_ptr<linear_feedback_controller::RobotModelBuilder> {
  using Category = typename std::iterator_traits<InputIt>::iterator_category;
  static_assert(std::is_base_of_v<std::input_iterator_tag, Category>);

  using ValueType = typename std::iterator_traits<InputIt>::value_type;
  static_assert(std::is_base_of_v<JointDescription, ValueType>);

  const auto number_of_joints = std::distance(first, last);
  assert(number_of_joints >= 0 &&
         "Wrong input range [first, last) (negative size)");

  auto controlled_joints = std::vector<std::string>{};
  controlled_joints.reserve(number_of_joints);

  auto moving_joints = std::vector<std::string>{};
  moving_joints.reserve(number_of_joints);

  for (; first != last; ++first) {
    const JointDescription &joint = *first;

    if (IsControlled(joint.type)) {
      controlled_joints.emplace_back(joint.name);
    }

    if (IsMoving(joint.type)) {
      moving_joints.emplace_back(joint.name);
    }
  }

  if (auto rmb =
          std::make_unique<linear_feedback_controller::RobotModelBuilder>();
      rmb->build_model(std::string{model.urdf}, moving_joints,
                       controlled_joints, model.has_free_flyer)) {
    return rmb;
  } else {
    return nullptr;
  }
}

template <typename Range>
auto MakeRobotModelBuilderFrom(ModelDescription model, Range &&range) {
  return MakeRobotModelBuilderFrom(model,
                                   std::cbegin(std::forward<Range>(range)),
                                   std::cend(std::forward<Range>(range)));
}

}  // namespace tests::utils
