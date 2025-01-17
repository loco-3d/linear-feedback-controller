#pragma once

#include <array>
#include <iomanip>  // std::quoted
#include <memory>
#include <optional>
#include <ostream>
#include <string_view>
#include <type_traits>

#include "linear_feedback_controller/robot_model_builder.hpp"
#include "view.hpp"

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

  *os << ".type = ";
  PrintTo(joint.type, os);
  *os << ", ";

  *os << "}";
}

/// Global information about the model we wish to create
/// IMPORTANT: Should not be used directly but as function argument
template <typename Range>
struct ModelDescription {
  static_assert(
      std::is_base_of_v<typename View<Range>::ValueType, JointDescription>,
      "The Range provided to joint_list MUST contains JointDescription in it");

  /// PrintFormat used by PrintTo to format a Model
  struct PrintFormat;

  std::string_view urdf; /*!< Complete Robot URDF description */
  Range joint_list;      /*!< Joint description list */
  bool has_free_flyer;   /*!< Indicates if the model uses free flyer */
};

/// CTAD use to create a ModelDescription using the Range type
template <typename Range>
ModelDescription(std::string_view, Range, bool) -> ModelDescription<Range>;

/**
 *  @brief Create an array of ModelDescription<> with and without free flyer,
 *         for each joint_list provided
 *
 *  @tparam ListType The joint_list type used by ModelDescription<ListType>
 *  @tparam Lists... All list arguments types
 *
 *  @param[in] urdf The common URDF used by every ModelDescription
 *  @param[in] joint_lists... All arguments forwarded to the '.joint_list = '
 *                            constructor
 *
 *  @return std::array<ModelDescription<ListType>, 2 * sizeof...(Lists)>
 *          Containing all ModelDescriptions constructed
 */
template <typename ListType, typename... Lists>
constexpr auto MakeAllModelDescriptionsFor(std::string_view urdf,
                                           Lists &&...joint_lists) noexcept
    -> std::array<ModelDescription<ListType>, 2 * (sizeof...(Lists))> {
  static_assert((... and std::is_constructible_v<ListType, Lists>));

  return {
      // NOTE: do not forward the first joint_list, otherwise rvalue may be
      // moved away
      ModelDescription<ListType>{
          .urdf = urdf,
          .joint_list = joint_lists,
          .has_free_flyer = false,
      }...,
      ModelDescription<ListType>{
          .urdf = urdf,
          .joint_list = std::forward<Lists>(joint_lists),
          .has_free_flyer = true,
      }...,
  };
}

/// Declaration of the PrintFormat
template <typename Range>
struct ModelDescription<Range>::PrintFormat {
  bool full_urdf = false; /*!< Print the full URDF string */
};

/**
 *  @brief Print a given \a model into \a os
 *
 *  @param[in] model The model we wish to print
 *  @param[inout] os The output stream ptr we wish to print to
 *  @param[in] fmt The format specifier use to print the model
 */
template <typename T>
constexpr auto PrintTo(
    const ModelDescription<T> &model, std::ostream *os,
    typename ModelDescription<T>::PrintFormat fmt = {}) noexcept -> void {
  if (os == nullptr) return;

  *os << "ModelDescription{";

  // Dumb function to retreive the robot name from the URDF
  constexpr auto GetRobotNameFromURDF =
      [](std::string_view urdf) -> std::optional<std::string_view> {
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
  };

  *os << ".urdf = ";
  if (fmt.full_urdf) {
    *os << std::quoted(model.urdf);
  } else if (const auto robot_name = GetRobotNameFromURDF(model.urdf);
             robot_name.has_value()) {
    *os << *robot_name;
  } else {
    *os << "str{";
    *os << ".data() = @ " << (void const *)model.urdf.data() << ", ";
    *os << ".size() = " << model.urdf.size() << ", ";
    *os << "}";
  }
  *os << ", ";

  *os << ".joint_list = ";
  PrintTo(View{model.joint_list}, os);
  *os << ", ";

  *os << ".has_free_flyer = " << model.has_free_flyer;

  *os << "}";
}

/**
 *  @brief Helper function to create a RobotModelBuilder using a given \a model
 *         and a list of Joints to control
 *
 *  @param[in] model The model description (URDF) we wish to build
 *
 *  @return std::unique_ptr<RobotModelBuilder> A valid RobotModelBuilder (i.e.
 *          build_model() returned true), nullptr otherwise
 */
template <typename T>
auto MakeRobotModelBuilderFrom(ModelDescription<T> model)
    -> std::unique_ptr<linear_feedback_controller::RobotModelBuilder> {
  const auto number_of_joints = ssize(View{model.joint_list});
  assert(number_of_joints >= 0 &&
         "Wrong joint_list input range (negative size)");

  auto controlled_joints = std::vector<std::string>{};
  controlled_joints.reserve(number_of_joints);

  auto moving_joints = std::vector<std::string>{};
  moving_joints.reserve(number_of_joints);

  for (const auto &joint : model.joint_list) {
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
}  // namespace tests::utils
