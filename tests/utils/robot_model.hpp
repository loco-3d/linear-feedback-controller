#ifndef LINEAR_FEEDBACK_CONTROLLER_TESTS__ROBOT_MODEL_HPP_
#define LINEAR_FEEDBACK_CONTROLLER_TESTS__ROBOT_MODEL_HPP_

#include <initializer_list>
#include <iomanip>  // std::quoted
#include <memory>
#include <optional>
#include <ostream>
#include <string_view>
#include <vector>

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

  *os << ".type = ";
  PrintTo(joint.type, os);
  *os << ", ";

  *os << "}";
}

struct JointNamesPair {
  std::vector<std::string> controlled;
  std::vector<std::string> moving;

  /**
   *  @brief Create a pair of names for each JointType
   *
   *  @param[in] joint_desc_list List of JointDescription
   *
   *  @return JointNamesPair Pair of name list
   */
  static inline auto From(const std::vector<JointDescription> &joint_desc_list)
      -> JointNamesPair {
    JointNamesPair out;

    out.controlled.reserve(joint_desc_list.size());
    out.moving.reserve(joint_desc_list.size());

    for (const auto &joint : joint_desc_list) {
      if (IsControlled(joint.type)) {
        out.controlled.emplace_back(joint.name);
      }

      if (IsMoving(joint.type)) {
        out.moving.emplace_back(joint.name);
      }
    }

    return out;
  }
};

/**
 *  @brief Create a list of JointDescription from the given lists
 *
 *  @param[in] controlled List of controlled joint names
 *  @param[in] moving List of moving joint names
 */
inline auto MakeJointDescriptionListFrom(
    const std::vector<std::string> &controlled,
    const std::vector<std::string> &moving) -> std::vector<JointDescription> {
  std::vector<JointDescription> out;

  // The worst case would be that each names are differents
  out.reserve(moving.size() + controlled.size());

  for (std::string_view name : moving) {
    out.emplace_back(JointDescription{
        .name = name,
        .type = JointType::Moving,
    });
  }

  for (std::string_view name : controlled) {
    // If the name has already been pushed into out, update the type to Both
    auto found = std::find_if(out.begin(), out.end(),
                              [name](const JointDescription &joint_desc) {
                                return joint_desc.name == name;
                              });

    if (found != out.end()) {
      found->type = JointType::Both;
    } else {
      out.emplace_back(JointDescription{
          .name = name,
          .type = JointType::Controlled,
      });
    }
  }

  return out;
}

/// Global information about the model we wish to create
struct ModelDescription {
  /// PrintFormat used by PrintTo to format a Model
  struct PrintFormat;

  std::string_view urdf; /*!< Complete Robot URDF description */
  std::vector<JointDescription> joint_list; /*!< Joint description list */
  bool has_free_flyer; /*!< Indicates if the model uses free flyer */
};

/**
 *  @brief Create a vector of ModelDescription with and without free flyer,
 *         for each joint_list provided
 *
 *  @param[in] urdf The common URDF used by every ModelDescription
 *  @param[in] all_joint_lists All arguments forwarded to the '.joint_list =
 * ' constructor
 *
 *  @return std::vector<ModelDescription> Containing all
 *          ModelDescriptions constructed
 */
inline auto MakeAllModelDescriptionsFor(
    std::string_view urdf,
    std::initializer_list<std::vector<JointDescription>> all_joint_lists)
    -> std::vector<ModelDescription> {
  std::vector<ModelDescription> out;
  out.reserve(2 * all_joint_lists.size());

  for (const auto &joint_list : all_joint_lists) {
    out.emplace_back(ModelDescription{
        .urdf = urdf,
        .joint_list = joint_list,
        .has_free_flyer = false,
    });
    out.emplace_back(ModelDescription{
        .urdf = urdf,
        .joint_list = std::move(joint_list),
        .has_free_flyer = true,
    });
  }

  return out;
}

/// Declaration of the PrintFormat
struct ModelDescription::PrintFormat {
  bool as_param_name = false; /*!< Use the gtest param name paradigm */
  bool full_urdf = false;     /*!< Print the full URDF string */
};

/**
 *  @brief Print a given \a model into \a os
 *
 *  @param[in] model The model we wish to print
 *  @param[inout] os The output stream ptr we wish to print to
 *  @param[in] fmt The format specifier use to print the model
 */
inline auto PrintTo(const ModelDescription &model, std::ostream *os,
                    typename ModelDescription::PrintFormat fmt = {}) noexcept
    -> void {
  if (os == nullptr) return;

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

  if (not fmt.as_param_name) {
    *os << "ModelDescription{";

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

    *os << ".joint_list = [ ";
    for (const auto &joint : model.joint_list) {
      PrintTo(joint, os);
      *os << ", ";
    }
    *os << "], ";

    *os << ".has_free_flyer = " << model.has_free_flyer;

    *os << "}";
  } else {
    // GTest Param name forbids with space etc...

    // TODO: Add URDF ?

    if (model.has_free_flyer) {
      *os << "FreeFlyer_";
    }

    *os << model.joint_list.size() << "_Joints";
    for (const auto &[name, type] : model.joint_list) {
      *os << "_" << name << "_" << ToString(type);
    }
  }
}

/**
 *  @brief Helper function to create a RobotModelBuilder using a given \a model
 *
 *  @param[in] model The model description (URDF) we wish to build
 *
 *  @return std::unique_ptr<RobotModelBuilder> A valid RobotModelBuilder
 *          (i.e. build_model() returned true), nullptr otherwise
 */
inline auto MakeRobotModelBuilderFrom(const ModelDescription &model)
    -> std::unique_ptr<linear_feedback_controller::RobotModelBuilder> {
  auto [controlled, moving] = JointNamesPair::From(model.joint_list);
  auto rmb = std::make_unique<linear_feedback_controller::RobotModelBuilder>();
  if (rmb->build_model(std::string{model.urdf}, std::move(moving),
                       std::move(controlled), model.has_free_flyer)) {
    return rmb;
  } else {
    return nullptr;
  }
}
}  // namespace tests::utils

#endif  // LINEAR_FEEDBACK_CONTROLLER_TESTS__ROBOT_MODEL_HPP_
