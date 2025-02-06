#ifndef LINEAR_FEEDBACK_CONTROLLER_TESTS__LINEAR_FEEDBACK_CONTROLLER_HPP_
#define LINEAR_FEEDBACK_CONTROLLER_TESTS__LINEAR_FEEDBACK_CONTROLLER_HPP_

#include <chrono>  // duration_cast/milliseconds

#include "linear_feedback_controller/linear_feedback_controller.hpp"
#include "pd_controller.hpp"
#include "robot_model.hpp"

namespace tests::utils {

inline auto MakeAllControllerParametersFrom(
    std::string_view urdf,
    std::initializer_list<std::vector<JointDescription>> all_joint_lists,
    std::initializer_list<linear_feedback_controller::Duration> durations)
    -> std::vector<linear_feedback_controller::ControllerParameters> {
  using linear_feedback_controller::ControllerParameters;

  const auto FromEigen = [](const Eigen::VectorXd& val) -> std::vector<double> {
    std::vector<double> out(val.size());
    Eigen::Map<Eigen::VectorXd>(out.data(), out.size()) = val;
    return out;
  };

  std::vector<ControllerParameters> out;

  out.reserve(2 * all_joint_lists.size() * durations.size());
  for (const auto& joint_list : all_joint_lists) {
    const auto [controlled, moving] = JointNamesPair::From(joint_list);

    const auto gains = Gains::Random(controlled.size());
    for (const auto& duration : durations) {
      for (const auto has_free_flyer : {false, true}) {
        out.emplace_back(ControllerParameters{
            .urdf = std::string{urdf},
            .moving_joint_names = moving,
            .p_gains = FromEigen(gains.p),
            .d_gains = FromEigen(gains.d),
            .controlled_joint_names = controlled,
            .robot_has_free_flyer = has_free_flyer,
            .pd_to_lf_transition_duration = duration,
        });
      }
    }
  }

  return out;
}

/// PrintTo Format used by PrintTo
struct ControllerParametersPrintFormat {
  tests::utils::ModelDescription::PrintFormat model = {};
  bool as_param_name = false;
};

}  // namespace tests::utils

// For ADL
namespace linear_feedback_controller {

inline auto PrintTo(const ControllerParameters& params, std::ostream* os,
                    tests::utils::ControllerParametersPrintFormat fmt = {})
    -> void {
  if (os == nullptr) return;

  using tests::utils::MakeJointDescriptionListFrom;
  const auto model = tests::utils::ModelDescription{
      .urdf = params.urdf,
      .joint_list = MakeJointDescriptionListFrom(params.controlled_joint_names,
                                                 params.moving_joint_names),
      .has_free_flyer = params.robot_has_free_flyer,
  };

  constexpr auto ToEigen = [](const std::vector<double>& v) {
    return Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
  };

  const auto gains = tests::utils::Gains{
      .p = ToEigen(params.p_gains),
      .d = ToEigen(params.d_gains),
  };

  using std::chrono::duration_cast;
  using std::chrono::milliseconds;
  const auto duration =
      duration_cast<milliseconds>(params.pd_to_lf_transition_duration).count();

  if (not fmt.as_param_name) {
    *os << "ControllerParameters{";

    PrintTo(model, os, fmt.model);
    *os << ", ";

    PrintTo(gains, os);
    *os << ", ";

    *os << ".pd_to_lf_transition_duration = " << duration << "ms ";

    *os << "}";
  } else {
    fmt.model.as_param_name = true;
    PrintTo(model, os, fmt.model);
    // TODO gains ?
    *os << "_" << duration << "ms";
  }
}

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_TESTS__LINEAR_FEEDBACK_CONTROLLER_HPP_
