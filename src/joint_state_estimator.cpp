#include "linear_feedback_controller/joint_state_estimator.hpp"

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace linear_feedback_controller {

controller_interface::CallbackReturn JointStateEstimator::on_init() {
  try {
    param_listener_ =
        std::make_shared<joint_state_estimator::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointStateEstimator::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = params_.command_interfaces;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
JointStateEstimator::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_config.names = params_.state_interfaces;
  return state_interface_config;
}

controller_interface::CallbackReturn JointStateEstimator::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();
  // Pre-reserve command/state interfaces.
  state_interfaces_.reserve(params_.state_interfaces.size());
  command_interfaces_.reserve(params_.command_interfaces.size());
  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStateEstimator::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Check of we have access to all command interface.
  bool ret = controller_interface::get_ordered_interfaces(
    command_interfaces_, params_.command_interfaces, std::string(""),
    command_ordered_interfaces_);
  
    if(!ret ||
      params_.command_interfaces.size() != command_ordered_interfaces_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu command interfaces, got %zu",
                 params_.command_interfaces.size(),
                 command_ordered_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Check if we have access to all state interfaces.
  ret = controller_interface::get_ordered_interfaces(
    state_interfaces_, params_.state_interfaces, std::string(""),
    state_ordered_interfaces_);
  if (!ret ||
          params_.state_interfaces.size() !=
          state_ordered_interfaces_.size()) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Expected %zu state interfaces, got %zu",
                 params_.state_interfaces.size(),
                 state_ordered_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStateEstimator::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  command_ordered_interfaces_.clear();
  state_ordered_interfaces_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointStateEstimator::update(
    const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    double state_interface_value = state_interfaces_[i].get_value();
    if (!std::isnan(state_interface_value)) {
      bool ret = command_interfaces_[i].set_value(state_interface_value);
      if(!ret)
      {
        RCLCPP_ERROR_STREAM(get_node()->get_logger(),
          "Problem writing in the robot command interface : "
          << command_interfaces_[i].get_name());
        return controller_interface::return_type::ERROR;  
      }
    } else {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
        "Nan detected in the robot state interface : "
        << state_interfaces_[i].get_name());
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace linear_feedback_controller

PLUGINLIB_EXPORT_CLASS(linear_feedback_controller::JointStateEstimator,
                       controller_interface::ControllerInterface)
