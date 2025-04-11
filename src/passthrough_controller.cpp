// Copyright (c) 2023, PAL Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "linear_feedback_controller/passthrough_controller.hpp"

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace linear_feedback_controller {

controller_interface::CallbackReturn PassthroughController::on_init() {
  try {
    param_listener_ =
        std::make_shared<passthrough_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  reference_interface_names_ = params_.reference_interfaces;
  command_interface_names_ = params_.command_interfaces;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PassthroughController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
PassthroughController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn PassthroughController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();
  reference_interface_names_ = params_.reference_interfaces;
  command_interface_names_ = params_.command_interfaces;

  // pre-reserve command interfaces
  command_interfaces_.reserve(command_interface_names_.size());

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

  // for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize(reference_interface_names_.size(),
                               std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PassthroughController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!is_in_chained_mode()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Not in chained mode.");
    return controller_interface::CallbackReturn::ERROR;
  }

  bool ret = controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_names_, std::string(""),
      ordered_command_interfaces_);
  if (!ret ||
      command_interface_names_.size() != ordered_command_interfaces_.size() ||
      command_interface_names_.size() != reference_interfaces_.size()) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Expected %zu command interfaces, got %zu",
                 command_interface_names_.size(),
                 ordered_command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(),
            std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PassthroughController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  ordered_command_interfaces_.clear();
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PassthroughController::on_set_chained_mode(bool /*chained_mode*/) {
  return true;
}

controller_interface::return_type
PassthroughController::update_and_write_commands(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (size_t i = 0; i < ordered_command_interfaces_.size(); ++i) {
    if (!std::isnan(reference_interfaces_[i])) {
      bool ret = ordered_command_interfaces_[i].get().set_value(
          reference_interfaces_[i]);
      if (!ret) {
        RCLCPP_ERROR_STREAM(
            get_node()->get_logger(),
            "Error writing into the command interface : "
                << ordered_command_interfaces_[i].get().get_name());
        return controller_interface::return_type::ERROR;
      }
    } else {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Nan detected in the reference interface : "
                              << reference_interface_names_[i]);
      bool ret = ordered_command_interfaces_[i].get().set_value(0.0);
      return controller_interface::return_type::OK;
    }
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
PassthroughController::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  for (size_t i = 0; i < reference_interface_names_.size(); ++i) {
    reference_interfaces.emplace_back(get_node()->get_name(),
                                      reference_interface_names_[i],
                                      &reference_interfaces_[i]);
  }
  return reference_interfaces;
}

controller_interface::return_type
PassthroughController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return controller_interface::return_type::OK;
}

}  // namespace linear_feedback_controller

PLUGINLIB_EXPORT_CLASS(linear_feedback_controller::PassthroughController,
                       controller_interface::ChainableControllerInterface)
