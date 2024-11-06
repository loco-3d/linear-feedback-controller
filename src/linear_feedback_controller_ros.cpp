// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#include "linear_feedback_controller/linear_feedback_controller_ros.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pal_statistics/pal_statistics_macros.hpp"

using namespace std::chrono_literals;

namespace linear_feedback_controller {

LinearFeedbackControllerRos::LinearFeedbackControllerRos()
    : ChainableControllerInterface() {}

LinearFeedbackControllerRos::~LinearFeedbackControllerRos() {}

CallbackReturn LinearFeedbackControllerRos::on_init() {
  bool all_good = true;
  std::string robot_description = "";

  all_good &= load_parameters();
  all_good &= wait_for_robot_description(robot_description);
  all_good &= load_linear_feedback_controller(robot_description);
  all_good &= setup_reference_interface();
  all_good &= allocate_memory();
  all_good &= initialize_introspection();
  RCLCPP_INFO(get_node()->get_logger(), "Successfull init.");
  if (!all_good) {
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration
LinearFeedbackControllerRos::command_interface_configuration() const {
  // Output the estimated robot state as command interface.
  std::vector<std::string> command_interfaces_config_names = {};

  // Position and velocity interfaces for the joints and the freeflyer.
  const auto num_chainable_interfaces = lfc_.getRobotModel()->getJointNv();

  // Dynamic allocation.
  command_interfaces_config_names.reserve(num_chainable_interfaces);
  command_interfaces_config_names.clear();

  // Then the joint informations.
  for (const auto& joint : lfc_.getRobotModel()->get_moving_joint_names()) {
    const auto name = command_prefix_ + joint + "/" + HW_IF_EFFORT;
    command_interfaces_config_names.emplace_back(name);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interfaces_config_names};
}

InterfaceConfiguration
LinearFeedbackControllerRos::state_interface_configuration() const {
  // Get the joint state measurements.
  std::vector<std::string> state_interfaces_config_names;
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          state_interfaces_config_names};
}

std::vector<hardware_interface::CommandInterface>
LinearFeedbackControllerRos::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  for (size_t i = 0; i < reference_interface_names_.size(); ++i) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i],
        &reference_interfaces_[i]));
  }
  return reference_interfaces;
}

CallbackReturn LinearFeedbackControllerRos::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "Successfull configuration.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LinearFeedbackControllerRos::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // bool all_good = controller_interface::get_ordered_interfaces(
  //   state_interfaces_, robot_model_->getJointNames(),
  //   HW_IF_POSITION, joint_position_state_interface_);
  // all_good &= controller_interface::get_ordered_interfaces(
  //   state_interfaces_, robot_model_->getJointNames(),
  //   HW_IF_VELOCITY, joint_velocity_state_interface_);
  // all_good &= imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  // for (auto & ft_sensor : ft_sensors_) {
  //   all_good &= ft_sensor->assign_loaned_state_interfaces(state_interfaces_);
  // }

  // if (!all_good) {
  //   RCLCPP_ERROR_STREAM(
  //     get_node()->get_logger(), "Error while assigning the state
  //     interfaces.");
  //   return CallbackReturn::ERROR;
  // }

  // // Assign the command interface.
  // all_good &= base_command_interface_->assign_loaned_interfaces(
  //   command_interfaces_);
  // all_good &= controller_interface::get_ordered_interfaces(
  //   command_interfaces_, robot_model_->getJointNames(),
  //   HW_IF_POSITION, joint_position_command_interface_);
  // all_good &= controller_interface::get_ordered_interfaces(
  //   command_interfaces_, robot_model_->getJointNames(),
  //   HW_IF_VELOCITY, joint_velocity_command_interface_);

  // // Initialize states.
  // if (!read_state_from_hardware(
  //     input_joint_state_, input_imu_state_, input_ft_state_))
  // {
  //   RCLCPP_ERROR(
  //     get_node()->get_logger(),
  //     (std::ostringstream("Failed to read from the hardware.\n")
  //       << input_has_nan_).str().c_str());
  //   return controller_interface::CallbackReturn::ERROR;
  // }
  // convert_input_to_eigen_objects();

  // // Initialize the estimator.
  // eVector3 dummy_base_position = Eigen::Vector3d::Zero();
  // floating_base_estimator_->setInitialState(
  //   // Not used here. It assumes the feet are on the ground.
  //   dummy_base_position,
  //   floating_base_estimator_input_.imu_orientation_,
  //   floating_base_estimator_input_.joint_position_);

  // // Initilize the contact feet.
  // floating_base_estimator_input_.in_contact_feet_ =
  //   contact_estimator_->getStanceFootNames();
  // floating_base_estimator_input_.swing_feet_ =
  //   contact_estimator_->getSwingFootNames();

  // // Reset the timers.
  // time_profiler_.resetTimers();

  RCLCPP_INFO(get_node()->get_logger(), "Successfull activation.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LinearFeedbackControllerRos::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // imu_sensor_->release_interfaces();
  // for (auto & ft_sensor : ft_sensors_) {
  //   ft_sensor->release_interfaces();
  // }
  // joint_position_state_interface_.clear();
  // joint_velocity_state_interface_.clear();

  // joint_position_command_interface_.clear();
  // joint_velocity_command_interface_.clear();
  // base_command_interface_->release_interfaces();

  // // Default release interface from ros2_control.
  // release_interfaces();
  RCLCPP_INFO(get_node()->get_logger(), "Successfull desactivation.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LinearFeedbackControllerRos::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // if (parameters_.floating_base_state_estimator.spawn_thread) {
  //   floating_base_estimator_->joinThread();
  // }
  RCLCPP_INFO(get_node()->get_logger(), "Successfull cleanup.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LinearFeedbackControllerRos::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // if (!floating_base_estimator_) {
  //   RCLCPP_ERROR(
  //     get_node()->get_logger(),
  //     "The estimator plugin where not loaded or initialized.");
  //   return CallbackReturn::ERROR;
  // }

  /// @todo manage error

  return CallbackReturn::SUCCESS;
}

/// @brief ChainableControllerInterface::update_reference_from_subscribers
return_type LinearFeedbackControllerRos::update_reference_from_subscribers() {
  return return_type::OK;
}

return_type LinearFeedbackControllerRos::update_and_write_commands(
    const rclcpp::Time& /* time */, const rclcpp::Duration& /*period*/) {
  // RCLCPP_WARN(get_node()->get_logger(), "Update the state estimation.");

  // // Start recording time.
  // time_profiler_.startTimer(estimator_timer_name_);

  // // Read the hardware data.
  // if (!read_state_from_hardware(
  //     input_joint_state_, input_imu_state_, input_ft_state_))
  // {
  //   return return_type::ERROR;
  // }
  // convert_input_to_eigen_objects();
  // estimate_contacts(time);
  // estimate_base(time);

  // // Write the results of the estimator in the command interface.
  // const EstimatorOutput & output = floating_base_estimator_output_;
  // // Base 6D pose
  // command_interfaces_[0].set_value(output.position_.x());
  // command_interfaces_[1].set_value(output.position_.y());
  // command_interfaces_[2].set_value(output.position_.z());
  // command_interfaces_[3].set_value(output.orientation_.x());
  // command_interfaces_[4].set_value(output.orientation_.y());
  // command_interfaces_[5].set_value(output.orientation_.z());
  // command_interfaces_[6].set_value(output.orientation_.w());
  // // Base 6D velocity
  // command_interfaces_[7].set_value(output.linear_velocity_.x());
  // command_interfaces_[8].set_value(output.linear_velocity_.y());
  // command_interfaces_[9].set_value(output.linear_velocity_.z());
  // command_interfaces_[10].set_value(output.angular_velocity_.x());
  // command_interfaces_[11].set_value(output.angular_velocity_.y());
  // command_interfaces_[12].set_value(output.angular_velocity_.z());
  // std::size_t index = 13;
  // // Joint positions.
  // for (std::size_t i = 0; i < robot_model_->getJointStateSize(); ++i) {
  //   // joint_command_interface_[0][index] = output.joint_position_[i];
  //   ++index;
  // }
  // // Joint velocity.
  // for (std::size_t i = 0; i <
  // robot_model_->getJointStateSize(rpmc::Type::VELOCITY); ++i) {
  //   // joint_command_interface_[1][index] = output.joint_velocity_[i];
  //   ++index;
  // }

  // // Stop recoding time.
  // time_profiler_.stopTime(estimator_timer_name_);
  return return_type::OK;
}

bool LinearFeedbackControllerRos::update_parameters() {
  bool needs_update = parameter_listener_->is_old(parameters_);
  if (needs_update) {
    parameters_ = parameter_listener_->get_params();
  }
  return needs_update;
}

bool LinearFeedbackControllerRos::read_state_from_references() {
  // std::size_t num_joint = robot_model_->getJointStateSize();
  // for (size_t joint_ind = 0; joint_ind < num_joint; ++joint_ind) {
  //   input_joint_state.position[joint_ind] =
  //     state_interfaces_[joint_ind].get_value();
  //   input_joint_state.velocity[joint_ind] =
  //     state_interfaces_[num_joint + joint_ind].get_value();
  //   input_joint_state.effort[joint_ind] = 0.0;
  // }
  // input_has_nan_.joint_.hasNan(input_joint_state);

  // // If any IMU values are nan, assume values are zero.
  // imu_sensor_->get_values_as_message(input_imu_state);

  // // If any Force Torque values are nan, assume values are zero.
  // for (std::size_t i = 0; i < ft_sensors_.size(); ++i) {
  //   ft_sensors_[i]->get_values_as_message(input_ft_state[i]);
  // }

  // if (input_has_nan_.hasNan()) {
  //   RCLCPP_ERROR(
  //     get_node()->get_logger(),
  //     (std::ostringstream("Failed to read from the hardware.\n")
  //       << input_has_nan_).str().c_str());
  // }
  // return !input_has_nan_.hasNan();
  return true;
}

bool LinearFeedbackControllerRos::initialize_introspection() {
  // register_var(
  //   std::string("estimator_input_imu_linear_acceleration"),
  //   floating_base_estimator_input_.imu_linear_acceleration_);
  // register_var(
  //   std::string("estimator_input_imu_angular_velocity"),
  //   floating_base_estimator_input_.imu_angular_velocity_);
  // register_var(
  //   std::string("estimator_input_imu_orientation"),
  //   floating_base_estimator_input_.imu_orientation_);
  // register_var(
  //   std::string("estimator_input_joint_position"),
  //   floating_base_estimator_input_.joint_position_);
  // register_var(
  //   std::string("estimator_input_joint_velocity"),
  //   floating_base_estimator_input_.joint_velocity_);
  return true;
}

void LinearFeedbackControllerRos::register_var(const std::string& id,
                                               const Eigen::Vector3d& vec) {
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_x", &vec.x(),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_y", &vec.y(),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_z", &vec.z(),
                    &bookkeeping_);
}

void LinearFeedbackControllerRos::register_var(const std::string& id,
                                               const Eigen::Quaterniond& quat) {
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_x", &quat.x(),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_y", &quat.y(),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_z", &quat.z(),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_w", &quat.w(),
                    &bookkeeping_);
}

void LinearFeedbackControllerRos::register_var(const std::string& id,
                                               const std::vector<double>& vec) {
  for (std::size_t i = 0; i < vec.size(); ++i) {
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << id << "_" << i;
    REGISTER_VARIABLE(get_node(), "/introspection_data", oss.str(), &vec[i],
                      &bookkeeping_);
  }
}

bool LinearFeedbackControllerRos::wait_for_robot_description(
    std::string& robot_description) {
  auto new_node = std::make_shared<rclcpp::Node>(
      get_node()->get_name() + std::string("_robot_description"));
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      new_node, "robot_state_publisher");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "service not available, waiting again...");
  }
  auto parameters = parameters_client->get_parameters({"robot_description"});
  robot_description = parameters[0].value_to_string();
  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "The robot_description parameter is retrieved empty.");
    return false;
  }
  return true;
}

bool LinearFeedbackControllerRos::load_parameters() {
  try {
    parameter_listener_ =
        std::make_shared<linear_feedback_controller::ParamListener>(get_node());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown during the loading of the parameters: %s \n",
                 e.what());
    return false;
  }
  parameters_ = parameter_listener_->get_params();
  return true;
}

bool LinearFeedbackControllerRos::load_linear_feedback_controller(
    const std::string& robot_description) {
  ControllerParameters lfc_params;
  lfc_params.urdf = robot_description;
  lfc_params.srdf = parameters_.srdf;
  lfc_params.moving_joint_names = parameters_.moving_joint_names;
  lfc_params.controlled_joint_names = parameters_.moving_joint_names;
  lfc_params.p_gains = parameters_.p_gains;
  lfc_params.d_gains = parameters_.d_gains;
  lfc_params.default_configuration_name =
      parameters_.default_configuration_name;
  lfc_params.robot_has_free_flyer = parameters_.robot_has_free_flyer;
  lfc_params.from_pd_to_lf_duration =
      Duration(parameters_.from_pd_to_lf_duration);
  lfc_.load(lfc_params);

  return true;
}

bool LinearFeedbackControllerRos::setup_reference_interface() {
  // Setup the reference interface memory
  reference_interface_names_.clear();
  if (lfc_.getRobotModel()->get_robot_has_free_flyer()) {
    reference_interface_names_.push_back("base_translation_x");
    reference_interface_names_.push_back("base_translation_y");
    reference_interface_names_.push_back("base_translation_z");
    reference_interface_names_.push_back("base_orientation_qx");
    reference_interface_names_.push_back("base_orientation_qy");
    reference_interface_names_.push_back("base_orientation_qz");
    reference_interface_names_.push_back("base_orientation_qw");
    reference_interface_names_.push_back("base_linear_velocity_x");
    reference_interface_names_.push_back("base_linear_velocity_y");
    reference_interface_names_.push_back("base_linear_velocity_z");
    reference_interface_names_.push_back("base_angular_velocity_x");
    reference_interface_names_.push_back("base_angular_velocity_y");
    reference_interface_names_.push_back("base_angular_velocity_z");
  }
  for (const auto& joint : lfc_.getRobotModel()->get_moving_joint_names()) {
    const auto name = command_prefix_ + joint + "/" + HW_IF_POSITION;
    reference_interface_names_.emplace_back(name);
  }
  for (const auto& joint : lfc_.getRobotModel()->get_moving_joint_names()) {
    const auto name = command_prefix_ + joint + "/" + HW_IF_VELOCITY;
    reference_interface_names_.emplace_back(name);
  }

  return true;
}

bool LinearFeedbackControllerRos::allocate_memory() {
  // // Allocate dynamic memory
  // robot_state_ = robot_model_->createZeroState();
  // input_joint_state_.name = robot_model_->getJointNames();
  // double a_nan = std::numeric_limits<double>::quiet_NaN();
  // std::size_t num_joint = robot_model_->getJointNames().size();
  // // Initialize the HasNan structure.
  // input_has_nan_.joint_.reserve(num_joint);
  // // Inititalize the joint state with nan.
  // input_joint_state_.position.resize(num_joint, a_nan);
  // input_joint_state_.velocity.resize(num_joint, a_nan);
  // input_joint_state_.effort.resize(num_joint, a_nan);
  // // Inititalize the imu input with nan.
  // input_imu_state_.orientation.x = a_nan;
  // input_imu_state_.orientation.y = a_nan;
  // input_imu_state_.orientation.z = a_nan;
  // input_imu_state_.orientation.w = a_nan;
  // input_imu_state_.angular_velocity.x = a_nan;
  // input_imu_state_.angular_velocity.y = a_nan;
  // input_imu_state_.angular_velocity.z = a_nan;
  // input_imu_state_.linear_acceleration.x = a_nan;
  // input_imu_state_.linear_acceleration.y = a_nan;
  // input_imu_state_.linear_acceleration.z = a_nan;

  // const auto num_chainable_interfaces =
  //   robot_model_->getConfigurationVectorSize() +
  //   robot_model_->getVelocityVectorSize();
  // command_interfaces_.reserve(num_chainable_interfaces);
  // command_interfaces_.clear();

  // if (parameters_.chainable_controller.command_prefix.empty()) {
  //   command_prefix_ = "";
  // } else {
  //   command_prefix_ = parameters_.chainable_controller.command_prefix + "/";
  // }

  // base_command_interface_ =
  //   std::make_unique<OdometryCommandInterface>(command_prefix_ + "base");

  // Resize the reference interface vector to correspond with the names.
  reference_interfaces_.resize(reference_interface_names_.size(), 0.0);

  return true;
}

}  // namespace linear_feedback_controller
