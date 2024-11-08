// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#include "linear_feedback_controller/linear_feedback_controller_ros.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/helpers.hpp"
#include "pal_statistics/pal_statistics_macros.hpp"

using namespace std::chrono_literals;

namespace linear_feedback_controller {

LinearFeedbackControllerRos::LinearFeedbackControllerRos()
    : ChainableControllerInterface(), qos_(10) {}

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
  const auto num_chainable_interfaces = lfc_.get_robot_model()->get_joint_nv();

  // Dynamic allocation.
  command_interfaces_config_names.reserve(num_chainable_interfaces);
  command_interfaces_config_names.clear();

  // Then the joint informations.
  for (const auto& joint : lfc_.get_robot_model()->get_moving_joint_names()) {
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

  // Position and velocity interfaces for the joints and the freeflyer.
  const auto num_chainable_interfaces = lfc_.get_robot_model()->get_joint_nq() +
                                        lfc_.get_robot_model()->get_joint_nv();

  // Dynamic allocation.
  state_interfaces_config_names.reserve(num_chainable_interfaces);
  state_interfaces_config_names.clear();

  // Then the joint informations.
  for (auto interface : {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT}) {
    for (const auto& joint : lfc_.get_robot_model()->get_moving_joint_names()) {
      const auto name = joint + "/" + interface;
      state_interfaces_config_names.emplace_back(name);
    }
  }

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
  const std::vector<std::string> joint_names =
      lfc_.get_robot_model()->get_moving_joint_names();

  // Testing variable.
  bool all_good = true;

  // Get the state interfaces.
  all_good = controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names, HW_IF_POSITION,
      joint_position_state_interface_);
  all_good &= controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names, HW_IF_VELOCITY,
      joint_velocity_state_interface_);
  all_good &= controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names, HW_IF_EFFORT,
      joint_effort_state_interface_);
  if (!all_good) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Error while assigning the state interfaces.");
    return CallbackReturn::ERROR;
  }

  // Get current joint average position filtered and current averaged torque.
  for (Eigen::Index i = 0; i < init_joint_position_.size(); ++i) {
    init_joint_position_(i) =
        joint_position_state_interface_[i].get().get_value();
  }
  for (Eigen::Index i = 0; i < init_joint_effort_.size(); ++i) {
    init_joint_effort_(i) = joint_effort_state_interface_[i].get().get_value();
  }
  lfc_.set_initial_state(init_joint_position_, init_joint_effort_);

  // Assign the command interface.
  all_good &= controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names, HW_IF_EFFORT,
      joint_effort_command_interface_);

  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(),
            std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(get_node()->get_logger(), "Successfull activation.");
  return CallbackReturn::SUCCESS;
}

bool LinearFeedbackControllerRos::on_set_chained_mode(bool chained_mode) {
  auto callback =
      std::bind(&LinearFeedbackControllerRos::state_syncher_callback, this,
                std::placeholders::_1, std::placeholders::_2);
  if (chained_mode) {
    state_syncher_->registerDropCallback(callback);
  } else {
    state_syncher_->registerCallback(callback);
  }
  return true;
}

CallbackReturn LinearFeedbackControllerRos::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Custom release interfaces for this controller.
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  joint_effort_state_interface_.clear();
  joint_effort_command_interface_.clear();

  // Default release interface from ros2_control.
  release_interfaces();
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
  return CallbackReturn::SUCCESS;
}

return_type LinearFeedbackControllerRos::update_reference_from_subscribers() {
  synched_state_msg_.mutex.lock();
  state_msg_.msg_odom = synched_state_msg_.msg_odom;
  state_msg_.msg_joint_state = synched_state_msg_.msg_joint_state;
  synched_state_msg_.mutex.unlock();

  std::size_t index = 0;
  if (lfc_.get_robot_model()->get_robot_has_free_flyer()) {
    reference_interfaces_[0] = state_msg_.msg_odom.pose.pose.position.x;
    reference_interfaces_[1] = state_msg_.msg_odom.pose.pose.position.y;
    reference_interfaces_[2] = state_msg_.msg_odom.pose.pose.position.z;

    reference_interfaces_[3] = state_msg_.msg_odom.pose.pose.orientation.x;
    reference_interfaces_[4] = state_msg_.msg_odom.pose.pose.orientation.y;
    reference_interfaces_[5] = state_msg_.msg_odom.pose.pose.orientation.z;
    reference_interfaces_[6] = state_msg_.msg_odom.pose.pose.orientation.w;

    reference_interfaces_[7] = state_msg_.msg_odom.twist.twist.linear.x;
    reference_interfaces_[8] = state_msg_.msg_odom.twist.twist.linear.y;
    reference_interfaces_[9] = state_msg_.msg_odom.twist.twist.linear.z;

    reference_interfaces_[10] = state_msg_.msg_odom.twist.twist.angular.x;
    reference_interfaces_[11] = state_msg_.msg_odom.twist.twist.angular.y;
    reference_interfaces_[12] = state_msg_.msg_odom.twist.twist.angular.z;
    index = 13;
  }
  const auto joint_nq = lfc_.get_robot_model()->get_joint_nq();
  const auto joint_nv = lfc_.get_robot_model()->get_joint_nv();
  for (auto i = 0; i < joint_nq; ++i) {
    reference_interfaces_[index] = state_msg_.msg_joint_state.position[i];
    ++index;
  }
  for (auto i = 0; i < joint_nv; ++i) {
    reference_interfaces_[index] = state_msg_.msg_joint_state.velocity[i];
    ++index;
  }
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
  register_var(std::string("output_joint_effort_"), output_joint_effort_);

  if (lfc_.get_robot_model()->get_robot_has_free_flyer()) {
    // register_var(
    //   std::string("base_translation"),
    //   floating_base_estimator_input_.imu_angular_velocity_);
    // register_var(
    //   std::string("base_orientation"),
    //   floating_base_estimator_input_.imu_orientation_);
    // register_var(
    //   std::string("base_linear_velocity"),
    //   floating_base_estimator_input_.imu_angular_velocity_);
    // register_var(
    //   std::string("base_angular_velocity"),
    //   floating_base_estimator_input_.imu_orientation_);
  }
  register_var(std::string("input_robot_configuration"),
               input_robot_configuration_);
  register_var(std::string("input_robot_velocity"), input_robot_velocity_);
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
  if (lfc_.get_robot_model()->get_robot_has_free_flyer()) {
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
  for (const auto& joint : lfc_.get_robot_model()->get_moving_joint_names()) {
    const auto name = command_prefix_ + joint + "/" + HW_IF_POSITION;
    reference_interface_names_.emplace_back(name);
  }
  for (const auto& joint : lfc_.get_robot_model()->get_moving_joint_names()) {
    const auto name = command_prefix_ + joint + "/" + HW_IF_VELOCITY;
    reference_interface_names_.emplace_back(name);
  }

  return true;
}

bool LinearFeedbackControllerRos::allocate_memory() {
  input_robot_configuration_.resize(lfc_.get_robot_model()->get_nq());
  input_robot_velocity_.resize(lfc_.get_robot_model()->get_nv());

  output_joint_effort_ =
      Eigen::VectorXd::Zero(lfc_.get_robot_model()->get_joint_nv());
  joint_effort_command_interface_.reserve(
      lfc_.get_robot_model()->get_joint_nv());
  joint_effort_command_interface_.clear();

  if (parameters_.chainable_controller.command_prefix.empty()) {
    command_prefix_ = "";
  } else if (!ends_with(parameters_.chainable_controller.command_prefix, "/")) {
    command_prefix_ = parameters_.chainable_controller.command_prefix + "/";
  } else {
    command_prefix_ = parameters_.chainable_controller.command_prefix;
  }

  init_joint_position_ =
      Eigen::VectorXd::Zero(lfc_.get_robot_model()->get_joint_nq());
  init_joint_effort_ =
      Eigen::VectorXd::Zero(lfc_.get_robot_model()->get_joint_nv());

  // Resize the reference interface vector to correspond with the names.
  reference_interfaces_.resize(reference_interface_names_.size(), 0.0);

  // Allocate subscribers
  auto rmw_qos_profile = qos_.get_rmw_qos_profile();
  subscriber_odom_.subscribe(get_node(), "odom", rmw_qos_profile);
  subscriber_joint_state_.subscribe(get_node(), "joint_state", rmw_qos_profile);
  state_syncher_ =
      std::make_shared<message_filters::TimeSynchronizer<Odometry, JointState>>(
          subscriber_odom_, subscriber_joint_state_, rmw_qos_profile.depth);
  return true;
}

bool LinearFeedbackControllerRos::ends_with(const std::string& str,
                                            const std::string& suffix) const {
  return str.size() >= suffix.size() &&
         str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void LinearFeedbackControllerRos::state_syncher_callback(
    const Odometry::ConstSharedPtr& msg_odom,
    const JointState::ConstSharedPtr& msg_joint_state) {
  synched_state_msg_.mutex.lock();
  synched_state_msg_.msg_joint_state = *msg_joint_state;
  synched_state_msg_.msg_odom = *msg_odom;
  synched_state_msg_.mutex.unlock();
}

}  // namespace linear_feedback_controller
