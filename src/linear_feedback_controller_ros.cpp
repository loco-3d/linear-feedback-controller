#include "linear_feedback_controller/linear_feedback_controller_ros.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/helpers.hpp"
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"
#include "pal_statistics/pal_statistics_macros.hpp"

using namespace std::chrono_literals;

namespace linear_feedback_controller {

const std::string LinearFeedbackControllerRos::robot_description_name_ =
    "robot_description";

LinearFeedbackControllerRos::LinearFeedbackControllerRos()
    : ChainableControllerInterface() {}

LinearFeedbackControllerRos::~LinearFeedbackControllerRos() {}

CallbackReturn LinearFeedbackControllerRos::on_init() {
  bool all_good = true;
  std::string robot_description = "";
  if (load_parameters()) {
    return CallbackReturn::FAILURE;
  }
  if (wait_for_robot_description()) {
    return CallbackReturn::FAILURE;
  }
  if (get_robot_description(robot_description)) {
    return CallbackReturn::FAILURE;
  }
  if (load_linear_feedback_controller(robot_description)) {
    return CallbackReturn::FAILURE;
  }
  if (setup_reference_interface()) {
    return CallbackReturn::FAILURE;
  }
  if (allocate_memory()) {
    return CallbackReturn::FAILURE;
  }
  if (initialize_introspection()) {
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
    index = 7;
  }
  const auto joint_nq = lfc_.get_robot_model()->get_joint_nq();
  const auto joint_nv = lfc_.get_robot_model()->get_joint_nv();
  for (auto i = 0; i < joint_nq; ++i) {
    reference_interfaces_[index] = state_msg_.msg_joint_state.position[i];
    ++index;
  }
  if (lfc_.get_robot_model()->get_robot_has_free_flyer()) {
    reference_interfaces_[index + 0] = state_msg_.msg_odom.twist.twist.linear.x;
    reference_interfaces_[index + 1] = state_msg_.msg_odom.twist.twist.linear.y;
    reference_interfaces_[index + 2] = state_msg_.msg_odom.twist.twist.linear.z;
    reference_interfaces_[index + 3] =
        state_msg_.msg_odom.twist.twist.angular.x;
    reference_interfaces_[index + 4] =
        state_msg_.msg_odom.twist.twist.angular.y;
    reference_interfaces_[index + 5] =
        state_msg_.msg_odom.twist.twist.angular.z;
    index += 6;
  }
  for (auto i = 0; i < joint_nv; ++i) {
    assert(index < reference_interfaces_.size() &&
           "Index is greater than the number of reference_interfaces.");
    reference_interfaces_[index] = state_msg_.msg_joint_state.velocity[i];
    ++index;
  }
  return return_type::OK;
}

return_type LinearFeedbackControllerRos::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  // Read the hardware data.
  if (!read_state_from_references()) {
    return return_type::ERROR;
  }
  // publish the state.
  linear_feedback_controller_msgs::sensorEigenToMsg(input_sensor_,
                                                    input_sensor_msg_);
  sensor_publisher_->publish(input_sensor_msg_);

  // Get the current time.
  TimePoint time_lfc = TimePoint(Duration(time.seconds()));

  // Copy the last control received:
  synched_input_control_msg_.mutex.lock();
  input_control_msg_ = synched_input_control_msg_.msg;
  synched_input_control_msg_.mutex.unlock();
  linear_feedback_controller_msgs::controlMsgToEigen(input_control_msg_,
                                                     input_control_);

  // Copy the output of the control in order to log it.
  output_joint_effort_ =
      lfc_.compute_control(time_lfc, input_sensor_, input_control_,
                           parameters_.remove_gravity_compensation_effort);

  // Write the output of the control (joint effort), in the command interface.
  const auto joint_nv = lfc_.get_robot_model()->get_joint_nv();
  for (Eigen::Index i = 0; i < joint_nv; ++i) {
    joint_effort_command_interface_[i].get().set_value(output_joint_effort_[i]);
  }

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
  const auto nq = lfc_.get_robot_model()->get_nq();
  const auto nv = lfc_.get_robot_model()->get_nv();
  const auto joint_nq = lfc_.get_robot_model()->get_joint_nq();
  const auto joint_nv = lfc_.get_robot_model()->get_joint_nv();

  if (lfc_.get_robot_model()->get_robot_has_free_flyer()) {
    input_sensor_.base_pose =
        Eigen::VectorXd::Map(&reference_interfaces_[0], 7);
    input_sensor_.joint_state.position =
        Eigen::VectorXd::Map(&reference_interfaces_[7], joint_nq);
    input_sensor_.base_twist =
        Eigen::VectorXd::Map(&reference_interfaces_[nq], 6);
    new_joint_velocity_ =
        Eigen::VectorXd::Map(&reference_interfaces_[nq + 6], joint_nv);
  } else {
    input_sensor_.base_pose.fill(std::numeric_limits<double>::signaling_NaN());
    input_sensor_.joint_state.position =
        Eigen::VectorXd::Map(&reference_interfaces_[0], nq);
    input_sensor_.base_twist.fill(std::numeric_limits<double>::signaling_NaN());
    new_joint_velocity_ = Eigen::VectorXd::Map(&reference_interfaces_[nq], nv);
  }

  for (Eigen::Index i = 0; i < joint_nv; ++i) {
    input_sensor_.joint_state.velocity(i) = filters::exponentialSmoothing(
        new_joint_velocity_(i), input_sensor_.joint_state.velocity(i),
        parameters_.joint_velocity_filter_coefficient);
  }
  input_sensor_.joint_state.effort.fill(
      std::numeric_limits<double>::signaling_NaN());
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
  register_var(std::string("input_sensor_base_pose"), input_sensor_.base_pose);
  register_var(std::string("input_sensor_base_twist"),
               input_sensor_.base_twist);
  register_var(std::string("input_sensor_joint_state_position"),
               input_sensor_.joint_state.position);
  register_var(std::string("input_sensor_joint_state_velocity"),
               input_sensor_.joint_state.velocity);
  register_var(std::string("input_sensor_joint_state_effort"),
               input_sensor_.joint_state.effort);
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

void LinearFeedbackControllerRos::register_var(const std::string& id,
                                               const Eigen::VectorXd& vec) {
  for (Eigen::Index i = 0; i < vec.size(); ++i) {
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << id << "_" << i;
    REGISTER_VARIABLE(get_node(), "/introspection_data", oss.str(), &vec(i),
                      &bookkeeping_);
  }
}

void LinearFeedbackControllerRos::register_var(
    const std::string& id, const Eigen::Matrix<double, 7, 1>& vec) {
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_x", &vec(0),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_y", &vec(1),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_z", &vec(2),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_qx", &vec(0),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_qy", &vec(1),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_qz", &vec(2),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_qw", &vec(2),
                    &bookkeeping_);
}

void LinearFeedbackControllerRos::register_var(
    const std::string& id, const Eigen::Matrix<double, 6, 1>& vec) {
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_x", &vec(0),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_y", &vec(1),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_z", &vec(2),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_wx", &vec(0),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_wy", &vec(1),
                    &bookkeeping_);
  REGISTER_VARIABLE(get_node(), "/introspection_data", id + "_wz", &vec(2),
                    &bookkeeping_);
}

bool LinearFeedbackControllerRos::wait_for_robot_description() {
  robot_description_node_ = std::make_shared<rclcpp::Node>(
      get_node()->get_name() + std::string("_") + robot_description_name_);
  robot_description_parameter_client_ =
      std::make_shared<rclcpp::SyncParametersClient>(robot_description_node_,
                                                     "robot_state_publisher");
  while (!robot_description_parameter_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "service not available, waiting again...");
  }
  return true;
}

bool LinearFeedbackControllerRos::get_robot_description(
    std::string& robot_description) {
  auto parameters = robot_description_parameter_client_->get_parameters(
      {robot_description_name_});
  robot_description = parameters[0].value_to_string();
  if (robot_description.empty()) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "The " << robot_description_name_ << " parameter is retrieved empty.");
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
  lfc_params.moving_joint_names = parameters_.moving_joint_names;
  lfc_params.controlled_joint_names = parameters_.moving_joint_names;
  lfc_params.p_gains.clear();
  lfc_params.d_gains.clear();
  for (auto joint_name : parameters_.moving_joint_names) {
    lfc_params.p_gains.emplace_back(
        parameters_.moving_joint_names_map.at(joint_name).p);
    lfc_params.d_gains.emplace_back(
        parameters_.moving_joint_names_map.at(joint_name).d);
  }
  lfc_params.default_configuration_name =
      parameters_.default_configuration_name;
  lfc_params.robot_has_free_flyer = parameters_.robot_has_free_flyer;
  lfc_params.pd_to_lf_transition_duration =
      Duration(parameters_.pd_to_lf_transition_duration);
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
  }
  for (const auto& joint : lfc_.get_robot_model()->get_moving_joint_names()) {
    const auto name = command_prefix_ + joint + "/" + HW_IF_POSITION;
    reference_interface_names_.emplace_back(name);
  }
  if (lfc_.get_robot_model()->get_robot_has_free_flyer()) {
    reference_interface_names_.push_back("base_linear_velocity_x");
    reference_interface_names_.push_back("base_linear_velocity_y");
    reference_interface_names_.push_back("base_linear_velocity_z");
    reference_interface_names_.push_back("base_angular_velocity_x");
    reference_interface_names_.push_back("base_angular_velocity_y");
    reference_interface_names_.push_back("base_angular_velocity_z");
  }
  for (const auto& joint : lfc_.get_robot_model()->get_moving_joint_names()) {
    const auto name = command_prefix_ + joint + "/" + HW_IF_VELOCITY;
    reference_interface_names_.emplace_back(name);
  }
  return true;
}

bool LinearFeedbackControllerRos::allocate_memory() {
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

  input_sensor_.joint_state.name =
      lfc_.get_robot_model()->get_moving_joint_names();
  input_sensor_.joint_state.position =
      Eigen::VectorXd::Zero(lfc_.get_robot_model()->get_joint_nq());
  input_sensor_.joint_state.velocity =
      Eigen::VectorXd::Zero(lfc_.get_robot_model()->get_joint_nv());
  input_sensor_.joint_state.effort =
      Eigen::VectorXd::Zero(lfc_.get_robot_model()->get_joint_nv());
  input_sensor_.joint_state.position.fill(
      std::numeric_limits<double>::signaling_NaN());
  input_sensor_.joint_state.velocity.fill(
      std::numeric_limits<double>::signaling_NaN());
  input_sensor_.joint_state.effort.fill(
      std::numeric_limits<double>::signaling_NaN());
  input_sensor_.base_pose.fill(std::numeric_limits<double>::signaling_NaN());
  input_sensor_.base_twist.fill(std::numeric_limits<double>::signaling_NaN());

  new_joint_velocity_ =
      Eigen::VectorXd::Zero(lfc_.get_robot_model()->get_joint_nv());
  new_joint_velocity_.fill(std::numeric_limits<double>::signaling_NaN());

  // Resize the reference interface vector to correspond with the names.
  reference_interfaces_.resize(reference_interface_names_.size(), 0.0);

  // Allocate subscribers
  {
    rclcpp::QoS qos = rclcpp::QoS(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    subscriber_odom_.subscribe(get_node(), "odom", rmw_qos_profile);
    subscriber_joint_state_.subscribe(get_node(), "joint_state",
                                      rmw_qos_profile);
    state_syncher_ = std::make_shared<
        message_filters::TimeSynchronizer<Odometry, JointState>>(
        subscriber_odom_, subscriber_joint_state_, rmw_qos_profile.depth);
  }
  {
    using namespace std::placeholders;
    rclcpp::QoS qos = rclcpp::QoS(10);
    qos.best_effort();
    sensor_publisher_ = get_node()->create_publisher<SensorMsg>("sensor", qos);
    control_subscriber_ = get_node()->create_subscription<ControlMsg>(
        "control", qos,
        std::bind(&LinearFeedbackControllerRos::control_subscription_callback,
                  this, _1));
  }

  return true;
}

bool LinearFeedbackControllerRos::ends_with(const std::string& str,
                                            const std::string& suffix) const {
  return str.size() >= suffix.size() &&
         str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void LinearFeedbackControllerRos::control_subscription_callback(
    const ControlMsg msg) {
  synched_input_control_msg_.mutex.lock();
  synched_input_control_msg_.msg = msg;
  synched_input_control_msg_.mutex.unlock();
}

void LinearFeedbackControllerRos::state_syncher_callback(
    const Odometry::ConstSharedPtr& msg_odom,
    const JointState::ConstSharedPtr& msg_joint_state) {
  synched_state_msg_.mutex.lock();
  synched_state_msg_.msg_joint_state = *msg_joint_state;
  synched_state_msg_.msg_odom = *msg_odom;
  synched_state_msg_.mutex.unlock();
}

// rclcpp_action::GoalResponse LinearFeedbackControllerRos::handle_goal(
//     const rclcpp_action::GoalUUID & uuid,
//     std::shared_ptr<const RunLFC::Goal> goal) {
//   RCLCPP_INFO(this->get_logger(),
//               "Received goal request with order %d", goal->order);
//   (void)uuid;
//   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }

// rclcpp_action::CancelResponse LinearFeedbackControllerRos::handle_cancel(
//   const std::shared_ptr<GoalHandleRunLFC> goal_handle) {
//   RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
//   (void)goal_handle;
//   return rclcpp_action::CancelResponse::ACCEPT;
// }

// void LinearFeedbackControllerRos::handle_accepted(
//   const std::shared_ptr<GoalHandleRunLFC> goal_handle) {
//   action_goal_handle_ = goal_handle;
// }

}  // namespace linear_feedback_controller

PLUGINLIB_EXPORT_CLASS(linear_feedback_controller::LinearFeedbackControllerRos,
                       controller_interface::ChainableControllerInterface);
