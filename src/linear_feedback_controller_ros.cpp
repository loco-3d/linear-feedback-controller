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
  std::string robot_description = "";
  if (!load_parameters()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Issues loading the parameters.");
    return CallbackReturn::FAILURE;
  }
  if (!wait_for_robot_description()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Issues waiting for the robot_description parameter service.");
    return CallbackReturn::FAILURE;
  }
  if (!get_robot_description(robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Issues loading the robot_description.");
    return CallbackReturn::FAILURE;
  }
  if (!load_linear_feedback_controller(robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Issues loading the linear feedback controller");
    return CallbackReturn::FAILURE;
  }
  if (!setup_reference_interface()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Issues setting up the reference interfaces.");
    return CallbackReturn::FAILURE;
  }
  if (!allocate_memory()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Issues allocating the memory");
    return CallbackReturn::FAILURE;
  }
  if (!initialize_introspection()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Issues initializing the introspection.");
    return CallbackReturn::FAILURE;
  }
  if (parameters_.chainable_controller.command_interfaces.size() !=
      lfc_.get_robot_model()->get_moving_joint_names().size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter chainable_controller.command_interfaces do not "
                 "have the correct size.");
    return CallbackReturn::FAILURE;
  }
  first_time_update_and_write_commands_ = true;
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
  for (const auto& cin : parameters_.chainable_controller.command_interfaces) {
    command_interfaces_config_names.emplace_back(cin);
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
  reference_interfaces.clear();
  for (size_t i = 0; i < reference_interface_names_.size(); ++i) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i],
        &reference_interfaces_[i]));
  }
  return reference_interfaces;
}

CallbackReturn LinearFeedbackControllerRos::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  first_time_update_and_write_commands_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn LinearFeedbackControllerRos::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if(!is_in_chained_mode())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Wrong activation, the controller needs to be in chain mode.");
  }

  const std::vector<std::string> joint_names =
      lfc_.get_robot_model()->get_moving_joint_names();

  // Testing variable.
  bool all_good = true;

  // Assign the command interface.
  all_good &= controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names, HW_IF_EFFORT,
      joint_effort_command_interface_);

  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(),
            std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(get_node()->get_logger(), "Successful activation.");
  return CallbackReturn::SUCCESS;
}

bool LinearFeedbackControllerRos::on_set_chained_mode(bool /* chained_mode */) {
  return true;
}

CallbackReturn LinearFeedbackControllerRos::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Custom release interfaces for this controller.
  joint_effort_command_interface_.clear();

  // Default release interface from ros2_control.
  release_interfaces();
  RCLCPP_INFO(get_node()->get_logger(), "Successful deactivation.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LinearFeedbackControllerRos::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // if (parameters_.floating_base_state_estimator.spawn_thread) {
  //   floating_base_estimator_->joinThread();
  // }
  RCLCPP_INFO(get_node()->get_logger(), "Successful cleanup.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LinearFeedbackControllerRos::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

// master (jazzy) version 01/03/2025
#if CONTROLLER_INTERFACE_VERSION_AT_LEAST(4, 0, 0)
return_type LinearFeedbackControllerRos::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
#else  // humble version
return_type LinearFeedbackControllerRos::update_reference_from_subscribers()
#endif
{
  return return_type::OK;
}

return_type LinearFeedbackControllerRos::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  
  // Read the hardware data.
  if (!read_state_from_references()) {
    return return_type::ERROR;
  }
  input_sensor_.stamp = time;
  
  // First time executing here.
  if (first_time_update_and_write_commands_) {
    // Get current joint average position filtered and current averaged torque.
    lfc_.set_initial_state(
      input_sensor_.joint_state.effort, input_sensor_.joint_state.position);
    first_time_update_and_write_commands_ = false;
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
  if (input_control_msg_.initial_state.joint_state.name.size() != 0) {
    linear_feedback_controller_msgs::controlMsgToEigen(input_control_msg_,
                                                       input_control_);
  }
  // Copy the output of the control in order to log it.
  output_joint_effort_ =
      lfc_.compute_control(time_lfc, input_sensor_, input_control_,
                           parameters_.remove_gravity_compensation_effort);

  if(output_joint_effort_.hasNaN())
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
        "NaN detect in output joint effort command: "
        << output_joint_effort_.transpose());
      return controller_interface::return_type::ERROR;
  }

  // Write the output of the control (joint effort), in the command interface.
  const auto joint_nv = lfc_.get_robot_model()->get_joint_nv();
  for (Eigen::Index i = 0; i < joint_nv; ++i) {
    bool ret = joint_effort_command_interface_[i].get().set_value(
      output_joint_effort_[i]);
    
    if(!ret){
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
        "Error in writing command value [" << output_joint_effort_[i]
        << "] at this interface: ["
        << joint_effort_command_interface_[i].get().get_name() << "]");
      return controller_interface::return_type::ERROR;
    }
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
    input_sensor_.joint_state.effort =
        Eigen::VectorXd::Map(&reference_interfaces_[nq + nv], joint_nv);
  } else {
    input_sensor_.base_pose.fill(std::numeric_limits<double>::signaling_NaN());
    input_sensor_.joint_state.position =
        Eigen::VectorXd::Map(&reference_interfaces_[0], nq);
    input_sensor_.base_twist.fill(std::numeric_limits<double>::signaling_NaN());
    new_joint_velocity_ = Eigen::VectorXd::Map(&reference_interfaces_[nq], nv);
    input_sensor_.joint_state.effort =
        Eigen::VectorXd::Map(&reference_interfaces_[nq + nv], joint_nv);
  }

  for (Eigen::Index i = 0; i < joint_nv; ++i) {
    input_sensor_.joint_state.velocity(i) = filters::exponentialSmoothing(
        new_joint_velocity_(i), input_sensor_.joint_state.velocity(i),
        parameters_.joint_velocity_filter_coefficient);
  }

  if(reference_interface_names_.size() != reference_interfaces_.size())
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
        "Inconsistent size: reference_interface_names_.size("
        << reference_interface_names_.size()
        << ") != reference_interfaces_.size("
        << reference_interfaces_.size()
        << ").");
    return false;
  }

  if(
    input_sensor_.joint_state.position.hasNaN() &&
    input_sensor_.joint_state.velocity.hasNaN() &&
    input_sensor_.joint_state.effort.hasNaN()
  ) {
    RCLCPP_ERROR(get_node()->get_logger(), "The joint state must be NaN free.");
    for(size_t i = 0 ; reference_interfaces_.size() ; ++i)
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
        "reference[" << reference_interface_names_[i]
        << "] = " << reference_interfaces_[i] << ".");
    }
    return false;
  }
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
  for (const auto& joint_name : parameters_.moving_joint_names) {
    lfc_params.p_gains.emplace_back(
        parameters_.moving_joint_names_map.at(joint_name).p);
    lfc_params.d_gains.emplace_back(
        parameters_.moving_joint_names_map.at(joint_name).d);
  }
  lfc_params.robot_has_free_flyer = parameters_.robot_has_free_flyer;
  lfc_params.pd_to_lf_transition_duration =
      Duration(parameters_.pd_to_lf_transition_duration);
  return lfc_.load(lfc_params);
}

bool LinearFeedbackControllerRos::setup_reference_interface() {
  // Setup the reference interface memory.
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
    const auto name = parameters_.chainable_controller.reference_prefix +
                      joint + "/" + HW_IF_POSITION;
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
    const auto name = parameters_.chainable_controller.reference_prefix +
                      joint + "/" + HW_IF_VELOCITY;
    reference_interface_names_.emplace_back(name);
  }
  for (const auto& joint : lfc_.get_robot_model()->get_moving_joint_names()) {
    const auto name = parameters_.chainable_controller.reference_prefix +
                      joint + "/" + HW_IF_EFFORT;
    reference_interface_names_.emplace_back(name);
  }
  reference_interfaces_.resize(
    reference_interface_names_.size(),
    std::numeric_limits<double>::quiet_NaN());
  return true;
}

bool LinearFeedbackControllerRos::allocate_memory() {
  const auto nv = lfc_.get_robot_model()->get_nv();
  const auto joint_nq = lfc_.get_robot_model()->get_joint_nq();
  const auto joint_nv = lfc_.get_robot_model()->get_joint_nv();

  output_joint_effort_ = Eigen::VectorXd::Zero(joint_nv);
  joint_effort_command_interface_.reserve(joint_nv);
  joint_effort_command_interface_.clear();

  input_sensor_.joint_state.name =
      lfc_.get_robot_model()->get_moving_joint_names();
  input_sensor_.joint_state.position = Eigen::VectorXd::Zero(joint_nq);
  input_sensor_.joint_state.velocity = Eigen::VectorXd::Zero(joint_nv);
  input_sensor_.joint_state.effort = Eigen::VectorXd::Zero(joint_nv);

  input_sensor_.joint_state.position.fill(
      std::numeric_limits<double>::signaling_NaN());
  input_sensor_.joint_state.effort.fill(
      std::numeric_limits<double>::signaling_NaN());
  input_sensor_.base_pose.fill(std::numeric_limits<double>::signaling_NaN());
  input_sensor_.base_twist.fill(std::numeric_limits<double>::signaling_NaN());

  input_sensor_msg_.joint_state.position.resize(joint_nq, 0.0);
  input_sensor_msg_.joint_state.velocity.resize(joint_nv, 0.0);
  input_sensor_msg_.joint_state.effort.resize(joint_nv, 0.0);

  input_control_.initial_state = input_sensor_;
  input_control_.feedback_gain = Eigen::MatrixXd::Zero(nv, 2 * nv);
  input_control_.feedback_gain.fill(
      std::numeric_limits<double>::signaling_NaN());
  input_control_.feedforward = Eigen::VectorXd::Zero(nv);
  input_control_.feedforward.fill(std::numeric_limits<double>::signaling_NaN());

  new_joint_velocity_ = Eigen::VectorXd::Zero(joint_nv);
  new_joint_velocity_.fill(std::numeric_limits<double>::signaling_NaN());

  // Allocate subscribers
  rclcpp::QoS qos = rclcpp::QoS(10);
  qos.best_effort();
  using namespace std::placeholders;

  sensor_publisher_ = get_node()->create_publisher<SensorMsg>("sensor", qos);
  control_subscriber_ = get_node()->create_subscription<ControlMsg>(
      "control", qos,
      std::bind(&LinearFeedbackControllerRos::control_subscription_callback,
                this, _1));
  return true;
}

bool LinearFeedbackControllerRos::ends_with(const std::string& str,
                                            const std::string& suffix) const {
  return str.size() >= suffix.size() &&
         str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void LinearFeedbackControllerRos::control_subscription_callback(
    const ControlMsg msg) {
  RCLCPP_INFO_ONCE(get_node()->get_logger(), "Received sensor msgs.");
  synched_input_control_msg_.mutex.lock();
  synched_input_control_msg_.msg = msg;
  synched_input_control_msg_.mutex.unlock();
}

}  // namespace linear_feedback_controller

PLUGINLIB_EXPORT_CLASS(linear_feedback_controller::LinearFeedbackControllerRos,
                       controller_interface::ChainableControllerInterface);
