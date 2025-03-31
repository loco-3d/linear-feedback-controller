#ifndef LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_ROS_HPP
#define LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_ROS_HPP

#include <chrono>
#include <memory>

// ROS 2
#include "control_toolbox/filters.hpp"
#include "linear_feedback_controller_msgs/msg/control.hpp"
#include "linear_feedback_controller_msgs/msg/sensor.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// ROS 2 control
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// PAL public utilities for logging data
#include "pal_statistics/pal_statistics.hpp"

// Class to wrap in ROS
#include "linear_feedback_controller/linear_feedback_controller.hpp"

// Auto-generated header by the `generate_parameter_library` package.
#include "linear_feedback_controller/generated_parameters.hpp"
#include "linear_feedback_controller/visibility.hpp"

namespace linear_feedback_controller {

using controller_interface::CallbackReturn;
using controller_interface::ChainableControllerInterface;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using nav_msgs::msg::Odometry;
using rclcpp_lifecycle::LifecycleNode;
using sensor_msgs::msg::JointState;
using SensorMsg = linear_feedback_controller_msgs::msg::Sensor;
using ControlMsg = linear_feedback_controller_msgs::msg::Control;

#define CONTROLLER_INTERFACE_VERSION_AT_LEAST(major, minor, patch) \
  ((CONTROLLER_INTERFACE_MAJOR_VERSION > (major)) ||               \
   (CONTROLLER_INTERFACE_MAJOR_VERSION == (major) &&               \
    CONTROLLER_INTERFACE_MINOR_VERSION > (minor)) ||               \
   (CONTROLLER_INTERFACE_MAJOR_VERSION == (major) &&               \
    CONTROLLER_INTERFACE_MINOR_VERSION == (minor) &&               \
    CONTROLLER_INTERFACE_PATCH_VERSION >= (patch)))

struct LINEAR_FEEDBACK_CONTROLLER_PRIVATE ProtectedControlMsg {
  ControlMsg msg;
  std::mutex mutex;
};

class LINEAR_FEEDBACK_CONTROLLER_PUBLIC LinearFeedbackControllerRos
    : public ChainableControllerInterface {
 public:
  /// @brief The interfaces are defined as the types in
  /// 'allowed_interface_types_'
  ///        member. For convenience, for each type the interfaces are ordered
  ///        so that i-th position matches i-th index in joint_names_
  template <typename T>
  using InterfaceVector = std::vector<std::reference_wrapper<T>>;

 public:
  // Public methods.

  /// @brief Construct a new State Estimation Controller object.
  LinearFeedbackControllerRos();

  /// @brief Destroy the State Estimation Controller object.
  virtual ~LinearFeedbackControllerRos();

  /// @brief @copydoc ControllerInterfaceBase::command_interface_configuration
  /// This function access the output torques.
  InterfaceConfiguration command_interface_configuration() const final;

  /// @brief @copydoc ControllerInterfaceBase::state_interface_configuration
  /// This function access none of the states.
  InterfaceConfiguration state_interface_configuration() const final;

  /// @brief @copydoc
  /// ChainableControllerInterface::on_export_reference_interfaces This function
  /// access the reference state from an estimator.
  std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces() final;

// master (jazzy) version 01/03/2025
#if CONTROLLER_INTERFACE_VERSION_AT_LEAST(4, 0, 0)
  /// @brief ChainableControllerInterface::update_reference_from_subscribers
  return_type update_reference_from_subscribers(
      const rclcpp::Time& time, const rclcpp::Duration& period) final;
#else  // humble version
  /// @brief ChainableControllerInterface::update_reference_from_subscribers
  return_type update_reference_from_subscribers() final;
#endif

  /// @brief ChainableControllerInterface::update_and_write_commands
  return_type update_and_write_commands(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) final;

  /// @brief @copydoc rclcpp_lifecycle::on_configure
  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) final;

  /// @brief @copydoc ControllerInterfaceBase::on_init
  /// can return SUCCESS, FAILURE, or ERROR
  CallbackReturn on_init() final;

  /// @brief @copydoc rclcpp_lifecycle::on_activate
  ///
  /// Here we assume that the robot is not moving and has it's feet on the
  /// ground.
  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) final;

  /// @brief @copydoc rclcpp_lifecycle::on_set_chained_mode
  bool on_set_chained_mode(bool chained_mode) final;

  /// @brief @copydoc rclcpp_lifecycle::on_deactivate
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) final;

  /// @brief @copydoc rclcpp_lifecycle::on_cleanup
  controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) final;

  /// @brief @copydoc rclcpp_lifecycle::on_error
  controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) final;

 protected:
  // Initialization methods.
  bool load_parameters();
  bool wait_for_robot_description();
  bool get_robot_description(std::string& robot_description);
  bool load_linear_feedback_controller(const std::string& robot_description);
  bool setup_reference_interface();
  bool allocate_memory();
  bool initialize_introspection();

  // Run methods
  bool read_state_from_references();

  // Allow an update of the parameters dynamically.
  bool update_parameters();

  // Pal Statistics.
  void register_var(const std::string& id, const Eigen::Vector3d& vec);
  void register_var(const std::string& id,
                    const Eigen::Matrix<double, 6, 1>& vec);
  void register_var(const std::string& id,
                    const Eigen::Matrix<double, 7, 1>& vec);
  void register_var(const std::string& id, const Eigen::Quaterniond& quat);
  void register_var(const std::string& id, const std::vector<double>& vec);
  void register_var(const std::string& id, const Eigen::VectorXd& vec);

  // Utils.
  bool ends_with(const std::string& str, const std::string& suffix) const;

  // Callbacks.
  void state_syncher_callback(
      const Odometry::ConstSharedPtr& msg_odom,
      const JointState::ConstSharedPtr& msg_joint_state);
  void control_subscription_callback(const ControlMsg msg);

 protected:
  // ROS params.
  std::shared_ptr<linear_feedback_controller::ParamListener>
      parameter_listener_;
  linear_feedback_controller::Params parameters_;

  // Fetch the robot_description
  static const std::string robot_description_name_;
  rclcpp::Node::SharedPtr robot_description_node_;
  rclcpp::SyncParametersClient::SharedPtr robot_description_parameter_client_;

  // Reference interfaces
  InterfaceVector<double> base_reference_interface_;
  InterfaceVector<double> base_reference_interface_;
  InterfaceVector<double> joint_position_reference_interface_;
  InterfaceVector<double> joint_velocity_reference_interface_;
  InterfaceVector<double> joint_effort_reference_interface_;

  // Reference interfaces.
  std::vector<std::string> reference_interface_names_;

  // Command interfaces.
  InterfaceVector<hardware_interface::LoanedCommandInterface>
    joint_effort_command_interface_;

  /// @brief Controller without ROS.
  LinearFeedbackController lfc_;

  /// @brief Joint position measured at init time.
  Eigen::VectorXd init_joint_position_;
  /// @brief Joint torques measured at init time.
  Eigen::VectorXd init_joint_effort_;

  // Inputs/Ouputs attributes.
  TimePoint input_time_;
  linear_feedback_controller_msgs::Eigen::Sensor input_sensor_;
  linear_feedback_controller_msgs::Eigen::Control input_control_;
  linear_feedback_controller_msgs::msg::Sensor input_sensor_msg_;
  linear_feedback_controller_msgs::msg::Control input_control_msg_;
  Eigen::VectorXd output_joint_effort_;

  // Logging attributes.
  pal_statistics::RegistrationsRAII bookkeeping_;

  // MPC communication
  rclcpp::Publisher<SensorMsg>::SharedPtr sensor_publisher_;
  rclcpp::Subscription<ControlMsg>::SharedPtr control_subscriber_;
  ProtectedControlMsg synched_input_control_msg_;

  // Used in the filtering of the joint velocity.
  Eigen::VectorXd new_joint_velocity_;

  bool first_time_update_and_write_commands_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_ROS_HPP
