// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#ifndef LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_ROS_HPP
#define LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_ROS_HPP

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"

// ROS 2 control
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Class to wrap in ROS
#include "linear_feedback_controller/linear_feedback_controller.hpp"

// PAL public utilities for logging data
#include "pal_statistics/pal_statistics.hpp"

// Auto-generated header by the `generate_parameter_library` package.
#include "linear_feedback_controller/generated_parameters.hpp"

namespace linear_feedback_controller {

using controller_interface::CallbackReturn;
using controller_interface::ChainableControllerInterface;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

/**
 * @brief This chainable controller provides a base state estimator.
 *        It computes the state of the base and joint of the robot accroding to
 *        measurement of the IMU and joint state.
 */
class LinearFeedbackControllerRos : public ChainableControllerInterface {
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
  virtual InterfaceConfiguration command_interface_configuration() const final;

  /// @brief @copydoc ControllerInterfaceBase::state_interface_configuration
  /// This function access none of the states.
  virtual InterfaceConfiguration state_interface_configuration() const final;

  /// @brief @copydoc
  /// ChainableControllerInterface::on_export_reference_interfaces This function
  /// access the reference state from an estimator.
  virtual std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces() final;

  /// @brief ChainableControllerInterface::update_reference_from_subscribers
  virtual return_type update_reference_from_subscribers() final;

  /// @brief ChainableControllerInterface::update_and_write_commands
  virtual return_type update_and_write_commands(
      const rclcpp::Time& time, const rclcpp::Duration& period) final;

  /// @brief @copydoc rclcpp_lifecycle::on_configure
  virtual CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) final;

  /// @brief @copydoc ControllerInterfaceBase::on_init
  /// can return SUCCESS, FAILURE, or ERROR
  virtual CallbackReturn on_init() final;

  /// @brief @copydoc rclcpp_lifecycle::on_activate
  ///
  /// Here we assume that the robot is not moving and has it's feet on the
  /// ground.
  virtual CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) final;

  /// @brief @copydoc rclcpp_lifecycle::on_deactivate
  virtual CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) final;

  /// @brief @copydoc rclcpp_lifecycle::on_cleanup
  virtual controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

  /// @brief @copydoc rclcpp_lifecycle::on_error
  virtual controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

 protected:
  // Initialization methods.
  bool load_parameters();
  bool wait_for_robot_description(std::string& robot_description);
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
  void register_var(const std::string& id, const Eigen::Quaterniond& quat);
  void register_var(const std::string& id, const std::vector<double>& vec);

  // Utils.
  bool ends_with(const std::string& str, const std::string& suffix) const;

 protected:
  // Functional Attributes.
  std::shared_ptr<linear_feedback_controller::ParamListener>
      parameter_listener_;
  linear_feedback_controller::Params parameters_;

  // State interfaces
  InterfaceVector<hardware_interface::LoanedStateInterface>
      joint_position_state_interface_;
  InterfaceVector<hardware_interface::LoanedStateInterface>
      joint_velocity_state_interface_;
  InterfaceVector<hardware_interface::LoanedStateInterface>
      joint_effort_state_interface_;

  // Reference interfaces.
  std::vector<std::string> reference_interface_names_;
  std::string reference_prefix_;

  // Command interfaces.
  InterfaceVector<hardware_interface::LoanedCommandInterface>
      joint_effort_command_interface_;
  std::string command_prefix_;

  /// @brief Controller without ROS.
  LinearFeedbackController lfc_;

  // Inputs/Ouputs attributes.
  /// @brief Joint position measured at init time.
  Eigen::VectorXd init_joint_position_;
  /// @brief Joint torques measured at init time.
  Eigen::VectorXd init_joint_effort_;
  /// @brief Genrealized coordinates.
  Eigen::VectorXd input_robot_configuration_;
  /// @brief Genrealized coordinates.
  Eigen::VectorXd input_robot_velocity_;
  Eigen::VectorXd output_joint_effort_;

  // Logging attributes.
  pal_statistics::RegistrationsRAII bookkeeping_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_ROS_HPP
