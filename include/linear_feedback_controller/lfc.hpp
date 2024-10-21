#ifndef LINEAR_FEEDBACK_CONTROLLER_LFC_HPP
#define LINEAR_FEEDBACK_CONTROLLER_LFC_HPP

#include "Eigen/Core"
#include "linear_feedback_controller/robot_model_builder.hpp"
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"

namespace linear_feedback_controller {

class LFC {
 public:
  LFC();
  virtual ~LFC();

  void initialize(const RobotModelBuilder::SharedPtr& rmb);

  const Eigen::VectorXd& compute_control(
      const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
      const linear_feedback_controller_msgs::Eigen::Control& control_msg);

  Eigen::VectorXd desired_configuration_;
  Eigen::VectorXd desired_velocity_;
  Eigen::VectorXd measured_configuration_;
  Eigen::VectorXd measured_velocity_;

  /**
   *  @brief Difference between the measured and the desired state,
   *  \$f x = [ q^T, \dot{q}^T ]^T \$f
   *
   */
  Eigen::VectorXd diff_state_;

  Eigen::VectorXd control_;
  RobotModelBuilder::SharedPtr rmb_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_LFC_HPP
