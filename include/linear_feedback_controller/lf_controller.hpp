#ifndef LINEAR_FEEDBACK_CONTROLLER_LFCONTROLLER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_LFCONTROLLER_HPP

#include "Eigen/Core"
#include "linear_feedback_controller/robot_model_builder.hpp"
#include "linear_feedback_controller/visibility.hpp"
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"

namespace linear_feedback_controller {

class LINEAR_FEEDBACK_CONTROLLER_PUBLIC LFController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static constexpr int kNbFreeFlyerDof = 6;

  LFController();
  virtual ~LFController();

  void initialize(const RobotModelBuilder::SharedPtr& rmb);

  const Eigen::VectorXd& compute_control(
      const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
      const linear_feedback_controller_msgs::Eigen::Control& control_msg);

 private:
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

#endif  // LINEAR_FEEDBACK_CONTROLLER_LFCONTROLLER_HPP
