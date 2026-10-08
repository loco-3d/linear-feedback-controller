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

  /// @brief Enable a per-joint 2nd-order Butterworth low-pass on the feedback
  /// torque u_fb = K*(x - x_ref). @param cutoff_hz 0 disables it (default).
  /// @param sample_rate_hz the rate compute_control() is called at.
  void configure_feedback_lowpass(double cutoff_hz, double sample_rate_hz);

  /// @brief Scale the feedback torque before it is added to the feedforward.
  /// @param scale 1.0 = nominal, 0.0 = feedforward only, 0<s<1 lowers the
  /// fast-loop gain. Applied after the optional low-pass.
  void set_feedback_gain_scale(double scale) { fb_scale_ = scale; }

  const Eigen::VectorXd& compute_control(
      const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
      const linear_feedback_controller_msgs::Eigen::Control& control_msg);

  /// @name Introspection of the last compute_control() call (for /lfc_debug).
  /// @{
  /// @brief Feedback torque K*(x - x_ref) BEFORE the optional low-pass.
  const Eigen::VectorXd& get_feedback_torque_raw() const { return u_fb_raw_; }
  /// @brief Feedback torque actually added to the feedforward (post low-pass).
  const Eigen::VectorXd& get_feedback_torque() const { return u_fb_; }
  /// @brief The reference configuration used (= control.initial_state q,
  /// already interpolated across the MPC cycle by the ROS layer when enabled).
  const Eigen::VectorXd& get_desired_configuration() const {
    return desired_configuration_;
  }
  /// @brief The reference velocity used (see get_desired_configuration).
  const Eigen::VectorXd& get_desired_velocity() const {
    return desired_velocity_;
  }
  /// @}

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

  /// @name Feedback-torque low-pass (see configure_feedback_lowpass)
  /// 2nd-order Butterworth, direct-form-II transposed, per joint.
  /// @{
  bool fb_lp_enabled_ = false;
  double fb_lp_b0_ = 1.0, fb_lp_b1_ = 0.0, fb_lp_b2_ = 0.0;
  double fb_lp_a1_ = 0.0, fb_lp_a2_ = 0.0;
  Eigen::VectorXd fb_lp_x1_, fb_lp_x2_, fb_lp_y1_, fb_lp_y2_;
  bool fb_lp_primed_ = false;  ///< seed the filter state on the first sample
  Eigen::VectorXd u_fb_;       ///< scratch: K*(x - x_ref) before filtering
  Eigen::VectorXd u_fb_raw_;   ///< copy of u_fb_ before the low-pass (debug)
  Eigen::VectorXd fb_lp_out_;  ///< scratch: filtered feedback torque
  /// @}

  /// @brief Multiplies u_fb before it is added to the feedforward (see
  /// set_feedback_gain_scale). 1.0 = nominal, 0.0 = feedforward only.
  double fb_scale_ = 1.0;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_LFCONTROLLER_HPP
