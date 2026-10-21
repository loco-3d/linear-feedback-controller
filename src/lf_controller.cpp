#include "linear_feedback_controller/lf_controller.hpp"

#include <cmath>

#include "pinocchio/algorithm/joint-configuration.hpp"

namespace linear_feedback_controller {

LFController::LFController() {}

LFController::~LFController() {}

void LFController::initialize(const RobotModelBuilder::SharedPtr& rmb) {
  if (!rmb) {
    throw std::invalid_argument("RobotModelBuilder pointer cannot be null.");
  }
  rmb_ = rmb;
  const auto nq = rmb_->get_nq();
  const auto nv = rmb_->get_nv();
  const auto joint_nv = rmb_->get_joint_nv();

  desired_configuration_ = Eigen::VectorXd::Zero(nq);
  desired_velocity_ = Eigen::VectorXd::Zero(nv);
  measured_configuration_ = Eigen::VectorXd::Zero(nq);
  measured_velocity_ = Eigen::VectorXd::Zero(nv);
  control_ = Eigen::VectorXd::Zero(joint_nv);
  diff_state_ = Eigen::VectorXd::Zero(2 * nv);

  u_fb_ = Eigen::VectorXd::Zero(joint_nv);
  u_fb_raw_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_out_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_x1_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_x2_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_y1_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_y2_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_primed_ = false;
}

void LFController::configure_feedback_lowpass(double cutoff_hz,
                                              double sample_rate_hz) {
  if (cutoff_hz <= 0.0 || sample_rate_hz <= 0.0 ||
      cutoff_hz >= 0.5 * sample_rate_hz) {
    fb_lp_enabled_ = false;
    return;
  }
  // 2nd-order Butterworth low-pass, bilinear transform with pre-warping.
  constexpr double kPi = 3.14159265358979323846;
  const double q = 1.0 / std::sqrt(2.0);
  const double k = std::tan(kPi * cutoff_hz / sample_rate_hz);
  const double norm = 1.0 / (1.0 + k / q + k * k);
  fb_lp_b0_ = k * k * norm;
  fb_lp_b1_ = 2.0 * fb_lp_b0_;
  fb_lp_b2_ = fb_lp_b0_;
  fb_lp_a1_ = 2.0 * (k * k - 1.0) * norm;
  fb_lp_a2_ = (1.0 - k / q + k * k) * norm;
  fb_lp_enabled_ = true;
  fb_lp_primed_ = false;
}

const Eigen::VectorXd& LFController::compute_control(
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
    const linear_feedback_controller_msgs::Eigen::Control& control_msg) {
  if (!rmb_) {
    throw std::runtime_error(
        "LFController is not initialized. Call initialize() before "
        "compute_control().");
  }
  const linear_feedback_controller_msgs::Eigen::Sensor& ctrl_init =
      control_msg.initial_state;

  // Reconstruct the state vector: x = [q, v]. The desired state is the OCP knot
  // (possibly interpolated across the cycle by the ROS layer for delay comp).
  rmb_->construct_robot_state(ctrl_init, desired_configuration_,
                              desired_velocity_);
  rmb_->construct_robot_state(sensor_msg, measured_configuration_,
                              measured_velocity_);

  // diff_state = [ q_des (-) q_meas ; v_des - v_meas ]
  pinocchio::difference(rmb_->get_model(), measured_configuration_,
                        desired_configuration_,
                        diff_state_.head(rmb_->get_model().nv));
  diff_state_.tail(rmb_->get_model().nv) =
      desired_velocity_ - measured_velocity_;

  // Feedback torque, optionally low-pass filtered (see
  // configure_feedback_lowpass).
  u_fb_.noalias() = control_msg.feedback_gain * diff_state_;
  u_fb_raw_ = u_fb_;
  if (fb_lp_enabled_) {
    if (!fb_lp_primed_) {
      fb_lp_x1_ = fb_lp_x2_ = fb_lp_y1_ = fb_lp_y2_ = u_fb_;
      fb_lp_primed_ = true;
    }
    // y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] - a1 y[n-1] - a2 y[n-2]
    fb_lp_out_.noalias() = fb_lp_b0_ * u_fb_ + fb_lp_b1_ * fb_lp_x1_ +
                           fb_lp_b2_ * fb_lp_x2_ - fb_lp_a1_ * fb_lp_y1_ -
                           fb_lp_a2_ * fb_lp_y2_;
    fb_lp_x2_ = fb_lp_x1_;
    fb_lp_x1_ = u_fb_;
    fb_lp_y2_ = fb_lp_y1_;
    fb_lp_y1_ = fb_lp_out_;
    u_fb_ = fb_lp_out_;
  }

  // Optional attenuation of the fast-loop gain (see set_feedback_gain_scale).
  // u_fb_raw_ above stays the unscaled K*(x - x_ref).
  if (fb_scale_ != 1.0) u_fb_ *= fb_scale_;

  control_.noalias() = control_msg.feedforward + u_fb_;
  return control_;
}

}  // namespace linear_feedback_controller
