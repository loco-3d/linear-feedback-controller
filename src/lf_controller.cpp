#include "linear_feedback_controller/lf_controller.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

namespace linear_feedback_controller {

static constexpr int LFController::kNbFreeFlyerDof = 6;

LFController::LFController() {}

LFController::~LFController() {}

void LFController::initialize(const RobotModelBuilder::SharedPtr& rmb) {
  rmb_ = rmb;

  desired_configuration_ = Eigen::VectorXd::Zero(rmb_->get_model().nq);
  desired_velocity_ = Eigen::VectorXd::Zero(rmb_->get_model().nv);
  measured_configuration_ = Eigen::VectorXd::Zero(rmb_->get_model().nq);
  measured_velocity_ = Eigen::VectorXd::Zero(rmb_->get_model().nv);
  if (rmb_->get_robot_has_free_flyer()) {
    control_ = Eigen::VectorXd::Zero(rmb_->get_model().nv - kNbFreeFlyerDof);
  } else {
    control_ = Eigen::VectorXd::Zero(rmb_->get_model().nv);
  }
}

const Eigen::VectorXd& LFController::compute_control(
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
    const linear_feedback_controller_msgs::Eigen::Control& control_msg) {
  // Shortcuts for easier code writing.
  const linear_feedback_controller_msgs::Eigen::JointState& sensor_js =
      sensor_msg.joint_state;
  const linear_feedback_controller_msgs::Eigen::JointState& ctrl_js =
      control_msg.initial_state.joint_state;
  const linear_feedback_controller_msgs::Eigen::Sensor& ctrl_init =
      control_msg.initial_state;

  // Reconstruct the state vector: x = [q, v]
  if (rmb_->get_robot_has_free_flyer()) {
    desired_configuration_.head<7>() = ctrl_init.base_pose;
    desired_velocity_.head<6>() = ctrl_init.base_twist;
    measured_configuration_.head<7>() = sensor_msg.base_pose;
    measured_velocity_.head<6>() = sensor_msg.base_twist;
  }
  const int nb_dof_q = ctrl_js.position.size();
  const int nb_dof_v = ctrl_js.velocity.size();
  desired_configuration_.tail(nb_dof_q) = ctrl_js.position;
  desired_velocity_.tail(nb_dof_v) = ctrl_js.velocity;
  measured_configuration_.tail(nb_dof_q) = sensor_js.position;
  measured_velocity_.tail(nb_dof_v) = sensor_js.velocity;

  // Compute the linear feedback controller desired torque.
  pinocchio::difference(rmb_->get_model(), measured_configuration_,
                        desired_configuration_,
                        diff_state_.head(rmb_->get_model().nv));
  diff_state_.tail(rmb_->get_model().nv) =
      desired_velocity_ - measured_velocity_;
  control_.noalias() =
      control_msg.feedforward + control_msg.feedback_gain * diff_state_;

  return control_;
}

}  // namespace linear_feedback_controller
