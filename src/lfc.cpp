#include "linear_feedback_controller/lfc.hpp"

namespace linear_feedback_controller {

LFC::LFC() {}

LFC::~LFC() {}

void initialize(const RobotModelBuilder::SharedPtr& rmb) {
  rmb_ = rmb;

  desired_configuration_ = Eigen::VectorXd::Zero(rmb_->getPinocchioModel().nq)
      desired_velocity_ = Eigen::VectorXd::Zero(rmb_->getPinocchioModel().nv)
          measured_configuration_ = Eigen::VectorXd::Zero(
              rmb_->getPinocchioModel().nq) measured_velocity_ =
              Eigen::VectorXd::Zero(rmb_->getPinocchioModel()
                                        .nv) if (rmb_->getRobotHasFreeFlyer()) {
    control_ = Eigen::VectorXd::Zero(rmb_->getPinocchioModel().nv - 6)
  }
  else {
    control_ = Eigen::VectorXd::Zero(rmb_->getPinocchioModel().nv)
  }
}

const Eigen::VectorXd& LFC::compute_control(
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg, ) {
  // Shortcuts for easier code writing.
  const linear_feedback_controller_msgs::Eigen::JointState& sensor_js =
      sensor_msg.joint_state;
  const linear_feedback_controller_msgs::Eigen::JointState& ctrl_js =
      sensor_msg.initial_state.joint_state;
  const linear_feedback_controller_msgs::Eigen::Sensor& ctrl_init =
      sensor_msg.initial_state;

  // Reconstruct the state vector: x = [q, v]
  if (rmb_->getRobotHasFreeFlyer()) {
    desired_configuration_.head<7>() = ctrl_init.base_pose;
    desired_velocity_.head<6>() = ctrl_init.base_twist;
    measured_configuration_.head<7>() = eigen_sensor_msg_.base_pose;
    measured_velocity_.head<6>() = eigen_sensor_msg_.base_twist;
  }
  int nb_dof_q = ctrl_js.position.size();
  int nb_dof_v = ctrl_js.velocity.size();
  desired_configuration_.tail(nb_dof_q) = ctrl_js.position;
  desired_velocity_.tail(nb_dof_v) = ctrl_js.velocity;
  measured_configuration_.tail(nb_dof_q) = sensor_js.position;
  measured_velocity_.tail(nb_dof_v) = sensor_js.velocity;

  // Compute the linear feedback controller desired torque.
  pinocchio::difference(rmb_->getPinocchioModel(), measured_configuration_,
                        desired_configuration_,
                        diff_state_.head(rmb_->getPinocchioModel().nv));
  diff_state_.tail(rmb_->getPinocchioModel().nv) =
      desired_velocity_ - measured_velocity_;
  lf_desired_torque_ = eigen_control_msg_.feedforward +
                       eigen_control_msg_.feedback_gain * diff_state_;

  // Define the support foot
  /// @todo automatize this in the estimator?
  desired_swing_ids_.clear();
  desired_stance_ids_.clear();
  if (ctrl_init.contacts[0].active) {
    desired_stance_ids_.push_back("left_sole_link");
  } else {
    desired_swing_ids_.push_back("left_sole_link");
  }
  if (ctrl_init.contacts[1].active) {
    desired_stance_ids_.push_back("right_sole_link");
  } else {
    desired_swing_ids_.push_back("right_sole_link");
  }
}

}  // namespace linear_feedback_controller
