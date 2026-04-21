#include "linear_feedback_controller/pd_controller.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

namespace linear_feedback_controller {

PDController::PDController() {}

PDController::~PDController() {}

void PDController::initialize(const RobotModelBuilder::SharedPtr& rmb) {
  if (!rmb) {
    throw std::invalid_argument("RobotModelBuilder pointer cannot be null.");
  }
  rmb_ = rmb;
}

void PDController::set_gains(const Eigen::VectorXd& p_gains,
                             const Eigen::VectorXd& d_gains) {
  if (!rmb_) {
    throw std::runtime_error(
        "PDController is not initialized. Call initialize() before "
        "set_gains().");
  }
  if (p_gains.size() != static_cast<Eigen::Index>(rmb_->get_joint_nv())) {
    throw std::invalid_argument("p_gains.size() != joint_nv.");
  }
  if (d_gains.size() != static_cast<Eigen::Index>(rmb_->get_joint_nv())) {
    throw std::invalid_argument("d_gains.size() != joint_nv.");
  }
  if (!p_gains.array().isFinite().all() || !d_gains.array().isFinite().all()) {
    throw std::invalid_argument(
        "'p_gains' and 'd_gains' should only contain valid data (no NaN or "
        "INF).");
  }

  p_gains_ = p_gains;
  d_gains_ = d_gains;
}

void PDController::set_gains(const std::vector<double>& p_gains,
                             const std::vector<double>& d_gains) {
  if (!rmb_) {
    throw std::runtime_error(
        "PDController is not initialized. Call initialize() before "
        "set_gains().");
  }
  if (p_gains.size() != static_cast<Eigen::Index>(rmb_->get_joint_nv())) {
    throw std::invalid_argument("p_gains.size() != joint_nv.");
  }
  if (d_gains.size() != static_cast<Eigen::Index>(rmb_->get_joint_nv())) {
    throw std::invalid_argument("d_gains.size() != joint_nv.");
  }
  for (const auto& val : p_gains) {
    if (!std::isfinite(val)) {
      throw std::invalid_argument(
          "'p_gains' should only contain valid data (no NaN or INF).");
    }
  }
  for (const auto& val : d_gains) {
    if (!std::isfinite(val)) {
      throw std::invalid_argument(
          "'d_gains' should only contain valid data (no NaN or INF).");
    }
  }

  p_gains_ = Eigen::VectorXd::Map(p_gains.data(), p_gains.size());
  d_gains_ = Eigen::VectorXd::Map(d_gains.data(), d_gains.size());
}

void PDController::set_reference(const Eigen::VectorXd& tau_ref,
                                 const Eigen::VectorXd& q_ref) {
  // We should not compare anymore tau_ref.size() and q_ref.size() — they live
  // in different spaces (nv vs nq). The consistency is now checked as following
  // to handle case where we have a freeflyer (nq > nv):
  if (!rmb_) {
    throw std::runtime_error(
        "PDController is not initialized. Call initialize() before "
        "set_reference().");
  }
  if (tau_ref.size() != static_cast<Eigen::Index>(rmb_->get_joint_nv())) {
    throw std::invalid_argument("tau_ref.size() != joint_nv");
  }
  // q_ref is in joint space (joint_hw_nq), not full pinocchio nq
  if (q_ref.size() != static_cast<Eigen::Index>(rmb_->get_joint_hw_nq())) {
    throw std::invalid_argument("q_ref.size() != joint_hw_nq");
  }
  if (!tau_ref.array().isFinite().all() || !q_ref.array().isFinite().all()) {
    throw std::invalid_argument("References should only contain valid data.");
  }
  tau_ref_ = tau_ref;
  q_ref_ = q_ref;
  control_ = Eigen::VectorXd::Zero(tau_ref.size());
}

const Eigen::VectorXd& PDController::compute_control(const Eigen::VectorXd& q,
                                                     const Eigen::VectorXd& v) {
  // Check if the controller is properly initialized and the reference and gains
  // are set
  if (!rmb_) {
    throw std::runtime_error(
        "PDController is not initialized. Call initialize() before "
        "compute_control().");
  }
  if (q_ref_.size() == 0 || tau_ref_.size() == 0) {
    throw std::runtime_error(
        "Reference is not set. Call set_reference() before compute_control().");
  }
  if (p_gains_.size() == 0 || d_gains_.size() == 0) {
    throw std::runtime_error(
        "Gains are not set. Call set_gains() before compute_control().");
  }
  const auto joint_nq = rmb_->get_joint_hw_nq();
  const auto joint_nv = rmb_->get_joint_nv();
  // Check the size of q and v against the model dimensions
  if (q.size() != static_cast<Eigen::Index>(joint_nq)) {
    throw std::invalid_argument(
        "q.size() != model.nq. Make sure q is a full configuration vector.");
  }
  if (v.size() != static_cast<Eigen::Index>(joint_nv)) {
    throw std::invalid_argument(
        "v.size() != model.nv. Make sure v has the right size (model.nv)");
  }
  // Embed joint-space q into full pinocchio configuration for difference()
  const auto full_nq = rmb_->get_nq();
  Eigen::VectorXd q_full = pinocchio::neutral(rmb_->get_model());
  q_full.tail(joint_nq) = q;
  Eigen::VectorXd q_ref_full = pinocchio::neutral(rmb_->get_model());
  q_ref_full.tail(joint_nq) = q_ref_;

  // pinocchio::difference returns full nv; take only the actuated tail
  const Eigen::VectorXd error_q =
      pinocchio::difference(rmb_->get_model(), q_ref_full, q_full)
          .tail(joint_nv);

  control_ = tau_ref_.array() - p_gains_.array() * error_q.array() -
             d_gains_.array() * v.array();

  return control_;
}

}  // namespace linear_feedback_controller
