#include "linear_feedback_controller/pd_controller.hpp"

namespace linear_feedback_controller {

PDController::PDController() {}

PDController::~PDController() {}

void PDController::set_gains(const Eigen::VectorXd& p_gains,
                             const Eigen::VectorXd& d_gains) {
  p_gains_ = p_gains;
  d_gains_ = d_gains;
}

void PDController::set_reference(const Eigen::VectorXd& tau_ref,
                                 const Eigen::VectorXd& q_ref) {
  tau_ref_ = tau_ref;
  q_ref_ = q_ref;
  control_ = Eigen::VectorXd::Zero(tau_ref.size());
}

const Eigen::VectorXd& PDController::compute_control(const Eigen::VectorXd& q,
                                                     const Eigen::VectorXd& v) {
  assert(q.size() == v.size());
  assert(tau_ref_.size() == v.size());

  control_ = tau_ref_.array() - p_gains_.array() * (q - q_ref_).array() -
             d_gains_.array() * v.array();
  return control_;
}

}  // namespace linear_feedback_controller
