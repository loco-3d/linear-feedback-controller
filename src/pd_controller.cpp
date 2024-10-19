#include "linear_feedback_controller/pd_controller.hpp"

namespace linear_feedback_controller {

PDController::PDController() {}

PDController::~PDController() {}

PDController::compute_control(Eigen::VectorXd tau_ref, Eigen::VectorXd q_ref,
                              Eigen::VectorXd q, Eigen::VectorXd v,
                              Eigen::VectorXd p_gains,
                              Eigen::VectorXd d_gains) {
  control_ = tau.array() - p_gain.array() * (q - initial_position_[i]).array() -
             d_gain.array() * v.array();
  return control_;
}

}  // namespace linear_feedback_controller
