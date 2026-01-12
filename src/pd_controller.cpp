#include "linear_feedback_controller/pd_controller.hpp"

namespace linear_feedback_controller {

PDController::PDController() {}

PDController::~PDController() {}

void PDController::set_gains(const Eigen::VectorXd& p_gains,
                             const Eigen::VectorXd& d_gains) {
  if (p_gains.size() != d_gains.size()) {
    throw std::invalid_argument(
        "Size missmatch between 'p_gains' and 'd_gains'.");
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
  if (p_gains.size() != d_gains.size()) {
    throw std::invalid_argument(
        "Size missmatch between 'p_gains' and 'd_gains'.");
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
  // DEBUG
  std::cout << "[PD] set_reference: tau_ref size=" << tau_ref.size()
            << ", q_ref size=" << q_ref.size() << std::endl;

  if (tau_ref.size() != q_ref.size()) {
    throw std::invalid_argument(
        "Size missmatch between 'tau_ref' and 'q_ref'.");
  }
  if (!tau_ref.array().isFinite().all() || !q_ref.array().isFinite().all()) {
    throw std::invalid_argument(
        "References should only contain valid data (no NaN or INF).");
  }

  tau_ref_ = tau_ref;
  q_ref_ = q_ref;
  control_ = Eigen::VectorXd::Zero(tau_ref.size());
}

const Eigen::VectorXd& PDController::compute_control(const Eigen::VectorXd& q,
                                                     const Eigen::VectorXd& v) {
  // // DEBUG
  // std::cout << "[PD] compute_control: q size=" << q.size()
  //           << ", v size=" << v.size()
  //           << ", p_gains size=" << p_gains_.size()
  //           << ", q_ref size=" << q_ref_.size()
  //           << ", tau_ref size=" << tau_ref_.size() << std::endl;
            
  if (q.size() != p_gains_.size() || v.size() != p_gains_.size() ||
      q_ref_.size() != p_gains_.size()) {
    throw std::invalid_argument(
        "Incoherent vector sizes between inputs (q,v) and internal controller "
        "state (gains, reference). Check configuration.");
  }

  // tau = tau_ref - Kp * (q - q_ref) - Kd * v
  control_ = tau_ref_.array() - p_gains_.array() * (q - q_ref_).array() -
             d_gains_.array() * v.array();



  // DEBUG : log détaillé au premier appel <<<
  static bool first_log = true;
  if (first_log) {
    std::cout << "[PD] compute_control: FIRST CALL" << std::endl;
    std::cout << "  q.size = " << q.size()
              << ", v.size = " << v.size()
              << ", p_gains.size = " << p_gains_.size()
              << ", q_ref.size = " << q_ref_.size()
              << ", tau_ref.size = " << tau_ref_.size() << std::endl;

    std::cout << "[PD] vectors:" << std::endl;
    std::cout << "  q       = " << q.transpose() << std::endl;
    std::cout << "  v       = " << v.transpose() << std::endl;
    std::cout << "  q_ref_  = " << q_ref_.transpose() << std::endl;
    std::cout << "  tau_ref_= " << tau_ref_.transpose() << std::endl;
    std::cout << "  Kp      = " << p_gains_.transpose() << std::endl;
    std::cout << "  Kd      = " << d_gains_.transpose() << std::endl;
    std::cout << "  tau     = " << control_.transpose() << std::endl;

    for (int i = 0; i < q.size(); ++i) {
      std::cout << "    i=" << i
                << "  q=" << q[i]
                << "  q_ref=" << q_ref_[i]
                << "  err=" << (q_ref_[i] - q[i])
                << "  v=" << v[i]
                << "  tau_ref=" << tau_ref_[i]
                << "  Kp=" << p_gains_[i]
                << "  Kd=" << d_gains_[i]
                << "  tau=" << control_[i]
                << std::endl;
    }
    first_log = false;
  }

  
  return control_;
}

}  // namespace linear_feedback_controller
