#ifndef LINEAR_FEEDBACK_CONTROLLER_pd_controller_HPP
#define LINEAR_FEEDBACK_CONTROLLER_pd_controller_HPP

#include "Eigen/Core"
#include "linear_feedback_controller/robot_model_builder.hpp"

namespace linear_feedback_controller {

class PDController {
 public:
  PDController();
  ~PDController();

  const Eigen::VectorXd& compute_control(const Eigen::VectorXd& q,
                                         const Eigen::VectorXd& v);

  void setGains(const Eigen::VectorXd& p_gains, const Eigen::VectorXd& d_gains);

  void setReference(const Eigen::VectorXd& tau_ref,
                    const Eigen::VectorXd& q_ref);

 private:
  Eigen::VectorXd tau_ref_;
  Eigen::VectorXd q_ref_;
  Eigen::VectorXd p_gains_;
  Eigen::VectorXd d_gains_;
  Eigen::VectorXd control_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_pd_controller_HPP
