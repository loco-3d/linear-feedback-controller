#ifndef LINEAR_FEEDBACK_CONTROLLER_PD_CONTROLLER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_PD_CONTROLLER_HPP

#include "Eigen/Core"
#include "linear_feedback_controller/robot_model_builder.hpp"
#include "linear_feedback_controller/visibility.hpp"

namespace linear_feedback_controller {

class LINEAR_FEEDBACK_CONTROLLER_PUBLIC PDController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PDController();
  ~PDController();

  const Eigen::VectorXd& compute_control(const Eigen::VectorXd& q,
                                         const Eigen::VectorXd& v);

  void set_gains(const Eigen::VectorXd& p_gains,
                 const Eigen::VectorXd& d_gains);

  void set_gains(const std::vector<double>& p_gains,
                 const std::vector<double>& d_gains);

  void set_reference(const Eigen::VectorXd& tau_ref,
                     const Eigen::VectorXd& q_ref);

 private:
  Eigen::VectorXd tau_ref_;
  Eigen::VectorXd q_ref_;
  Eigen::VectorXd p_gains_;
  Eigen::VectorXd d_gains_;
  Eigen::VectorXd control_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_PD_CONTROLLER_HPP
