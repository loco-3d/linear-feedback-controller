#ifndef LINEAR_FEEDBACK_CONTROLLER_pd_controller_HPP
#define LINEAR_FEEDBACK_CONTROLLER_pd_controller_HPP

#include "Eigen/Core"
#include "linear-feedback-controller/robot_model_builder.hpp"

namespace linear_feedback_controller {

class PDController {
 public:
  PDController();
  ~PDController();

  const Eigen::VectorXd& compute_control(Eigen::VectorXd q, Eigen::VectorXd v);

  Eigen::VectorXd control_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_pd_controller_HPP
