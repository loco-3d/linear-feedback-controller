#include "linear_feedback_controller/controller.hpp"

namespace linear_feedback_controller {

Controller::Controller() {
  robot_model_builder_ = std::make_shared<RobotModelBuilder>();
  control_.resize(0);
}

Controller::~Controller() {}

bool Controller::initialize(const ControllerParameters& params,
                            const Eigen::VectorXd& tau_init,
                            const Eigen::VectorXd& jq_init) {
  params_ = params;

  // Load the robot model.
  robot_model_builder_->build_model(
      params.urdf_, params.srdf_, params.moving_joint_names_,
      params.controlled_joint_names_, params.default_configuration_name_,
      params.robot_has_free_flyer_);

  // Construct the contact estimator if the robot has a free-flyer.
  if (params.robot_has_free_flyer_) {
    contact_detectors_.resize(params.contact_detector_params_.size());
    for (std::size_t i = 0; i < contact_detectors_.size(); ++i) {
      contact_detectors_[i].setParameters(params.contact_detector_params_[i]);
    }
  }

  // Min jerk to smooth the control when we switch from pd to lfc.
  min_jerk_.setParameters(params_.from_pd_to_lf_duration_.count(), 1.0);

  // Setup the pd controller.
  pd_controller_.set_gains(params.p_gains_, params.d_gains_);
  pd_controller_.set_reference(tau_init, jq_init);

  // Setup the lfc controller.
  lf_controller_.initialize(robot_model_builder_);

  return true;
}

const Eigen::VectorXd& Controller::compute_control() { return control_; }

}  // namespace linear_feedback_controller
