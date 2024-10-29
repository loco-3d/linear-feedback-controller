#include "linear_feedback_controller/controller.hpp"

namespace linear_feedback_controller {

Controller::Controller() {
  robot_model_builder_ = std::make_shared<RobotModelBuilder>();
  control_.resize(0);
  first_control_received_time_ = TimePoint::min();
}

Controller::~Controller() {}

bool Controller::load(const ControllerParameters& params) {
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

  // Setup the lfc controller.
  lf_controller_.initialize(robot_model_builder_);

  return true;
}

bool Controller::configure(const Eigen::VectorXd& tau_init,
                           const Eigen::VectorXd& jq_init) {
  pd_controller_.set_reference(tau_init, jq_init);
  return true;
}

const Eigen::VectorXd& Controller::compute_control(TimePoint time,
                                                   Sensor sensor,
                                                   Control control) {
  // Shortcuts for easier code writing.
  const auto& sensor_js = sensor.joint_state;
  const auto& ctrl_js = control.initial_state.joint_state;

  // Self documented variables.
  const bool control_msg_received = !ctrl_js.name.empty();
  const bool first_control_received_time_initialized =
      first_control_received_time_ == TimePoint::min();
  const bool during_switch =
      (time - first_control_received_time_) < params_.from_pd_to_lf_duration_;

  // Check whenever the first data has arrived and save the time.
  if (control_msg_received && !first_control_received_time_initialized) {
    first_control_received_time_ = time;
  }

  if (!first_control_received_time_initialized) {
    control_ =
        pd_controller_.compute_control(sensor_js.position, sensor_js.velocity);
  } else if (during_switch) {
    double weight = ((time - first_control_received_time_).count()) /
                    params_.from_pd_to_lf_duration_.count();
    weight = std::clamp(weight, 0.0, 1.0);
    const Eigen::VectorXd& pd_ctrl =
        pd_controller_.compute_control(sensor_js.position, sensor_js.velocity);
    const Eigen::VectorXd& lf_ctrl =
        lf_controller_.compute_control(sensor, control);

    control_ = (1.0 - weight) * pd_ctrl + weight * lf_ctrl;
  } else {
    control_ = lf_controller_.compute_control(sensor, control);
  }

  return control_;
}

}  // namespace linear_feedback_controller
