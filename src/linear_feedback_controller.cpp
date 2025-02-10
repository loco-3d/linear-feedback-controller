#include "linear_feedback_controller/linear_feedback_controller.hpp"

namespace linear_feedback_controller {

LinearFeedbackController::LinearFeedbackController() {
  robot_model_builder_ = std::make_shared<RobotModelBuilder>();
  control_.resize(0);
  first_control_received_time_ = TimePoint::min();
}

LinearFeedbackController::~LinearFeedbackController() {}

bool LinearFeedbackController::load(const ControllerParameters& params) {
  params_ = params;

  // Load the robot model.
  if (!robot_model_builder_->build_model(
          params_.urdf, params_.moving_joint_names,
          params_.controlled_joint_names, params_.robot_has_free_flyer)) {
    return false;
  }

  // Min jerk to smooth the control when we switch from pd to lfc.
  min_jerk_.set_parameters(params_.pd_to_lf_transition_duration.count(), 1.0);

  // Setup the pd controller.
  pd_controller_.set_gains(params_.p_gains, params_.d_gains);

  // Setup the lfc controller.
  lf_controller_.initialize(robot_model_builder_);

  // Allocate memory
  robot_configuration_ = Eigen::VectorXd::Zero(robot_model_builder_->get_nq());
  robot_velocity_ = Eigen::VectorXd::Zero(robot_model_builder_->get_nv());
  robot_velocity_null_ = Eigen::VectorXd::Zero(robot_model_builder_->get_nv());
  return true;
}

bool LinearFeedbackController::set_initial_state(
    const Eigen::VectorXd& tau_init, const Eigen::VectorXd& jq_init) {
  pd_controller_.set_reference(tau_init, jq_init);
  return true;
}

const Eigen::VectorXd& LinearFeedbackController::compute_control(
    const TimePoint& time, const Sensor& sensor, const Control& control,
    const bool remove_gravity_compensation_effort) {
  // Shortcuts for easier code writing.
  const auto& sensor_js = sensor.joint_state;
  const auto& ctrl_js = control.initial_state.joint_state;

  // Self documented variables.
  const bool control_msg_received = !control.feedforward.hasNaN();
  const bool first_control_received_time_initialized =
      first_control_received_time_ != TimePoint::min();
  const bool during_switch = (time - first_control_received_time_) <
                             params_.pd_to_lf_transition_duration;

  // Check whenever the first data has arrived and save the time.
  if (control_msg_received && !first_control_received_time_initialized) {
    first_control_received_time_ = time;
  }

  if (!first_control_received_time_initialized) {
    control_ =
        pd_controller_.compute_control(sensor_js.position, sensor_js.velocity);
  } else if (during_switch) {
    double weight = ((time - first_control_received_time_).count()) /
                    params_.pd_to_lf_transition_duration.count();
    weight = std::clamp(weight, 0.0, 1.0);
    const Eigen::VectorXd& pd_ctrl =
        pd_controller_.compute_control(sensor_js.position, sensor_js.velocity);
    const Eigen::VectorXd& lf_ctrl =
        lf_controller_.compute_control(sensor, control);

    control_ = (1.0 - weight) * pd_ctrl + weight * lf_ctrl;
  } else {
    control_ = lf_controller_.compute_control(sensor, control);
  }

  if (remove_gravity_compensation_effort) {
    robot_model_builder_->construct_robot_state(sensor, robot_configuration_,
                                                robot_velocity_);

    // NOTE: .tail() is used to remove the freeflyer components
    control_ -=
        pinocchio::rnea(robot_model_builder_->get_model(),
                        robot_model_builder_->get_data(), robot_configuration_,
                        robot_velocity_null_, robot_velocity_null_)
            .tail(control_.size());
  }

  return control_;
}

RobotModelBuilder::ConstSharedPtr LinearFeedbackController::get_robot_model()
    const {
  return robot_model_builder_;
}

}  // namespace linear_feedback_controller
