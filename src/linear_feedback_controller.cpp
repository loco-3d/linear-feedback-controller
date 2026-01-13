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

  // DEBUG
  std::cout << "[LFC] load(): nq=" << robot_model_builder_->get_nq()
            << ", nv=" << robot_model_builder_->get_nv() << ", joint_cfg_nq="
            << robot_model_builder_->get_joint_configuration_nq()
            << ", joint_pos_nq="
            << robot_model_builder_->get_joint_position_nq()
            << ", joint_nv=" << robot_model_builder_->get_joint_nv()
            << std::endl;

  // Setup the pd controller.
  pd_controller_.set_gains(params_.p_gains, params_.d_gains);

  // DEBUG : ordre des joints / indices <<<
  std::cout << "[LFC] load(): controlled joints from params ("
            << params_.controlled_joint_names.size() << "):" << std::endl;
  for (int i = 0; i < (int)params_.controlled_joint_names.size(); ++i) {
    std::cout << "  idx " << i << " -> " << params_.controlled_joint_names[i]
              << std::endl;
  }
  std::cout << "[LFC] load(): p_gains size=" << params_.p_gains.size()
            << ", d_gains size=" << params_.d_gains.size() << std::endl;

  // Setup the lfc controller.
  lf_controller_.initialize(robot_model_builder_);

  // Allocate memory
  robot_configuration_ = Eigen::VectorXd::Zero(robot_model_builder_->get_nq());
  robot_velocity_ = Eigen::VectorXd::Zero(robot_model_builder_->get_nv());
  robot_velocity_null_ = Eigen::VectorXd::Zero(robot_model_builder_->get_nv());

  tau_init_ = Eigen::VectorXd::Zero(robot_model_builder_->get_joint_nv());
  tau_gravity_ = Eigen::VectorXd::Zero(robot_model_builder_->get_joint_nv());
  control_pd_ = Eigen::VectorXd::Zero(robot_model_builder_->get_joint_nv());
  control_lf_ = Eigen::VectorXd::Zero(robot_model_builder_->get_joint_nv());

  // DEBUG
  std::cout << "[LFC] load(): tau_init size=" << tau_init_.size()
            << ", p_gains size=" << params_.p_gains.size() << std::endl;

  return true;
}

bool LinearFeedbackController::set_initial_state(
    const Eigen::VectorXd& tau_init, const Eigen::VectorXd& jq_init) {
  const int joint_nq = robot_model_builder_->get_joint_position_nq();
  const int joint_nv = robot_model_builder_->get_joint_nv();

  std::cout << "[LFC] set_initial_state: tau_init size=" << tau_init.size()
            << ", jq_init size=" << jq_init.size() << ", joint_nq=" << joint_nq
            << ", joint_nv=" << joint_nv << std::endl;

  // Vérif sur tau_init
  if (tau_init.size() != joint_nv) {
    std::stringstream ss;
    ss << "[LFC] set_initial_state: tau_init size=" << tau_init.size()
       << " but expected " << joint_nv;
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument(ss.str());
  }

  // vérif jq_init size
  if (jq_init.size() != joint_nq) {
    std::stringstream ss;
    ss << "[LFC] set_initial_state: jq_init size= " << jq_init.size()
       << " but expected " << joint_nq;
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument(ss.str());
  }

  Eigen::VectorXd q_ref_pd = jq_init;

  // DEBUG
  std::cout << "[LFC] set_initial_state : tau_init size=" << tau_init.size()
            << ", q_ref_pd size=" << q_ref_pd.size() << std::endl;
  std::cout << "[LFC] set_initial_state: q_ref_pd = " << q_ref_pd.transpose()
            << std::endl;
  std::cout << "[LFC] set_initial_state: tau_init = " << tau_init.transpose()
            << std::endl;

  pd_controller_.set_reference(tau_init, q_ref_pd);
  tau_init_ = tau_init;
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

  if (remove_gravity_compensation_effort) {
    robot_model_builder_->construct_robot_state(sensor, robot_configuration_,
                                                robot_velocity_);

    // NOTE: .tail() is used to remove the freeflyer components
    tau_gravity_ =
        pinocchio::rnea(robot_model_builder_->get_model(),
                        robot_model_builder_->get_data(), robot_configuration_,
                        robot_velocity_null_, robot_velocity_null_)
            .tail(tau_init_.size());
  }

  // inputs for PD controller
  const int joint_pos_nq = robot_model_builder_->get_joint_position_nq();
  const int joint_nv = robot_model_builder_->get_joint_nv();

  // Sanity check design : PD / hardware vivent en joint_position_nq == joint_nv
  if (joint_pos_nq != joint_nv) {
    std::stringstream ss;
    ss << "[LFC] compute_control: design error, joint_pos_nq=" << joint_pos_nq
       << " but joint_nv=" << joint_nv;
    std::cerr << ss.str() << std::endl;
    throw std::logic_error(ss.str());
  }

  // Size verification on the sizes coming from ROS
  if (sensor_js.position.size() != joint_pos_nq) {
    std::stringstream ss;
    ss << "[LFC] compute_control: unexpected sensor_js.position size="
       << sensor_js.position.size() << " (expected " << joint_pos_nq
       << " = joint_position_nq)";
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument(ss.str());
  }

  if (sensor_js.velocity.size() != joint_nv) {
    std::stringstream ss;
    ss << "[LFC] compute_control: unexpected sensor_js.velocity size="
       << sensor_js.velocity.size() << " (expected " << joint_nv
       << " = joint_nv)";
    std::cerr << ss.str() << std::endl;
    throw std::invalid_argument(ss.str());
  }

  Eigen::VectorXd q_pd = sensor_js.position;
  Eigen::VectorXd v_pd = sensor_js.velocity;

  if (!first_control_received_time_initialized) {
    control_ = pd_controller_.compute_control(q_pd, v_pd);

    if (remove_gravity_compensation_effort) {
      control_ -= tau_init_;
    }
  } else if (during_switch) {
    double weight = ((time - first_control_received_time_).count()) /
                    params_.pd_to_lf_transition_duration.count();
    weight = std::clamp(weight, 0.0, 1.0);

    control_pd_ = pd_controller_.compute_control(q_pd, v_pd);
    control_lf_ = lf_controller_.compute_control(sensor, control);

    if (remove_gravity_compensation_effort) {
      control_pd_ -= tau_init_;
      control_lf_ -= tau_gravity_;
    }

    control_.noalias() = (1.0 - weight) * control_pd_ + weight * control_lf_;
  } else {
    control_ = lf_controller_.compute_control(sensor, control);

    if (remove_gravity_compensation_effort) {
      control_ -= tau_gravity_;
    }
  }

  return control_;
}

RobotModelBuilder::ConstSharedPtr LinearFeedbackController::get_robot_model()
    const {
  return robot_model_builder_;
}

}  // namespace linear_feedback_controller
