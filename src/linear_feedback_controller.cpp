#include "linear_feedback_controller/linear_feedback_controller.hpp"

#include <algorithm>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <flex-joints/flexi-hips.hpp>

namespace linear_feedback_controller {

LinearFeedbackController::LinearFeedbackController()
    : pal_base_ros_controller::BaseRobotWithEsimatorController() {}

LinearFeedbackController::~LinearFeedbackController() {}

bool LinearFeedbackController::loadEtras(ros::NodeHandle& node_handle) {
  // Parse the parameters.
  if (!parseRosParams(node_handle)) {
    return false;
  }

  // Build the rigid body model of the robot.
  if (in_robot_has_free_flyer_) {
    pinocchio::urdf::buildModelFromXML(
        in_urdf_, pinocchio::JointModelFreeFlyer(), pinocchio_model_complete_);
  } else {
    pinocchio::urdf::buildModelFromXML(in_urdf_, pinocchio_model_complete_);
  }
  std::istringstream iss_srdf(in_srdf_);
  pinocchio::srdf::loadReferenceConfigurationsFromXML(pinocchio_model_complete_,
                                                      iss_srdf, false);

  // Reduce the rigid body model and set initial position.
  if (!parseMovingJointNames(in_moving_joint_names_, moving_joint_names_,
                             moving_joint_ids_, locked_joint_ids_)) {
    return false;
  }
  q_default_complete_ =
      pinocchio_model_complete_.referenceConfigurations["half_sitting"];
  pinocchio_model_reduced_ = pinocchio::buildReducedModel(
      pinocchio_model_complete_, locked_joint_ids_, q_default_complete_);
  pinocchio_data_reduced_ = pinocchio::Data(pinocchio_model_reduced_);

  // Resize the control vector. They are the size of the reduced model.
  desired_configuration_.resize(pinocchio_model_reduced_.nq);
  desired_configuration_.fill(0.0);
  desired_velocity_.resize(pinocchio_model_reduced_.nv);
  desired_velocity_.fill(0.0);
  measured_configuration_.resize(pinocchio_model_reduced_.nq);
  measured_configuration_.fill(0.0);
  measured_velocity_.resize(pinocchio_model_reduced_.nv);
  measured_velocity_.fill(0.0);
  diff_state_.resize(2 * pinocchio_model_reduced_.nv);
  diff_state_.fill(0.0);
  std_joint_position_compensated_ = getActualJointPositions();
  std_joint_velocity_compensated_ = getActualJointVelocities();
  ei_joint_position_compensated_ = Eigen::VectorXd::Zero(std_joint_position_compensated_.size());
  ei_joint_velocity_compensated_ = Eigen::VectorXd::Zero(std_joint_velocity_compensated_.size());
  ei_joint_desired_torques_= Eigen::VectorXd::Zero(std_joint_velocity_compensated_.size());
  // Prepare the publisher and subscriber exchanging the control and state.

  sensor_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<
      linear_feedback_controller_msgs::Sensor> >(node_handle, "sensor_state",
                                                 1);
  ros::TransportHints hints;
  hints.tcpNoDelay(true);
  control_subscriber_ = node_handle.subscribe(
      "desired_control", 1,
      &LinearFeedbackController::controlSubscriberCallback, this, hints);

  // Filter the initial measured joint position and torque and save it.
  filterInitialState();

  // Allocate the memory of the various vector which are joint number dependent.
  ros_sensor_msg_.joint_state.name = moving_joint_names_;
  ros_sensor_msg_.joint_state.position.resize(moving_joint_names_.size(), 0.0);
  ros_sensor_msg_.joint_state.velocity.resize(moving_joint_names_.size(), 0.0);
  ros_sensor_msg_.joint_state.effort.resize(moving_joint_names_.size(), 0.0);
  eigen_sensor_msg_.joint_state.name = moving_joint_names_;
  eigen_sensor_msg_.joint_state.position.setZero(moving_joint_names_.size());
  eigen_sensor_msg_.joint_state.velocity.setZero(moving_joint_names_.size());
  eigen_sensor_msg_.joint_state.effort.setZero(moving_joint_names_.size());
  eigen_sensor_msg_.contacts.resize(2);
  eigen_control_msg_.initial_state.joint_state.position.setZero(
      moving_joint_names_.size());
  eigen_control_msg_.initial_state.joint_state.velocity.setZero(
      moving_joint_names_.size());
  eigen_control_msg_.initial_state.joint_state.effort.setZero(
      moving_joint_names_.size());
  eigen_control_msg_.initial_state.contacts.resize(2);
  eigen_control_msg_.feedback_gain.setZero(moving_joint_names_.size(),
                                           2 * pinocchio_model_reduced_.nv);
  eigen_control_msg_.feedforward.setZero(moving_joint_names_.size(),
                                         2 * pinocchio_model_reduced_.nv);
  eigen_control_msg_copy_ = eigen_control_msg_;

  contact_detectors_.resize(2);
  for (auto ct : contact_detectors_) {
    ct.setLowerThreshold(30);
    ct.setUpperThreshold(150);
    ct.setThresholdContactCounter(10);
  }
  pd_desired_torque_.resize(getControlledJointNames().size());
  pd_desired_torque_.fill(0.0);
  lf_desired_torque_.resize(moving_joint_names_.size());
  lf_desired_torque_.fill(0.0);
  /// @todo automatize this.
  eigen_sensor_msg_.contacts[0].name = "left_sole_link";
  eigen_sensor_msg_.contacts[1].name = "right_sole_link";
  desired_stance_ids_ = std::vector<std::string>();
  desired_swing_ids_ = std::vector<std::string>();
  desired_stance_ids_.push_back("left_sole_link");
  desired_stance_ids_.push_back("right_sole_link");

  min_jerk_.setParameters(from_pd_to_lf_duration_, 1.0);

  /// @todo Automatize the parametrization of these gains.
  p_arm_gain_ = 100.0;  // 500.0
  d_arm_gain_ = 8.0;    // 20.0
  p_torso_gain_ = 500.0;
  d_torso_gain_ = 20.0;
  p_leg_gain_ = 800.0;  // 100.0
  d_leg_gain_ = 35.0;   // 8.0; //50.0

  // Mutex watch dog deadline.
  ms_mutex_ = (int)getControllerDt().toSec() * 1000;

  // Register all variables that we wanna log.
  initializeIntrospection();

  // build flexibility compensator:
   flex::FlexSettings flex_settings;
   flex_settings.dt = getControllerDt().toSec();
   flex_settings.left_stiffness = eVector2(2800., 2800.);
   flex_settings.right_stiffness = eVector2(4200., 4200.);
   flex_settings.left_damping =
       2 * flex_settings.left_stiffness.cwiseSqrt();
   flex_settings.right_damping =
       2 * flex_settings.right_stiffness.cwiseSqrt();
   std::vector<std::string> hip_left_flexible_joints = {{"leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint"}};
   std::vector<std::string> hip_right_flexible_joints = {{"leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint"}};
   flex_settings.left_hip_indices = Eigen::Array3i(); // indexes of left hip z, x, y joints in jointState vector
   flex_settings.right_hip_indices = Eigen::Array3i(); // indexes of right hip z, x, y joints in jointState vector
   for (int i = 0; i < 3; ++i)
   {
     flex_settings.left_hip_indices[i] = indexVector(
         getControlledJointNames(), hip_left_flexible_joints[static_cast<size_t>(i)]);
     flex_settings.right_hip_indices[i] = indexVector(
         getControlledJointNames(), hip_right_flexible_joints[static_cast<size_t>(i)]);
   }
   flexibility_compensator_.initialize(flex_settings);


  ROS_INFO_STREAM("LinearFeedbackController::loadEtras(): Done.");
  return true;
}

void LinearFeedbackController::updateExtras(const ros::Time& time,
                                            const ros::Duration& period) {
  // Shortcuts for easier code writing.
  const linear_feedback_controller_msgs::Eigen::JointState& sensor_js =
      eigen_sensor_msg_.joint_state;
  const linear_feedback_controller_msgs::Eigen::JointState& ctrl_js =
      eigen_control_msg_.initial_state.joint_state;

  acquireSensorAndPublish(time, period);

  // Lock the mutex here to not have conflict with the subscriber callback.
  std::unique_lock<std::timed_mutex> lock(mutex_, std::try_to_lock);

  // If the control message is not empty then it means there is a controller
  // sending messages. Hence we update it. We also test if the lock is properly
  // acquired. It could happen that at this point a new control message is
  // arriving. Hence we skip the update from the message.
  if (lock.owns_lock() && (!ctrl_js.name.empty())) {
    eigen_control_msg_copy_ = eigen_control_msg_;  // for logging
    ROS_INFO_ONCE("LinearFeedbackController::updateExtras(): compute lfc.");
    // compute the freeze robot action and create a smooth transition between
    // the 2 controllers
    pdController();
    lfController();

    // double weight = min_jerk_.compute((init_lfc_time_ - time).toSec());
    double weight = ((time - init_lfc_time_).toSec()) / from_pd_to_lf_duration_;
    if (weight < 0.0) {
      weight = 0.0;
    } else if (weight > 1.0) {
      weight = 1.0;
    }

    for (std::size_t i = 0; i < ctrl_js.name.size(); ++i) {
      double weighted_desired_torque =
          weight * lf_desired_torque_(i) +
          (1 - weight) * pd_desired_torque_(pin_to_hwi_[i]);
      setDesiredJointState(ctrl_js.name[i],      //
                           ctrl_js.position[i],  //
                           ctrl_js.velocity[i],  //
                           0.0,                  // Acceleration.
                           weighted_desired_torque);
      ei_joint_desired_torques_[pin_to_hwi_[i]] = weighted_desired_torque;
    }
  } else if (ctrl_js.name.empty())  // No control has been sent yet.
  {
    ROS_INFO_ONCE("LinearFeedbackController::updateExtras(): compute pdc.");
    pdController();
    for (std::size_t i = 0; i < sensor_js.name.size(); i++) {
      // Here we need the hardware interface indexing.
      setDesiredJointState(getControlledJointNames()[pin_to_hwi_[i]],
                           initial_position_[pin_to_hwi_[i]],  //
                           0.0,                                // Velocity
                           0.0,                                // Acceleration
                           pd_desired_torque_(pin_to_hwi_[i]));
      ei_joint_desired_torques_[pin_to_hwi_[i]] = pd_desired_torque_(pin_to_hwi_[i]);

    }
    init_lfc_time_ = time;
  } else {
    ROS_INFO_STREAM(
        "LinearFeedbackController::updateExtras(): Don't do anything.");
    ROS_WARN("Skipping received state");
  }
  // Define the active contacts used for the base estimation.
  setEstimatorContactState(desired_stance_ids_, desired_swing_ids_);
  // Log data from the controller.
  computeIntrospection();
}

void LinearFeedbackController::startingExtras(const ros::Time& /*time*/) {
  // Filter the initial measured joint position and torque and save it.
  // filterInitialState();
}

void LinearFeedbackController::stoppingExtra(const ros::Time& /*time*/) {
  // Kill the robot, i.e. send zero torques to all joints.
  for (std::size_t i = 0; i < getControlledJointNames().size(); ++i) {
    setDesiredJointState(i, 0., 0., 0., 0.);
  }
}

bool LinearFeedbackController::parseMovingJointNames(
    const std::vector<std::string>& in_moving_joint_names,
    std::vector<std::string>& moving_joint_names,
    std::vector<long unsigned int>& moving_joint_ids,
    std::vector<long unsigned int>& locked_joint_ids) {
  // Get moving joints ids
  ROS_DEBUG_STREAM("Map the given moving joints names with their urdf ids");
  moving_joint_ids.clear();
  for (std::vector<std::string>::const_iterator it =
           in_moving_joint_names.begin();
       it != in_moving_joint_names.end(); ++it) {
    const std::string& joint_name = *it;
    pinocchio::JointIndex joint_id = 0;
    // do not consider joint that are not in the model
    if (pinocchio_model_complete_.existJointName(joint_name)) {
      joint_id = pinocchio_model_complete_.getJointId(joint_name);
      moving_joint_ids.push_back(joint_id);
      ROS_DEBUG_STREAM("joint_name=" << joint_name
                                     << ", joint_id=" << joint_id);
    } else {
      ROS_WARN_STREAM("joint_name=" << joint_name
                                    << " does not belong to the model");
    }
  }
  if (in_robot_has_free_flyer_) {
    moving_joint_ids.push_back(
        pinocchio_model_complete_.getJointId("root_joint"));
  }
  // Sort them to the pinocchio order (increasing number) and remove duplicates.
  std::sort(moving_joint_ids.begin(), moving_joint_ids.end());
  moving_joint_ids.erase(
      unique(moving_joint_ids.begin(), moving_joint_ids.end()),
      moving_joint_ids.end());

  // Remap the moving joint names to the Pinocchio order.
  moving_joint_names.clear();
  for (std::size_t i = 0; i < moving_joint_ids.size(); ++i) {
    moving_joint_names.push_back(
        pinocchio_model_complete_.names[moving_joint_ids[i]]);
  }

  // Locked joint ids in the Pinocchio order.
  locked_joint_ids.clear();
  for (std::vector<std::string>::const_iterator it =
           pinocchio_model_complete_.names.begin() + 1;
       it != pinocchio_model_complete_.names.end(); ++it) {
    const std::string& joint_name = *it;
    if (std::find(moving_joint_names.begin(), moving_joint_names.end(),
                  joint_name) == moving_joint_names.end()) {
      locked_joint_ids.push_back(
          pinocchio_model_complete_.getJointId(joint_name));
    }
  }

  // remove the "root_joint" if it exists in the moving_joint_names, we needed
  // it to create the above locked_joint_ids needed for the reduced model.
  auto root_name = std::remove(moving_joint_names.begin(),
                               moving_joint_names.end(), "root_joint");
  if (root_name != moving_joint_names.end()) {
    moving_joint_names.erase(root_name, moving_joint_names.end());
    auto root_id =
        std::remove(moving_joint_ids.begin(), moving_joint_ids.end(),
                    pinocchio_model_complete_.getJointId("root_joint"));
    moving_joint_ids.erase(root_id, moving_joint_ids.end());
  }

  // Create the joint name joint id mapping of the hardware interface.
  for (std::size_t i = 0; i < moving_joint_names.size(); ++i) {
    auto it = std::find(getControlledJointNames().begin(),
                        getControlledJointNames().end(), moving_joint_names[i]);
    // verify that the found moving joints are part of the hardware interface.
    if (it == getControlledJointNames().end()) {
      ROS_ERROR_STREAM(
          "LinearFeedbackController::parseMovingJointNames(): "
          << "Moving joint " << moving_joint_names[i]
          << " is not part of the current roscontrol hardware interface.");
      return false;
    }
    std::size_t it_dist =
        (size_t)std::distance(getControlledJointNames().begin(), it);
    pin_to_hwi_.emplace(std::make_pair(i, it_dist));
  }
  // Double check what we just did
  for (std::size_t i = 0; i < moving_joint_names.size(); ++i) {
    if (moving_joint_names[i] != getControlledJointNames()[pin_to_hwi_[i]]) {
      ROS_ERROR_STREAM("LinearFeedbackController::parseMovingJointNames(): "
                       << "The mapping from pinocchio to roscontrol is wrong. "
                       << "moving_joint_names[i] != "
                          "getControlledJointNames()[pin_to_hwi_[i]] => "
                       << moving_joint_names[i]
                       << " != " << getControlledJointNames()[pin_to_hwi_[i]]);
      return false;
    }
  }

  return true;
}

void LinearFeedbackController::lfController() {
  // Shortcuts for easier code writing.
  const linear_feedback_controller_msgs::Eigen::JointState& sensor_js =
      eigen_sensor_msg_.joint_state;
  const linear_feedback_controller_msgs::Eigen::JointState& ctrl_js =
      eigen_control_msg_.initial_state.joint_state;
  const linear_feedback_controller_msgs::Eigen::Sensor& ctrl_init =
      eigen_control_msg_.initial_state;

  // Reconstruct the state vector: x = [q, v]
  if (in_robot_has_free_flyer_) {
    desired_configuration_.head<7>() = ctrl_init.base_pose;
    desired_velocity_.head<6>() = ctrl_init.base_twist;
    measured_configuration_.head<7>() = eigen_sensor_msg_.base_pose;
    measured_velocity_.head<6>() = eigen_sensor_msg_.base_twist;
  }
  int nb_joint = ctrl_js.name.size();
  desired_configuration_.tail(nb_joint) = ctrl_js.position;
  desired_velocity_.tail(nb_joint) = ctrl_js.velocity;
  measured_configuration_.tail(nb_joint) = sensor_js.position;
  measured_velocity_.tail(nb_joint) = sensor_js.velocity;

  // Compute the linear feedback controller desired torque.
  pinocchio::difference(pinocchio_model_reduced_, measured_configuration_,
                        desired_configuration_,
                        diff_state_.head(pinocchio_model_reduced_.nv));
  diff_state_.tail(pinocchio_model_reduced_.nv) =
      desired_velocity_ - measured_velocity_;
  lf_desired_torque_ = eigen_control_msg_.feedforward +
                       eigen_control_msg_.feedback_gain * diff_state_;

  // Define the support foot
  /// @todo automatize this in the estimator?
  desired_swing_ids_.clear();
  desired_stance_ids_.clear();
  if (ctrl_init.contacts[0].active) {
    desired_stance_ids_.push_back("left_sole_link");
  } else {
    desired_swing_ids_.push_back("left_sole_link");
  }
  if (ctrl_init.contacts[1].active) {
    desired_stance_ids_.push_back("right_sole_link");
  } else {
    desired_swing_ids_.push_back("right_sole_link");
  }
}

void LinearFeedbackController::pdController() {
  pd_desired_torque_.fill(0.0);
  for (std::size_t i = 0; i < getControlledJointNames().size(); i++) {
    const int i_int = static_cast<int>(i);
    if (getControlledJointNames()[i].find("torso") != std::string::npos) {
      pd_desired_torque_(i) = initial_torque_[i] -
                              p_torso_gain_ * (getActualJointPosition(i_int) -
                                               initial_position_[i]) -
                              d_torso_gain_ * getActualJointVelocity(i_int);
    } else if (getControlledJointNames()[i].find("arm") != std::string::npos) {
      pd_desired_torque_(i) =
          initial_torque_[i] -
          p_arm_gain_ * (getActualJointPosition(i_int) - initial_position_[i]) -
          d_arm_gain_ * getActualJointVelocity(i_int);
    } else {
      pd_desired_torque_(i) =
          initial_torque_[i] -
          p_leg_gain_ * (getActualJointPosition(i_int) - initial_position_[i]) -
          d_leg_gain_ * getActualJointVelocity(i_int);
    }
  }
}

void LinearFeedbackController::controlSubscriberCallback(
    const linear_feedback_controller_msgs::Control& msg) {
  std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
  if (lock.try_lock_for(std::chrono::milliseconds(ms_mutex_))) {
    linear_feedback_controller_msgs::controlMsgToEigen(msg, eigen_control_msg_);
  }
}

// Get the parameters of the node return false in case of failure.
#define GET_ROS_PARAM(nh, name, var)                                      \
  {                                                                       \
    std::string name_searched = name;                                     \
    if (nh.searchParam(name, name_searched)) {                            \
      nh.getParam(name_searched, var);                                    \
    } else {                                                              \
      ROS_ERROR_STREAM(                                                   \
          "LinearFeedbackController::parseRosParams(): Missing ROS arg: " \
          << name_searched);                                              \
      return false;                                                       \
    }                                                                     \
    ROS_DEBUG_STREAM("LinearFeedbackController::parseRosParams(): "       \
                     << name_searched << ": " << var);                    \
  }

bool LinearFeedbackController::parseRosParams(ros::NodeHandle& node_handle) {
  // Get the parameters of the node return false in case of failure.
  GET_ROS_PARAM(node_handle, "robot_description", in_urdf_);
  GET_ROS_PARAM(node_handle, "robot_description_semantic", in_srdf_);
  GET_ROS_PARAM(node_handle, "moving_joint_names", in_moving_joint_names_);
  GET_ROS_PARAM(node_handle, "robot_has_free_flyer", in_robot_has_free_flyer_);
  GET_ROS_PARAM(node_handle, "lower_force_threshold",
                in_lower_force_threshold_);
  GET_ROS_PARAM(node_handle, "upper_force_threshold",
                in_upper_force_threshold_);
  GET_ROS_PARAM(node_handle, "threshold_contact_counter",
                in_threshold_contact_counter_);
  {
    std::string name_searched = "robot_has_free_flyer";
    if (node_handle.searchParam(name_searched, name_searched)) {
      node_handle.getParam(name_searched, in_robot_has_free_flyer_);
    } else {
      in_robot_has_free_flyer_ = true;
    }
    ROS_DEBUG_STREAM(
        "LinearFeedbackController::parseRosParams(): robot_has_free_flyer: "
        << in_robot_has_free_flyer_);
  }
  GET_ROS_PARAM(node_handle, "from_pd_to_lf_duration", from_pd_to_lf_duration_);
  size_t nbjoint = getControlledJointNames().size();
  in_torque_offsets_.resize(nbjoint, 0.0);
  for (size_t i = 0; i < nbjoint; ++i) {
    std::string param_name = "joints/" + getControlledJointNames()[i] +
                             "/actuator_params/torque_sensor_offset";
    GET_ROS_PARAM(node_handle, param_name, in_torque_offsets_[i]);
  }
  ROS_INFO_STREAM(
      "LinearFeedbackController::parseRosParams(): Loading parameters... "
      "Done.");
  return true;
}

void LinearFeedbackController::filterInitialState() {
  std::size_t nb_joint = getControlledJointNames().size();
  initial_position_.setZero(nb_joint);
  initial_position_filter_.setMaxSize(10);
  initial_torque_.setZero(nb_joint);
  initial_torque_filter_.setMaxSize(10);

  ros::Time start_time =
      ros::Time::now() -
      ros::Duration(0, 1000000);  // An epsilon time before now;
  while ((ros::Time::now() - start_time) < ros::Duration(1.0)) {
    for (std::size_t i = 0; i < nb_joint; ++i) {
      initial_position_(i) = getActualJointPosition(i);
      initial_torque_(i) = getJointMeasuredTorque(i) - in_torque_offsets_[i];
    }
    initial_position_filter_.acquire(initial_position_);
    initial_torque_filter_.acquire(initial_torque_);
    ros::Duration(0.01).sleep();
  }
  initial_torque_ = initial_torque_filter_.getFilteredData();
  initial_position_ = initial_position_filter_.getFilteredData();
}

void LinearFeedbackController::acquireSensorAndPublish(
    const ros::Time& time, const ros::Duration& /*period*/) {
  /// @todo Filter the data here?

  // Fill in the base data.
  eigen_sensor_msg_.base_pose.head<3>() = getEstimatedFloatingBasePosition();
  Eigen::Quaterniond q_base = getEstimatedFloatingBaseOrientation();
  q_base.normalize();
  eigen_sensor_msg_.base_pose.tail<4>() = q_base.coeffs();
  eigen_sensor_msg_.base_twist.head<3>() =
      getEstimatedFloatingBaseLinearVelocity();
  eigen_sensor_msg_.base_twist.tail<3>() =
      getEstimatedFloatingBaseAngularVelocity();

  // Fill the joint state data, i.e. the position, velocity and torque.
  const auto& act_jp = getActualJointPositions();
  const auto& act_jv = getActualJointVelocities();
  ei_joint_position_compensated_ = ::Eigen::Map<const ::Eigen::VectorXd>(
      act_jp.data(), static_cast<Eigen::Index>(act_jp.size()));
  ei_joint_velocity_compensated_ = ::Eigen::Map<const ::Eigen::VectorXd>(
      act_jv.data(), static_cast<Eigen::Index>(act_jv.size()));
  // Flexibility compensation
  if (!ei_joint_desired_torques_.isZero())  // @TODO: torques exists
  {
    // This update jointState_ and jointStateVelocity_
    flexibility_compensator_.correctEstimatedDeflections(ei_joint_desired_torques_,
                                                         ei_joint_position_compensated_,
                                                         ei_joint_velocity_compensated_);
    hip_deflection_left_roll_ =  flexibility_compensator_.getLeftFlex()[1];
    hip_deflection_left_pitch_ =  flexibility_compensator_.getLeftFlex()[0];
    hip_deflection_right_roll_ =  flexibility_compensator_.getRightFlex()[1];
    hip_deflection_right_pitch_ =  flexibility_compensator_.getRightFlex()[0];
  }
  else
  {
    ROS_WARN("ei_joint_desired_torques_ is zero");
  }

  std::size_t nb_joint = eigen_sensor_msg_.joint_state.position.size();
  for (std::size_t i = 0; i < nb_joint; ++i) {
    eigen_sensor_msg_.joint_state.position(i) = ei_joint_position_compensated_[pin_to_hwi_[i]];
    eigen_sensor_msg_.joint_state.velocity(i) = ei_joint_velocity_compensated_[pin_to_hwi_[i]];
    eigen_sensor_msg_.joint_state.effort(i) =
        getJointMeasuredTorque(pin_to_hwi_[i]) -
        in_torque_offsets_[pin_to_hwi_[i]];
  }

  // Construct the measured state and perform the forward kinematics for the
  // feet location.
  if (in_robot_has_free_flyer_) {
    measured_configuration_.head<7>() = eigen_sensor_msg_.base_pose;
    measured_velocity_.head<6>() = eigen_sensor_msg_.base_twist;
  }
  measured_configuration_.tail(nb_joint) =
      eigen_sensor_msg_.joint_state.position;
  measured_velocity_.tail(nb_joint) = eigen_sensor_msg_.joint_state.velocity;
  pinocchio::forwardKinematics(pinocchio_model_reduced_,
                               pinocchio_data_reduced_,
                               measured_configuration_);
  pinocchio::updateFramePlacements(pinocchio_model_reduced_,
                                   pinocchio_data_reduced_);

  // Fill the information about the force sensor and if the corresponding
  // end-effector are in contact or not.
  const std::vector<pal_controller_interface::FTSensorDefinitionPtr>&
      ft_sensors = getForceTorqueSensorDefinitions();
  assert(ft_sensors.size() >= 2 && "There must be at least 2 force sensor.");
  for (std::size_t i = 0; i < 2; ++i) {
    eigen_sensor_msg_.contacts[i].wrench.head<3>() = ft_sensors[i]->force;
    eigen_sensor_msg_.contacts[i].wrench.tail<3>() = ft_sensors[i]->torque;
    eigen_sensor_msg_.contacts[i].active = contact_detectors_[i].detectContact(
        eigen_sensor_msg_.contacts[i].wrench);
    pinocchio::SE3& contact_se3 =
        pinocchio_data_reduced_.oMf[pinocchio_model_reduced_.getFrameId(
            eigen_sensor_msg_.contacts[i].name)];
    eigen_sensor_msg_.contacts[i].pose.head<3>() = contact_se3.translation();
    Eigen::Map<pinocchio::SE3::Quaternion>(
        eigen_sensor_msg_.contacts[i].pose.tail<4>().data()) =
        contact_se3.rotation();
  }

  // Publish the message to the ROS topic.
  linear_feedback_controller_msgs::sensorEigenToMsg(eigen_sensor_msg_,
                                                    ros_sensor_msg_);
  ros_sensor_msg_.header.stamp = time;

  bool locked = false;
  const std::size_t nb_trials = 500;
  std::size_t trial_counter = 0;
  while (!locked && trial_counter < nb_trials /*&&
         (ros::Time::now() - time).toSec() < 0.1 * period.toSec()*/) {
    if (sensor_publisher_->trylock()) {
      sensor_publisher_->msg_ = ros_sensor_msg_;
      sensor_publisher_->unlockAndPublish();
      locked = true;
      break;
    }
    ++trial_counter;
  }
  if (trial_counter > 0) {
    ROS_DEBUG_STREAM("LinearFeedbackController::updateExtras(): "
                     << "Tried to lock the publisher mutex, " << trial_counter
                     << " times.");
  }
  if (!locked) {
    ROS_DEBUG_STREAM("LinearFeedbackController::updateExtras(): "
                     << "Cannot lock the publisher, "
                     << "the sensor message is not sent.");
  }
}

void LinearFeedbackController::computeCOP(
    const std::vector<linear_feedback_controller_msgs::Eigen::Contact>&
        contacts,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >&
        local_cops,
    Eigen::Vector3d& cop) {
  cop.setZero();
  double total_z_force = 0.0;
  assert(contacts.size() == local_cops.size() &&
         "There must as must CoPs as contacts.");
  for (size_t i = 0; i < contacts.size(); ++i) {
    auto force = contacts[i].wrench.head<3>();
    auto torque = contacts[i].wrench.tail<3>();
    auto pose = contacts[i].pose.head<3>();
    Eigen::Matrix3d rotation =
        Eigen::Quaterniond(contacts[i].pose(6), contacts[i].pose(3),
                           contacts[i].pose(4), contacts[i].pose(5))
            .matrix();
    if (std::abs(force.z()) > 1.) {
      local_cops[i] << -torque.y() / force.z(), torque.x() / force.z(), 0.0;
      cop += (rotation * local_cops[i] + pose) * force.z();
      total_z_force += force.z();
    }
  }

  // Store the result in this variable.
  if (total_z_force < 1.)
    cop.setZero();
  else {
    cop /= total_z_force;
  }
}

void LinearFeedbackController::computeCOMAccelerationFromForces(
    const double mass, const eVector3& gravity_vector,
    const std::vector<linear_feedback_controller_msgs::Eigen::Contact>&
        contacts,
    Eigen::Vector3d& com_acc) {
  Eigen::Vector3d sum_force(Eigen::Vector3d::Zero());
  for (size_t i = 0; i < contacts.size(); ++i) {
    Eigen::Matrix3d rotation =
        Eigen::Quaterniond(contacts[i].pose(6), contacts[i].pose(3),
                           contacts[i].pose(4), contacts[i].pose(5))
            .matrix();
    sum_force += rotation * contacts[i].wrench.head<3>();
  }
  com_acc = sum_force / mass + gravity_vector;
}

void LinearFeedbackController::computeLIPBase(
    const Eigen::Vector3d& com_position,
    const Eigen::Vector3d& com_acceleration, const eVector3& gravity_vector,
    const std::vector<linear_feedback_controller_msgs::Eigen::Contact>&
        contacts,
    Eigen::Vector3d& lip_base) {
  // cop = com + h/g * ddcom
  double average_contact_height = 0.0;
  for (size_t i = 0; i < contacts.size(); ++i) {
    average_contact_height += contacts[i].pose(2);
  }
  average_contact_height /= static_cast<double>(contacts.size());
  double h = com_position.z() - average_contact_height;
  double g = gravity_vector.norm();

  lip_base.setZero();
  lip_base.head<2>() =
      com_position.head<2>() - h / g * com_acceleration.head<2>();
}

void LinearFeedbackController::computeIntrospection() {
  // Sensor data
  pinocchio::forwardKinematics(pinocchio_model_reduced_,
                               pinocchio_data_reduced_,
                               measured_configuration_);
  pinocchio::updateFramePlacements(pinocchio_model_reduced_,
                                   pinocchio_data_reduced_);
  actual_com_position_ = pinocchio::centerOfMass(
      pinocchio_model_reduced_, pinocchio_data_reduced_,
      measured_configuration_, measured_velocity_, false);
  actual_com_velocity_ = pinocchio_data_reduced_.vcom[0];
  computeCOMAccelerationFromForces(
      pinocchio_data_reduced_.mass[0], pinocchio_model_reduced_.gravity981,
      eigen_sensor_msg_.contacts, actual_com_acceleration_);
  computeCOP(eigen_sensor_msg_.contacts, actual_local_cops_, actual_cop_);
  computeLIPBase(actual_com_position_, actual_com_acceleration_,
                 pinocchio_model_reduced_.gravity981,
                 eigen_sensor_msg_.contacts, actual_lip_base_);
  for (std::size_t i = 0; i < eigen_sensor_msg_.contacts.size(); ++i) {
    actual_feet_positions_[i] = eigen_sensor_msg_.contacts[i].pose.head<3>();
    actual_contact_forces_[i] = eigen_sensor_msg_.contacts[i].wrench.head<3>();
    actual_contact_torques_[i] = eigen_sensor_msg_.contacts[i].wrench.tail<3>();
  }

  computeCOMAccelerationFromForces(
      pinocchio_data_reduced_.mass[0], pinocchio_model_reduced_.gravity981,
      eigen_control_msg_copy_.initial_state.contacts,
      desired_com_acceleration_);
  computeCOP(eigen_control_msg_copy_.initial_state.contacts,
             desired_local_cops_, desired_cop_);
  for (std::size_t i = 0;
       i < eigen_control_msg_copy_.initial_state.contacts.size(); ++i) {
    desired_feet_positions_[i] =
        eigen_control_msg_copy_.initial_state.contacts[i].pose.head<3>();
    desired_contact_forces_[i] =
        eigen_control_msg_copy_.initial_state.contacts[i].wrench.head<3>();
    desired_contact_torques_[i] =
        eigen_control_msg_copy_.initial_state.contacts[i].wrench.tail<3>();
  }
  desired_initial_joint_configuration_ =
      eigen_control_msg_copy_.initial_state.joint_state.position;
  desired_initial_joint_velocity_ =
      eigen_control_msg_copy_.initial_state.joint_state.velocity;
  desired_initial_joint_torques_ =
      eigen_control_msg_copy_.initial_state.joint_state.effort;
  desired_feedforward_torque_ = eigen_control_msg_copy_.feedforward;
}

void LinearFeedbackController::initializeIntrospection() {
  REGISTER_VARIABLE("/introspection_data", "actual_com_position",
                    &actual_com_position_, &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "actual_com_velocity",
                    &actual_com_velocity_, &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "sensor_com_acceleration",
                    &actual_com_acceleration_, &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "actual_cop", &actual_cop_,
                    &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "desired_cop", &desired_cop_,
                    &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "actual_lip_base", &actual_lip_base_,
                    &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "desired_lip_base",
                    &desired_lip_base_, &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "hip_deflection_right_pitch",
                    &hip_deflection_right_pitch_, &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "hip_deflection_right_roll",
                    &hip_deflection_right_roll_, &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "hip_deflection_left_pitch",
                    &hip_deflection_left_pitch_, &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "hip_deflection_left_roll",
                    &hip_deflection_left_roll_, &registered_variables_);

  // Resize the STD containers:
  actual_local_cops_.resize(eigen_sensor_msg_.contacts.size());
  desired_local_cops_.resize(eigen_sensor_msg_.contacts.size());
  actual_feet_positions_.resize(eigen_sensor_msg_.contacts.size());
  desired_feet_positions_.resize(eigen_sensor_msg_.contacts.size());
  actual_contact_forces_.resize(eigen_sensor_msg_.contacts.size());
  desired_contact_forces_.resize(eigen_sensor_msg_.contacts.size());
  actual_contact_torques_.resize(eigen_sensor_msg_.contacts.size());
  desired_contact_torques_.resize(eigen_sensor_msg_.contacts.size());
  for (size_t i = 0; i < eigen_sensor_msg_.contacts.size(); ++i) {
    REGISTER_VARIABLE("/introspection_data",
                      "actual_local_cops_" + eigen_sensor_msg_.contacts[i].name,
                      &actual_local_cops_[i], &registered_variables_);
    REGISTER_VARIABLE(
        "/introspection_data",
        "desired_local_cops_" + eigen_sensor_msg_.contacts[i].name,
        &desired_local_cops_[i], &registered_variables_);
    REGISTER_VARIABLE(
        "/introspection_data",
        "actual_feet_positions_" + eigen_sensor_msg_.contacts[i].name,
        &actual_feet_positions_[i], &registered_variables_);
    REGISTER_VARIABLE(
        "/introspection_data",
        "desired_feet_positions_" + eigen_sensor_msg_.contacts[i].name,
        &desired_feet_positions_[i], &registered_variables_);
    REGISTER_VARIABLE(
        "/introspection_data",
        "actual_contact_forces_" + eigen_sensor_msg_.contacts[i].name,
        &actual_contact_forces_[i], &registered_variables_);
    REGISTER_VARIABLE(
        "/introspection_data",
        "desired_contact_forces_" + eigen_sensor_msg_.contacts[i].name,
        &desired_contact_forces_[i], &registered_variables_);
    REGISTER_VARIABLE(
        "/introspection_data",
        "actual_contact_torques_" + eigen_sensor_msg_.contacts[i].name,
        &actual_contact_torques_[i], &registered_variables_);
    REGISTER_VARIABLE(
        "/introspection_data",
        "desired_contact_torques_" + eigen_sensor_msg_.contacts[i].name,
        &desired_contact_torques_[i], &registered_variables_);
  }

  desired_initial_joint_configuration_.setZero(in_moving_joint_names_.size());
  desired_initial_joint_velocity_.setZero(in_moving_joint_names_.size());
  desired_initial_joint_torques_.setZero(in_moving_joint_names_.size());
  desired_feedforward_torque_.setZero(in_moving_joint_names_.size());
  for (std::size_t i = 0; i < in_moving_joint_names_.size(); ++i) {
    REGISTER_VARIABLE(
        "/introspection_data",
        "desired_initial_joint_configuration_" + in_moving_joint_names_[i],
        &desired_initial_joint_configuration_[i], &registered_variables_);
    REGISTER_VARIABLE(
        "/introspection_data",
        "desired_initial_joint_velocity_" + in_moving_joint_names_[i],
        &desired_initial_joint_velocity_[i], &registered_variables_);
    REGISTER_VARIABLE(
        "/introspection_data",
        "desired_initial_joint_torques_" + in_moving_joint_names_[i],
        &desired_initial_joint_torques_[i], &registered_variables_);
    REGISTER_VARIABLE("/introspection_data",
                      "desired_feedforward_torque_" + in_moving_joint_names_[i],
                      &desired_feedforward_torque_[i], &registered_variables_);
  }
}

bool LinearFeedbackController::readEstimator()
{
  if (true) // @TRUE
  {
    // convert from eigen to std::vector<double>
    std_joint_position_compensated_ = std::vector<double>(
        ei_joint_position_compensated_.data(),
        ei_joint_position_compensated_.data() + ei_joint_position_compensated_.size());
    std_joint_velocity_compensated_ = std::vector<double>(
        ei_joint_velocity_compensated_.data(),
        ei_joint_velocity_compensated_.data() + ei_joint_velocity_compensated_.size());
  }
  else
  {
    std_joint_position_compensated_ = getActualJointPositions();
    std_joint_velocity_compensated_ = getActualJointVelocities();
  }

  return updateEstimator(std_joint_position_compensated_, std_joint_velocity_compensated_);
}


}  // namespace linear_feedback_controller
