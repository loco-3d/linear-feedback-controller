/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <algorithm>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "linear_feedback_controller/linear_feedback_controller.hpp"

namespace linear_feedback_controller
{

LinearFeedbackController::LinearFeedbackController()
  : pal_base_ros_controller::BaseRobotWithEsimatorController()
{
}

LinearFeedbackController::~LinearFeedbackController()
{
}

bool LinearFeedbackController::loadEtras(ros::NodeHandle& node_handle)
{
  // Parse the parameters.
  if (!parseRosParams(node_handle))
  {
    return false;
  }

  // Build the rigid body model of the robot.
  if (in_robot_has_free_flyer_)
  {
    pinocchio::urdf::buildModelFromXML(in_urdf_, pinocchio::JointModelFreeFlyer(),
                                       pinocchio_model_complete_);
  }
  else
  {
    pinocchio::urdf::buildModelFromXML(in_urdf_, pinocchio_model_complete_);
  }
  std::istringstream iss_srdf(in_srdf_);
  pinocchio::srdf::loadReferenceConfigurationsFromXML(pinocchio_model_complete_, iss_srdf, false);

  // Reduce the rigid body model and set initial position.
  if (!parseMovingJointNames(in_moving_joint_names_, moving_joint_names_,
                             moving_joint_ids_, locked_joint_ids_))
  {
    return false;
  }
  q_default_complete_ = pinocchio_model_complete_.referenceConfigurations["half_sitting"];
  pinocchio_model_reduced_ = pinocchio::buildReducedModel(
      pinocchio_model_complete_, locked_joint_ids_, q_default_complete_);

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

  // Prepare the publisher and subscriber exchanging the control and state.
  sensor_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<linear_feedback_controller_msgs::Sensor> >(
          node_handle, "sensor_state", 1);
  ros::TransportHints hints;
  hints.tcpNoDelay(true);
  control_subscriber_ = node_handle.subscribe(
      "desired_control", 1, &LinearFeedbackController::controlSubscriberCallback, this, hints);

  // Filter the initial measured joint position and torque and save it.
  filterInitialState();

  // Allocate the memory of the various vector which are joint number dependent.
  ros_sensor_msg_.joint_state.name = moving_joint_names_;
  ros_sensor_msg_.joint_state.position.resize(moving_joint_names_.size(), 0.0);
  ros_sensor_msg_.joint_state.velocity.resize(moving_joint_names_.size(), 0.0);
  ros_sensor_msg_.joint_state.effort.resize(moving_joint_names_.size(), 0.0);
  eigen_sensor_msg_.joint_state.name = moving_joint_names_;
  eigen_sensor_msg_.joint_state.position.resize(moving_joint_names_.size());
  eigen_sensor_msg_.joint_state.position.fill(0.0);
  eigen_sensor_msg_.joint_state.velocity.resize(moving_joint_names_.size());
  eigen_sensor_msg_.joint_state.velocity.fill(0.0);
  eigen_sensor_msg_.joint_state.effort.resize(moving_joint_names_.size());
  eigen_sensor_msg_.joint_state.effort.fill(0.0);
  eigen_sensor_msg_.contacts.resize(2);
  pd_desired_torque_.resize(moving_joint_names_.size());
  pd_desired_torque_.fill(0.0);
  /// @todo automatize this.
  eigen_sensor_msg_.contacts[0].name = "left_sole_link";
  eigen_sensor_msg_.contacts[1].name = "right_sole_link";
  desired_stance_ids_ = std::vector<std::string>();
  desired_swing_ids_ = std::vector<std::string>();
  desired_stance_ids_.push_back("left_sole_link");
  desired_stance_ids_.push_back("right_sole_link");

  /// @todo Automatize the parametrization of these gains.
  p_arm_gain_ = 100.0;  // 500.0
  d_arm_gain_ = 8.0;    // 20.0
  p_torso_gain_ = 500.0;
  d_torso_gain_ = 20.0;
  p_leg_gain_ = 800.0;  // 100.0; // 800.0
  d_leg_gain_ = 35.0;   // 8.0; //50.0
  // dd_reconfigure_.reset(new ddynamic_reconfigure::DDynamicReconfigure(control_nh));
  // dd_reconfigure_->RegisterVariable(&p_arm_gain_, "p_arm");
  // dd_reconfigure_->RegisterVariable(&d_arm_gain_, "d_arm");
  // dd_reconfigure_->RegisterVariable(&p_torso_gain_, "p_torso");
  // dd_reconfigure_->RegisterVariable(&d_torso_gain_, "d_torso");
  // dd_reconfigure_->RegisterVariable(&p_leg_gain_, "p_leg");
  // dd_reconfigure_->RegisterVariable(&d_leg_gain_, "d_leg");
  // dd_reconfigure_->PublishServicesTopics();
  return true;
}

void LinearFeedbackController::updateExtras(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  // Shortcuts for easier code writing.
  const linear_feedback_controller_msgs::Eigen::JointState& sensor_js =
      eigen_sensor_msg_.joint_state;
  const linear_feedback_controller_msgs::Eigen::JointState& ctrl_js =
      eigen_control_msg_.initial_state.joint_state;
  const linear_feedback_controller_msgs::Eigen::Sensor& ctrl_init =
      eigen_control_msg_.initial_state;

  acquireSensorAndPublish();

  // Lock the mutex here to not have conflict with the subscriber callback.
  std::unique_lock<std::timed_mutex> lock(mutex_, std::try_to_lock);

  // If the control message is not empty then it means there is a controller
  // sending messages. Hence we update it. We also test if the lock is properly
  // acquired. It could happen that at this point a new control message is
  // arriving. Hence we skip the update from the message.
  if (lock.owns_lock() && (!ctrl_js.name.empty()))
  {
    // Reconstruct the state vector: x = [q, v]
    if (in_robot_has_free_flyer_)
    {
      desired_configuration_.head<7>() = ctrl_init.base_pose;
      desired_velocity_.head<6>() = ctrl_init.base_twist;
      measured_configuration_.head<7>() = eigen_sensor_msg_.base_pose;
      measured_velocity_.head<6>() = eigen_sensor_msg_.base_twist;
    }
    int nb_joint = ctrl_js.name.size();
    desired_configuration_.tail(nb_joint) = ctrl_js.position;
    desired_velocity_.tail(nb_joint) = ctrl_js.velocity;
    const Eigen::VectorXd desired_torque = ctrl_js.effort;
    measured_configuration_.tail(nb_joint) = sensor_js.position;
    measured_velocity_.tail(nb_joint) = sensor_js.velocity;

    // Compute the difference between the poses.
    pinocchio::difference(pinocchio_model_reduced_, measured_configuration_, desired_configuration_,
                          diff_state_.head(pinocchio_model_reduced_.nv));
    diff_state_.tail(pinocchio_model_reduced_.nv) = desired_velocity_ - measured_velocity_;

    for (std::size_t i = 1; i < ctrl_js.name.size(); ++i)
    {
      /// @todo Figure out why i = 1 in this for loop.
      setDesiredJointState(ctrl_js.name[i], ctrl_js.position[i], ctrl_js.velocity[i],
                           0.0,  // Acceleration.
                           ctrl_js.effort[i] +
                               eigen_control_msg_.feedback_gain.row(i - 1) * diff_state_);
    }

    // Define the support foot
    /// @todo automatize this in the estimator?
    desired_swing_ids_.clear();
    desired_stance_ids_.clear();
    if (ctrl_init.contacts[0].active)
    {
      desired_stance_ids_.push_back("left_sole_link");
    }
    else
    {
      desired_swing_ids_.push_back("left_sole_link");
    }
    if (ctrl_init.contacts[1].active)
    {
      desired_stance_ids_.push_back("right_sole_link");
    }
    else
    {
      desired_swing_ids_.push_back("right_sole_link");
    }
  }
  else if (ctrl_js.name.empty())  // No control has been sent yet.
  {
    
    for (std::size_t i = 0; i < sensor_js.name.size(); i++)
    {
      const int i_int = static_cast<int>(i);
      if (sensor_js.name[i].find("torso") != std::string::npos)
      {
        pd_desired_torque_(i) =
            initial_torque_[i] -
            p_torso_gain_ * (getActualJointPosition(i_int) - initial_position_[i]) -
            d_torso_gain_ * getActualJointVelocity(i_int);
      }
      else if (sensor_js.name[i].find("arm") != std::string::npos)
      {
        pd_desired_torque_(i) =
            initial_torque_[i] -
            p_arm_gain_ * (getActualJointPosition(i_int) - initial_position_[i]) -
            d_arm_gain_ * getActualJointVelocity(i_int);
      }
      else
      {
        pd_desired_torque_(i) =
            initial_torque_[i] -
            p_leg_gain_ * (getActualJointPosition(i_int) - initial_position_[i]) -
            d_leg_gain_ * getActualJointVelocity(i_int);
      }
      setDesiredJointState(sensor_js.name[i],
                           initial_position_[i],  //
                           0.0,                   // Velocity
                           0.0,                   // Acceleration
                           pd_desired_torque_(i));
    }
  }
  else
  {
    ROS_WARN("Skipping received state");
  }
  // define the active contacts used for the base estimation
  setEstimatorContactState(desired_stance_ids_, desired_swing_ids_);
  /// @todo redo the introspection.
  // computeIntrospection();
}

void LinearFeedbackController::startingExtras(const ros::Time& /*time*/)
{
}

void LinearFeedbackController::stoppingExtra(const ros::Time& /*time*/)
{
  // Kill the robot, i.e. send zero torques to all joints.
  for (std::size_t i = 0; i < getControlledJointNames().size(); ++i)
  {
    setDesiredJointState(i, 0., 0., 0., 0.);
  }
}

bool LinearFeedbackController::parseMovingJointNames(
    const std::vector<std::string>& in_moving_joint_names,
    std::vector<std::string>& moving_joint_names, std::vector<long unsigned int>& moving_joint_ids,
    std::vector<long unsigned int>& locked_joint_ids)
{
  // Get moving joints ids
  ROS_INFO_STREAM("Map the given moving joints names with their urdf ids");
  moving_joint_ids.clear();
  for (std::vector<std::string>::const_iterator it = in_moving_joint_names.begin();
       it != in_moving_joint_names.end(); ++it)
  {
    const std::string& joint_name = *it;
    pinocchio::JointIndex joint_id = 0;
    // do not consider joint that are not in the model
    if (pinocchio_model_complete_.existJointName(joint_name))
    {
      joint_id = pinocchio_model_complete_.getJointId(joint_name);
      moving_joint_ids.push_back(joint_id);
      ROS_INFO_STREAM("joint_name=" << joint_name << ", joint_id=" << joint_id);
    }
    else
    {
      ROS_WARN_STREAM("joint_name=" << joint_name << " does not belong to the model");
    }
  }
  // Sort them to the pinocchio order (increasing number) and remove duplicates.
  std::sort(moving_joint_ids.begin(), moving_joint_ids.end());
  moving_joint_ids.erase(unique(moving_joint_ids.begin(), moving_joint_ids.end()),
                         moving_joint_ids.end());

  // Remap the moving joint names to the Pinocchio oder.
  moving_joint_names.clear();
  for (std::size_t i = 0; i < moving_joint_ids.size(); ++i)
  {
    moving_joint_names.push_back(pinocchio_model_complete_.names[moving_joint_ids[i]]);
  }

  // Locked joint ids in the Pinocchio order.
  locked_joint_ids.clear();
  for (std::vector<std::string>::const_iterator it = pinocchio_model_complete_.names.begin() + 1;
       it != pinocchio_model_complete_.names.end(); ++it)
  {
    const std::string& joint_name = *it;
    if (std::find(moving_joint_names.begin(), moving_joint_names.end(), joint_name) ==
        moving_joint_names.end())
    {
      locked_joint_ids.push_back(pinocchio_model_complete_.getJointId(joint_name));
    }
  }

  // Create the joint name joint id mapping of the hardware interface.
  for (std::size_t i = 1; i < moving_joint_names.size(); ++i)
  {
    auto it = std::find(getControlledJointNames().begin(),
                        getControlledJointNames().end(), moving_joint_names[i]);
    // verify that the found moving joints are part of the hardware interface.
    if (it == getControlledJointNames().end())
    {
      ROS_ERROR_STREAM("Moving joint "
                       << moving_joint_names[i]
                       << " is not part of the current roscontrol hardware interface.");
      return false;
    }
    std::size_t it_dist = (size_t)std::distance(getControlledJointNames().begin(), it);
    pin_to_hwi_.emplace(std::make_pair(i, it_dist));
  }
  return true;
}

void LinearFeedbackController::controlSubscriberCallback(
    const linear_feedback_controller_msgs::Control& /*msg*/)
{
}

// Get the parameters of the node return false in case of failure.
#define GET_ROS_PARAM(nh, name, var)                                                     \
  if (!nh.hasParam(name))                                                                \
  {                                                                                      \
    ROS_INFO_STREAM("Missing ROS arg: " << name);                                        \
    return false;                                                                        \
  }                                                                                      \
  nh.getParam(name, var);

bool LinearFeedbackController::parseRosParams(ros::NodeHandle& node_handle)
{
  // Get the parameters of the node return false in case of failure.
  GET_ROS_PARAM(node_handle, "robot_description", in_urdf_);
  GET_ROS_PARAM(node_handle, "robot_description_semantic", in_srdf_);
  GET_ROS_PARAM(node_handle, "moving_joint_names", in_moving_joint_names_);
  GET_ROS_PARAM(node_handle, "robot_has_free_flyer", in_robot_has_free_flyer_);
  size_t nbjoint = getControlledJointNames().size();
  in_torque_offsets_.resize(nbjoint, 0.0);
  for (size_t i = 0; i < nbjoint; ++i)
  {
    std::string param_name =
        "joints/" + getControlledJointNames()[i] + "/actuator_params/torque_sensor_offset";
    GET_ROS_PARAM(node_handle, param_name, in_torque_offsets_[i]);
  }
  ROS_INFO_STREAM("LinearFeedbackController: Loading parameters... Done.");
  return true;
}

void LinearFeedbackController::filterInitialState()
{
  int nb_joint = static_cast<int>(getControlledJointNames().size());
  int nb_samples = 0;
  initial_position_.resize(nb_joint, 0.0);
  initial_torque_.resize(nb_joint, 0.0);
  ros::Time start_time =
      ros::Time::now() - ros::Duration(0, 1000000);  // An epsilon time before now;
  while ((ros::Time::now() - start_time) < ros::Duration(1.0))
  {
    for (int i = 0; i < nb_joint; ++i)
    {
      initial_torque_[i] += initial_position_[i] += getActualJointPosition(i);
    }
    ++nb_samples;
    ros::Duration(0.01).sleep();
  }
  for (int i = 0; i < nb_joint; i++)
  {
    initial_torque_[i] /= static_cast<double>(nb_samples);
    initial_position_[i] /= nb_samples;
  }
}

void LinearFeedbackController::acquireSensorAndPublish()
{
  /// @todo Filter the data here?

  // Fill in the base data.
  eigen_sensor_msg_.base_pose.head<3>() = getEstimatedFloatingBasePosition();
  Eigen::Quaterniond q_base = getEstimatedFloatingBaseOrientation();
  q_base.normalize();
  eigen_sensor_msg_.base_pose.tail<4>() = q_base.coeffs();
  eigen_sensor_msg_.base_twist.head<3>() = getEstimatedFloatingBaseLinearVelocity();
  eigen_sensor_msg_.base_twist.tail<3>() = getEstimatedFloatingBaseAngularVelocity();

  // Fill the joint state data, i.e. the position, velocity and torque.
  const auto& act_jp = getActualJointPositions();
  const auto& act_jv = getActualJointVelocities();
  std::size_t nb_joint = getActualJointPositions().size();
  for (std::size_t i = 0; i < nb_joint; ++i)
  {
    eigen_sensor_msg_.joint_state.position(i) = act_jp[pin_to_hwi_[i]];
    eigen_sensor_msg_.joint_state.velocity(i) = act_jv[pin_to_hwi_[i]];
    eigen_sensor_msg_.joint_state.effort(i) =
        getJointMeasuredTorque(pin_to_hwi_[i]) - in_torque_offsets_[pin_to_hwi_[i]];
  }

  // Fill the information about the force sensor and if the corresponding
  // end-effector are in contact or not.
  const std::vector<pal_controller_interface::FTSensorDefinitionPtr>& ft_sensors =
      getForceTorqueSensorDefinitions();
  assert(ft_sensors.size() >= 2 && "There must be at least 2 force sensor.");
  for (std::size_t i = 0; i < 2; ++i)
  {
    /// @todo verify that this is the good order, left then right...
    eigen_sensor_msg_.contacts[i].wrench.head<3>() = ft_sensors[i]->force;
    eigen_sensor_msg_.contacts[i].wrench.tail<3>() = ft_sensors[i]->torque;
    if (eigen_sensor_msg_.contacts[i].wrench(2) > 10)
    {
      eigen_sensor_msg_.contacts[i].active = true;
    }
    else
    {
      eigen_sensor_msg_.contacts[i].active = false;
    }
  }

  // Publish the message to the ROS topic.
  linear_feedback_controller_msgs::sensorEigenToMsg(eigen_sensor_msg_, ros_sensor_msg_);
  if (sensor_publisher_->trylock())
  {
    sensor_publisher_->msg_ = ros_sensor_msg_;
    sensor_publisher_->unlockAndPublish();
  }
  else
  {
    ROS_ERROR_STREAM("LinearFeedbackController::updateExtras(): "
                     << "Cannot lock the publisher, "
                     << "the sensor message is not sent.");
  }
}

// clang-format off



// void SubscriberControllerSandingLPF::stoppingExtra(const ros::Time& time)
// {
//     for (size_t i = 0; i < getControlledJointNames().size(); ++i)
//     {
//         //setDesiredJointTorque(i, 0.);
//         setDesiredJointState(i,0.,0.,0.,0.);
//     }
// }

// void SubscriberControllerSandingLPF::controlCb(const memmo_trajectory_controller::JointStateLPF &desired_state)
// {
//     //ROS_INFO_STREAM("Received desired torque from publisher");
// 	tp_->startTimer("subscriber");
//     std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
//     tp_->startTimer("mutex_subscriber");
//     if(lock.try_lock_for(std::chrono::milliseconds(ms_mutex_)))
//     {
//         desired_state_ = desired_state;
//     }
//     tp_->stopTime("mutex_subscriber");
//     tp_->stopTime("subscriber");
//     for(size_t i = 0; i < desired_state_.riccati.size(); ++i) {
// 		for (size_t j = 0; j < desired_state_.riccati[i].data.size(); ++j) {
// 		    riccati_gains_(i,j) = desired_state_.riccati[i].data[j];
// 		}
// 	}
// }

// clang-format on
}  // namespace linear_feedback_controller
