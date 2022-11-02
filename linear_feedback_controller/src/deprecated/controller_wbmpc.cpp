/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <memmo_trajectory_controller/memmo_subscriber_controller_sanding_lpf.h>
#include <pal_ros_utils/conversions.h>

namespace memmo
{

bool SubscriberControllerSandingLPF::loadEtras(ros::NodeHandle& control_nh)
{   
	// Create pinocchio model
	std::string urdf_filename = "/opt/openrobots/share/example-robot-data/robots/talos_data/robots/talos_reduced.urdf";
	std::string srdf_filename = "/opt/openrobots/share/example-robot-data/robots/talos_data/srdf/talos.srdf";
	pinocchio::Model pin_model_complete;
    pinocchio::urdf::buildModel(urdf_filename,pin_model_complete);
    std::vector<long unsigned int> controlled_joints_id;
    std::vector<long unsigned int> locked_joints_id;
	// Respect the joint order of the urdf: leg_left / leg_right / torso / arm_left / arm_right
	controlled_joints_names_.push_back("torso_1_joint");
	controlled_joints_names_.push_back("torso_2_joint");
	controlled_joints_names_.push_back("arm_right_1_joint");
	controlled_joints_names_.push_back("arm_right_2_joint");
	controlled_joints_names_.push_back("arm_right_3_joint");
	controlled_joints_names_.push_back("arm_right_4_joint");
	
	// Compute controlled joints id
	for(std::vector<std::string>::const_iterator it = controlled_joints_names_.begin();it != controlled_joints_names_.end(); ++it)
	{
		const std::string & joint_name = *it;
		ROS_INFO_STREAM(joint_name);
		ROS_INFO_STREAM(pin_model_complete.getJointId(joint_name));
		if(pin_model_complete.existJointName(joint_name))
		{   // do not consider joint that are not in the model
			controlled_joints_id.push_back(pin_model_complete.getJointId(joint_name));
		}
		else
		{
			std::cout << "joint: " << joint_name << " does not belong to the model" << std::endl;
		}
	}
	for(std::vector<std::string>::const_iterator it = pin_model_complete.names.begin() + 1;it != pin_model_complete.names.end(); ++it)
	{
		const std::string & joint_name = *it;
		if(std::find(controlled_joints_names_.begin(), controlled_joints_names_.end(), joint_name) == controlled_joints_names_.end())
		{
			locked_joints_id.push_back(pin_model_complete.getJointId(joint_name));
		}
	}
	// Reduce model and set initial position
	pinocchio::srdf::loadReferenceConfigurations(pin_model_complete, srdf_filename, false);
	Eigen::VectorXd q_default_complete = pin_model_complete.referenceConfigurations["half_sitting"];

	pin_model_ = pinocchio::buildReducedModel(pin_model_complete,locked_joints_id,q_default_complete);
	
	for(size_t i = 0; i < controlled_joints_names_.size(); i++)
    {
        auto it = std::find(getControlledJointNames().begin(),getControlledJointNames().end(), controlled_joints_names_[i]);
        if(it == getControlledJointNames().end())
        {
            return false;
        }
        size_t it_dist = std::distance(getControlledJointNames().begin(), it);
        actual_state_map_.emplace(std::make_pair(controlled_joints_names_[i], it_dist));
    }
	
    ros::TransportHints hints;
    hints.tcpNoDelay(true);
    joint_states_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(
                         control_nh, "actual_js_state", 1));
    base_state_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(
                         control_nh, "actual_base_state", 1));

    state_sub_ = control_nh.subscribe("desired_state", 1, &SubscriberControllerSandingLPF::controlCb, this, hints);
    actual_js_state_.name = getControlledJointNames();
    
    ros::Time start_time = ros::Time::now();
    size_t samples = 0;
    desired_pos_ = Eigen::VectorXd::Zero(pin_model_.nq); 
    desired_vel_ = Eigen::VectorXd::Zero(pin_model_.nv); 
    desired_tau_ = Eigen::VectorXd::Zero(pin_model_.nv); 
    actual_pos_ = Eigen::VectorXd::Zero(pin_model_.nq); 
    actual_vel_ = Eigen::VectorXd::Zero(pin_model_.nv); 
    actual_tau_ = Eigen::VectorXd::Zero(pin_model_.nv); 
    diff_state_ = Eigen::VectorXd::Zero(pin_model_.nq * 3);  
    riccati_gains_ = Eigen::MatrixXd::Zero(controlled_joints_names_.size(), pin_model_.nv * 3);
    std::vector<double> a_initial_torque;
    std::vector<double> a_initial_position;
    // std::vector<double> torque_offsets;
    a_initial_torque.resize(actual_js_state_.name.size());
    a_initial_position.resize(actual_js_state_.name.size());
    torque_offsets_.resize(getControlledJointNames().size());

    for(size_t i = 0; i < getControlledJointNames().size(); i++)
    {   
        control_nh.getParam("joints/" + getControlledJointNames()[i] + "/actuator_params/torque_sensor_offset", torque_offsets_[i]);
    }

    while((ros::Time::now() - start_time) < ros::Duration(1.0))
    {
      for(size_t i = 0; i < actual_js_state_.name.size(); i++)
      {
          a_initial_torque[i] += getJointMeasuredTorque(i) - torque_offsets_[i];
          a_initial_position[i] += getActualJointPosition(i);
      }
      samples++;
      ros::Duration(0.01).sleep();
    }

    for(size_t i = 0; i < actual_js_state_.name.size(); i++)
    {
        initial_torque_.push_back(a_initial_torque[i] / double(samples));
        initial_position_.push_back(a_initial_position[i] / double(samples));
        desired_torque_.push_back(initial_torque_[i]);
        actual_torque_.push_back(initial_torque_[i]);
    }
    nb_measure_ = 20;
    for(size_t i = 0; i < nb_measure_; i++) {
		v_measures_.push_front(getActualJointVelocities());
        tau_measures_.push_front(actual_torque_);
	}
	actual_js_state_.velocity = getActualJointVelocities();
    actual_js_state_.effort = actual_torque_;

    tp_.reset(new pal_robot_tools::TimeProfiler);
    tp_->registerTimer("subscriber");
    tp_->registerTimer("mutex_subscriber");
    REGISTER_VARIABLE("/introspection_data", "subscriber_time",
                      tp_->getLastCycleTimePtr("subscriber"), &registered_variables_);
    REGISTER_VARIABLE("/introspection_data", "subscriber_periodicity",
                      tp_->getPeriodicityPtr("subscriber"), &registered_variables_);
    REGISTER_VARIABLE("/introspection_data", "mutex_subscriber_time",
                      tp_->getLastCycleTimePtr("mutex_subscriber"), &registered_variables_);

    ms_mutex_ = getControllerDt().toSec() * 1000;
    ROS_INFO_STREAM("Mutex waiting time " << ms_mutex_);

    p_arm_gain_ = 100.0; // 500.0
    d_arm_gain_ = 8.0; // 20.0
    p_torso_gain_ = 500.0;
    d_torso_gain_ = 20.0;
    p_leg_gain_ = 800.0; // 100.0; // 800.0
    d_leg_gain_ = 35.0; // 8.0; //50.0

    dd_reconfigure_.reset(new ddynamic_reconfigure::DDynamicReconfigure(control_nh));
    dd_reconfigure_->RegisterVariable(&p_arm_gain_, "p_arm");
    dd_reconfigure_->RegisterVariable(&d_arm_gain_, "d_arm");
    dd_reconfigure_->RegisterVariable(&p_torso_gain_, "p_torso");
    dd_reconfigure_->RegisterVariable(&d_torso_gain_, "d_torso");
    dd_reconfigure_->RegisterVariable(&p_leg_gain_, "p_leg");
    dd_reconfigure_->RegisterVariable(&d_leg_gain_, "d_leg");
    dd_reconfigure_->PublishServicesTopics();
    return true;
}

void SubscriberControllerSandingLPF::updateExtras(const ros::Time& time, const ros::Duration& period)
{   
    // Get measured position
	actual_js_state_.position = getActualJointPositions();
    
    // Filter measured velocities
    v_measures_.pop_back();
    v_measures_.push_front(getActualJointVelocities());
    for (size_t i = 0; i < actual_js_state_.velocity.size(); i++) {
		double vel_average = 0;
		for (std::vector<double> vel : v_measures_) {
			vel_average += vel[i];
		}
		actual_js_state_.velocity[i] = vel_average / double(nb_measure_);
	}

    // Filter measured torques
    for(size_t i = 0; i < actual_js_state_.name.size(); i++)
    {
        actual_torque_[i] = getJointMeasuredTorque(i) - torque_offsets_[i];
    }
    tau_measures_.pop_back();
    tau_measures_.push_front(actual_torque_);
    for (size_t i = 0; i < actual_js_state_.effort.size(); i++) {
		double torque_average = 0;
		for (std::vector<double> tau : tau_measures_) {
			torque_average += tau[i];
		}
		actual_js_state_.effort[i] = torque_average / double(nb_measure_);
	}

    // actual_js_state_.effort = actual_torque_; // When not filtered

    base_linear_vel_ = getEstimatedFloatingBaseLinearVelocity();
    base_angular_vel_ = getEstimatedFloatingBaseAngularVelocity();
    base_orientation_ = getEstimatedFloatingBaseOrientation();
    base_position_ = getEstimatedFloatingBasePosition();

    pal::convert(base_linear_vel_, actual_base_state_.twist.twist.linear);
    pal::convert(base_angular_vel_, actual_base_state_.twist.twist.angular);
    pal::convert(base_orientation_, actual_base_state_.pose.pose.orientation);
    pal::convert(base_position_, actual_base_state_.pose.pose.position);

    if (joint_states_pub_->trylock())
    {
        actual_js_state_.header.stamp = time;
        joint_states_pub_->msg_ = actual_js_state_;
        joint_states_pub_->unlockAndPublish();
    }
    if(base_state_pub_->trylock())
    {
        actual_base_state_.header.stamp = time;
        base_state_pub_->msg_ = actual_base_state_;
        base_state_pub_->unlockAndPublish();
	//ROS_INFO_STREAM("Publish base state");
    }
    std::unique_lock<std::timed_mutex> lock(mutex_, std::try_to_lock);
    if(lock.owns_lock() && (!desired_state_.name.empty()))
    {   
		for (size_t i = 0; i < desired_state_.name.size(); i++) {
			desired_pos_[i] = desired_state_.position[i];
			desired_vel_[i] = desired_state_.velocity[i];
            desired_tau_[i] = desired_state_.effort[i];
			actual_pos_[i] = actual_js_state_.position[actual_state_map_[controlled_joints_names_[i]]];
			actual_vel_[i] = actual_js_state_.velocity[actual_state_map_[controlled_joints_names_[i]]];
            actual_tau_[i] = actual_js_state_.effort[actual_state_map_[controlled_joints_names_[i]]];
		}
		pinocchio::difference(pin_model_,actual_pos_,desired_pos_,diff_state_.head(pin_model_.nq));
		diff_state_.segment(pin_model_.nv, pin_model_.nv) = desired_vel_ - actual_vel_; 
        diff_state_.tail(pin_model_.nv) = desired_tau_ - actual_tau_; 
	    for(size_t i = 0; i < desired_state_.name.size(); i++)
        {
            //ROS_INFO_STREAM("desired effort without Ricatti of " << desired_state_.name[i] << " is " << desired_state_.effort[i] );
            //ROS_INFO_STREAM("Ricatti correction is " << riccati_gains_.row(i-1) * diff_state_ ); + riccati_gains_.row(i-1) * diff_state_
            setDesiredJointState(desired_state_.name[i],desired_state_.position[i],desired_state_.velocity[i],0.,desired_state_.effort_interp[i] + riccati_gains_.row(i) * diff_state_ );
        }
    }
    else if(desired_state_.name.empty())
    {   
        for(size_t i = 0; i < actual_js_state_.name.size(); i++)
        {
            if(actual_js_state_.name[i].find("torso") != std::string::npos)
                desired_torque_[i] = initial_torque_[i] - p_torso_gain_ * (getActualJointPosition(i) - initial_position_[i]) - d_torso_gain_ * getActualJointVelocity(i);
            else if(actual_js_state_.name[i].find("arm") != std::string::npos) 
			    desired_torque_[i] = initial_torque_[i] - p_arm_gain_ * (getActualJointPosition(i) - initial_position_[i]) - d_arm_gain_ * getActualJointVelocity(i);
            else
                desired_torque_[i] = initial_torque_[i] - p_leg_gain_ * (getActualJointPosition(i) - initial_position_[i]) - d_leg_gain_ * getActualJointVelocity(i);

            setDesiredJointState(actual_js_state_.name[i],initial_position_[i], 0., 0., desired_torque_[i]);
        }
    }
    else
    {
        ROS_WARN("Skipping received state");
    }
}


void SubscriberControllerSandingLPF::startingExtras(const ros::Time& time)
{
}

void SubscriberControllerSandingLPF::stoppingExtra(const ros::Time& time)
{
    for (size_t i = 0; i < getControlledJointNames().size(); ++i)
    {
        //setDesiredJointTorque(i, 0.);
        setDesiredJointState(i,0.,0.,0.,0.);
    }
}

void SubscriberControllerSandingLPF::controlCb(const memmo_trajectory_controller::JointStateLPF &desired_state)
{
    //ROS_INFO_STREAM("Received desired torque from publisher");
	tp_->startTimer("subscriber");
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    tp_->startTimer("mutex_subscriber");
    if(lock.try_lock_for(std::chrono::milliseconds(ms_mutex_)))
    {
        desired_state_ = desired_state;
    }
    tp_->stopTime("mutex_subscriber");
    tp_->stopTime("subscriber");
    for(size_t i = 0; i < desired_state_.riccati.size(); ++i) {
		for (size_t j = 0; j < desired_state_.riccati[i].data.size(); ++j) {
		    riccati_gains_(i,j) = desired_state_.riccati[i].data[j];
		}
	}
}

}
