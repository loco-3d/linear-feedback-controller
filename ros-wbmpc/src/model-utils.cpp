#include <limits>
#include <memmo_trajectory_controller/model_utils.h>
#include <eigen_conversions/eigen_msg.h>

namespace model_utils {


FreeFlyerModelUtils::FreeFlyerModelUtils(const std::vector<std::string> controlled_joints_names,
                       const std::string urdf_filename,
                       const std::string srdf_filename)
  : controlled_joints_names_(controlled_joints_names), urdf_filename_(urdf_filename), 
    srdf_filename_(srdf_filename) 
{}

FreeFlyerModelUtils::FreeFlyerModelUtils() {}

pinocchio::Model FreeFlyerModelUtils::createPinocchioModel()
{
	pinocchio::Model pin_model_complete;
	std::vector<long unsigned int> locked_joints_id;
	if (controlled_joints_names_[0] != "root_joint")
	{
		throw_pretty("Joint 0 must be a root joint!");
	}
  if (false)
  {
    pinocchio::urdf::buildModelFromXML(robot_description_, pinocchio::JointModelFreeFlyer(),
                                       pin_model_complete);
    std::cout<<"### Build pinocchio model from rosparam robot_description."<<std::endl;
  }
  else
  {
    pinocchio::urdf::buildModel(urdf_filename_, pinocchio::JointModelFreeFlyer(),
                                pin_model_complete);
    std::cout<<"### Build pinocchio model from urdf file."<<std::endl;
  }

  // Check if listed joints belong to model
	for(std::vector<std::string>::const_iterator it = controlled_joints_names_.begin();it != controlled_joints_names_.end(); ++it)
	{
		const std::string & joint_name = *it;
		std::cout << joint_name << std::endl;
		std::cout << pin_model_complete.getJointId(joint_name) << std::endl;
		if(not(pin_model_complete.existJointName(joint_name)))
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
	pinocchio::srdf::loadReferenceConfigurations(pin_model_complete,srdf_filename_, false);
	pinocchio::srdf::loadRotorParameters(pin_model_complete, srdf_filename_, false);
	Eigen::VectorXd q_default_complete = pin_model_complete.referenceConfigurations["half_sitting"];
	pin_model_ = pinocchio::buildReducedModel(pin_model_complete,locked_joints_id,q_default_complete);

	// Make list of controlled joints for reduced model
	for(std::vector<std::string>::const_iterator it = pin_model_.names.begin()+1;it != pin_model_.names.end(); ++it)
	{
		const std::string & joint_name = *it;
		if(std::find(controlled_joints_names_.begin(), controlled_joints_names_.end(), joint_name) != controlled_joints_names_.end())
		{
			controlled_joints_id_.push_back((long)pin_model_.getJointId(joint_name) - 2);
		}
	}
	x_current_.resize(pin_model_.nq + pin_model_.nv);
	joints_state_.name = controlled_joints_names_;
	joints_state_.position.resize(controlled_joints_names_.size());
	joints_state_.base_position.resize(3);
	joints_state_.base_orientation.resize(4);
	joints_state_.base_velocity_linear.resize(3);
	joints_state_.base_velocity_angular.resize(3);
	joints_state_.velocity.resize(controlled_joints_names_.size());
  joints_state_.acceleration.resize(controlled_joints_names_.size());
	joints_state_.effort.resize(controlled_joints_names_.size());
  joints_state_.acceleration.resize(controlled_joints_names_.size());
	joints_state_.riccati.resize(controlled_joints_names_.size()-1);
	for (size_t i = 0; i < controlled_joints_names_.size() -1; i++) {
		joints_state_.riccati[i].data.resize(pin_model_.nv * 2);
	}
	joints_state_.dKp0.resize(controlled_joints_names_.size()-1);
	for (size_t i = 0; i < controlled_joints_names_.size() -1; i++) {
		joints_state_.dKp0[i].data.resize(3);
	}
	joints_state_.mocap_target_pos.resize(3);
  joints_state_.contact_left = true;
  joints_state_.contact_right = true;
  joints_state_.contact_force_left.resize(3);
  joints_state_.contact_torque_left.resize(3);
  joints_state_.contact_force_right.resize(3);
  joints_state_.contact_torque_right.resize(3);
	return pin_model_;
	
}

void FreeFlyerModelUtils::buildStateMap(sensor_msgs::JointStateConstPtr actual_joint_state)
{
	for(size_t i = 1; i < controlled_joints_names_.size(); i++)
	{
		auto it = std::find(actual_joint_state->name.begin(), actual_joint_state->name.end(), controlled_joints_names_[i]);
		if(it == actual_joint_state->name.end())
		{
			throw_pretty("Controlled joint " << controlled_joints_names_[i] << " not detected in joint states");
		}
		size_t it_dist = (size_t)std::distance(actual_joint_state->name.begin(), it);
		actual_state_map_.emplace(std::make_pair(controlled_joints_names_[i], it_dist));
	}
}

void FreeFlyerModelUtils::mapToPinocchio(sensor_msgs::JointStateConstPtr actual_joint_state, nav_msgs::OdometryConstPtr actual_base_state)
{
	// Remap position and velocity to fit pinocchio model
	for (size_t i = 1; i < controlled_joints_names_ .size(); i++)
	{
		x_current_[controlled_joints_id_[i] + 7] = actual_joint_state->position[actual_state_map_[controlled_joints_names_[i]]];
		x_current_[controlled_joints_id_[i] + pin_model_.nq + 6] = actual_joint_state->velocity[actual_state_map_[controlled_joints_names_[i]]];
	}
	tf::vectorMsgToEigen(actual_base_state->twist.twist.linear, base_linear_vel_);
	tf::vectorMsgToEigen(actual_base_state->twist.twist.angular, base_angular_vel_);
	tf::quaternionMsgToEigen(actual_base_state->pose.pose.orientation, base_orientation_);
	tf::pointMsgToEigen(actual_base_state->pose.pose.position, base_position_);

	base_orientation_.normalize();
	for (int i = 0; i < 3; i++)
	{
		x_current_[i] = base_position_[i];
		x_current_[i + pin_model_.nq] = base_linear_vel_[i];
		x_current_[i + pin_model_.nq + 3] = base_angular_vel_[i];
	}
	x_current_[3] = base_orientation_.x();
	x_current_[4] = base_orientation_.y();
	x_current_[5] = base_orientation_.z();
	x_current_[6] = base_orientation_.w();
}

void FreeFlyerModelUtils::mapToTF(const Eigen::VectorXd us0, const Eigen::MatrixXd K0, const Eigen::VectorXd ddq)
{
	for (long i = 0; i < K0.rows(); i++)
	{
		for (long j = 0; j < K0.cols(); j++) 
		{
			joints_state_.riccati[(size_t)i].data[(size_t)j] = K0(i,j);
		}
	}
	for (size_t i = 1; i < controlled_joints_names_.size(); i++)
	{
		joints_state_.effort[i] = us0[controlled_joints_id_[i]];
		joints_state_.position[i] = x_current_[controlled_joints_id_[i] + 7];
		joints_state_.velocity[i] = x_current_[controlled_joints_id_[i] + 6 + pin_model_.nq];
    if(ddq.size() > 0)
      joints_state_.acceleration[i] = ddq[controlled_joints_id_[i] + 6];
	}
	joints_state_.base_position = std::vector<double>(&base_position_[0],base_position_.data() + base_position_.size());
	joints_state_.base_orientation[0] = base_orientation_.x();
	joints_state_.base_orientation[1] = base_orientation_.y();
	joints_state_.base_orientation[2] = base_orientation_.z();
	joints_state_.base_orientation[3] = base_orientation_.w();
	joints_state_.base_velocity_linear = std::vector<double>(&base_linear_vel_[0],base_linear_vel_.data() + base_linear_vel_.size());
	joints_state_.base_velocity_angular = std::vector<double>(&base_angular_vel_[0],base_angular_vel_.data() + base_angular_vel_.size());
}

void FreeFlyerModelUtils::mapToTF(const Eigen::VectorXd us0,const Eigen::MatrixXd K0,const Eigen::MatrixXd dKp,const Eigen::VectorXd mocap_target_pos)
{
	for (long i = 0; i < K0.rows(); i++)
	{
		for (long j = 0; j < K0.cols(); j++) 
		{
			joints_state_.riccati[(size_t)i].data[(size_t)j] = K0(i,j);
		}
	}
	for (size_t i = 1; i < controlled_joints_names_.size(); i++)
	{
		joints_state_.effort[i] = us0[controlled_joints_id_[i]];
		joints_state_.position[i] = x_current_[controlled_joints_id_[i] + 7];
		joints_state_.velocity[i] = x_current_[controlled_joints_id_[i] + 6 + pin_model_.nq];
	}
	joints_state_.base_position = std::vector<double>(&base_position_[0],base_position_.data() + base_position_.size());
	joints_state_.base_orientation[0] = base_orientation_.x();
	joints_state_.base_orientation[1] = base_orientation_.y();
	joints_state_.base_orientation[2] = base_orientation_.z();
	joints_state_.base_orientation[3] = base_orientation_.w();
	joints_state_.base_velocity_linear = std::vector<double>(&base_linear_vel_[0],base_linear_vel_.data() + base_linear_vel_.size());
	joints_state_.base_velocity_angular = std::vector<double>(&base_angular_vel_[0],base_angular_vel_.data() + base_angular_vel_.size());
	
	joints_state_.mocap_target_pos = std::vector<double>(&mocap_target_pos[0],mocap_target_pos.data() + mocap_target_pos.size()); 
	for (long i = 0; i < dKp.rows(); i++)
	{
		for (long j = 0; j < dKp.cols(); j++)
		{
			joints_state_.dKp0[(size_t)i].data[(size_t)j] = dKp(i,j);
		}
	} 
}

}

