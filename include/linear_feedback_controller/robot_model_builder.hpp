#ifndef LINEAR_FEEDBACK_CONTROLLER_ROBOT_MODEL_BUILDER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_ROBOT_MODEL_BUILDER_HPP

#include <vector>
#include <string>

// Rigid body dynamics
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include "pinocchio/multibody/data.hpp"

namespace linear_feedback_controller {

class RobotModelBuilder {

public:

RobotModelBuilder();

virtual ~RobotModelBuilder();

/**
 * @brief
 *
 * @param in_urdf Content of the urdf file.
 * @param in_srdf Content of the srdf file.
 * @param in_moving_joint_names List of active joint name.
 * @param in_robot_has_free_flyer If the robot has a free flyer or not.
 */
bool build_model(
	const std::string& in_urdf,
	const std::string& in_srdf,
	const std::vector<std::string>& in_moving_joint_names,
	bool in_robot_has_free_flyer,
	const std::vector<std::string>& in_controlled_joint_names
	);

/**
 * @brief Parse the joint moving names given by the user and build the
 * rigid body models accordingly.
 *
 * @param in_moving_joint_names
 * @param moving_joint_names
 * @param moving_joint_ids
 * @param locked_joint_ids
 */
bool parseMovingJointNames(
	const std::vector<std::string>& in_moving_joint_names,
    const std::vector<std::string>& in_controlled_joint_names,
	std::vector<std::string>& moving_joint_names,
	std::vector<pinocchio::Index>& moving_joint_ids,
	std::vector<pinocchio::Index>& locked_joint_ids);

/**
 * @brief Get the moving joint names.
 *
 * @return const std::vector<std::string>&
 */
const std::vector<std::string>& getMovingJointNames() const;

/**
 * @brief Get the moving joint ids.
 *
 * @return const std::vector<pinocchio::Index>&
 */
const std::vector<pinocchio::Index>& getMovingJointIds() const;

/**
 * @brief Get the locked joint ids.
 *
 * @return const std::vector<pinocchio::Index>&
 */
const std::vector<pinocchio::Index>& getLockedJointIds() const;

/**
 * @brief Get the URDF string.
 *
 * @return const std::string&
 */
const std::string& getUrdf() const;

/**
 * @brief Get the pinocchio model.
 *
 * @return const pinocchio::Model&
 */
const pinocchio::Model& getPinocchioModel() const;

/**
 * @brief Get the Pinocchio Data.
 *
 * @return pinocchio::Data&
 */
pinocchio::Data& getPinocchioData();

private:

/// @brief String containing the model of the robot in xml/urdf format.
std::string in_urdf_;
/// @brief String containing the extras of the model of the robot.
std::string in_srdf_;
/// @brief List of names that correspond to the joints moving by the MPC.
std::vector<std::string> in_moving_joint_names_;
/// @brief Are we using a robot that has a free-flyer?
bool in_robot_has_free_flyer_;
/// @brief The ordered list of the joint names that are actually controlled
std::vector<std::string> in_controlled_joint_names_;

/// @brief Mapping from the pinocchio indexing to the hardware interface.
std::map<int, int> pin_to_hwi_;
/// @brief Moving joint ids sorted in the urdf order.
std::vector<pinocchio::JointIndex> moving_joint_ids_;
/// @brief Sort the moving joint names using the urdf order.
std::vector<std::string> moving_joint_names_;
/// @brief Sort the locked (position moving) joint names using the urdf order.
std::vector<pinocchio::JointIndex> locked_joint_ids_;

/// @brief Pinocchio (Rigid body dynamics robot model).
pinocchio::Model pinocchio_model_complete_;

/// @brief Pinocchio (Rigid body dynamics robot model) removing locked joints.
pinocchio::Model pinocchio_model_reduced_;

/// @brief Computation cache.
pinocchio::Data pinocchio_data_reduced_;

/// @brief Initial whole body configuration setup in the SRDF file.
Eigen::VectorXd q_default_complete_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_ROBOT_MODEL_BUILDER_HPP
