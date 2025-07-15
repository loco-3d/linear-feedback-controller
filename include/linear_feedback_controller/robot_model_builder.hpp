#ifndef LINEAR_FEEDBACK_CONTROLLER_ROBOT_MODEL_BUILDER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_ROBOT_MODEL_BUILDER_HPP

#include <string>
#include <vector>

#include "linear_feedback_controller/visibility.hpp"
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"

// Rigid body dynamics
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

namespace linear_feedback_controller {

class LINEAR_FEEDBACK_CONTROLLER_PUBLIC RobotModelBuilder {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  RobotModelBuilder();

  virtual ~RobotModelBuilder();

  /**
   * @brief Build the robot model freezing the joints that are not moving.
   *
   * @param urdf Content of the urdf file.
   * @param srdf Content of the srdf file.
   * @param moving_joint_names List of active joint name.
   * @param controlled_joint_names List of actively controlled joints.
   * @param robot_has_free_flyer If the robot has a free flyer or not.
   */
  virtual bool build_model(const std::string& urdf,
                   const std::vector<std::string>& moving_joint_names,
                   const std::vector<std::string>& controlled_joint_names,
                   const bool robot_has_free_flyer);

  /**
   * @brief Get the moving joint names.
   *
   * @return const std::vector<std::string>&
   */
  virtual const std::vector<std::string>& get_moving_joint_names() const;

  /**
   * @brief Get the moving joint ids.
   *
   * @return const std::vector<pinocchio::Index>&
   */
  virtual const std::vector<pinocchio::Index>& get_moving_joint_ids() const;

  /**
   * @brief Get the locked joint ids.
   *
   * @return const std::vector<pinocchio::Index>&
   */
  virtual const std::vector<pinocchio::Index>& get_locked_joint_ids() const;

  /**
   * @brief Get the pinocchio model.
   *
   * @return const pinocchio::Model&
   */
  virtual const pinocchio::Model& get_model() const;

  /**
   * @brief Get the Pinocchio Data.
   *
   * @return pinocchio::Data&
   */
  virtual pinocchio::Data& get_data();

  /**
   * @brief Get if the robot model has a free flyer joint.
   *
   * @return true
   * @return false
   */
  virtual bool get_robot_has_free_flyer() const;

  /**
   * @brief Get the Pinocchio To Hardware Interface Map.
   *
   * @return std::map<int, int>
   */
  virtual const std::map<int, int>& get_pinocchio_to_hardware_interface_map() const;

  virtual void construct_robot_state(
      const linear_feedback_controller_msgs::Eigen::Sensor& Sensor,
      Eigen::VectorXd& robot_configuration, Eigen::VectorXd& robot_velocity);

  virtual int get_joint_nv() const;
  virtual int get_joint_nq() const;
  virtual int get_nv() const;
  virtual int get_nq() const;

 private:
  /// @brief List of names that correspond to the joints moving by the MPC.
  std::vector<std::string> moving_joint_names_;
  /// @brief Are we using a robot that has a free-flyer?
  bool robot_has_free_flyer_;
  /// @brief The ordered list of the joint names that are actually controlled
  std::vector<std::string> controlled_joint_names_;

  /// @brief Mapping from the pinocchio indexing to the hardware interface.
  std::map<int, int> pin_to_hwi_;
  /// @brief Moving joint ids sorted in the urdf order.
  std::vector<pinocchio::JointIndex> moving_joint_ids_;
  /// @brief Sort the locked (position moving) joint names using the urdf order.
  std::vector<pinocchio::JointIndex> locked_joint_ids_;

  /// @brief Pinocchio (Rigid body dynamics robot model) removing locked joints.
  pinocchio::Model pinocchio_model_;

  /// @brief Computation cache.
  pinocchio::Data pinocchio_data_;

  /// @brief Initial whole body configuration setup in the SRDF file.
  Eigen::VectorXd q_default_complete_;

  /// @brief This is the number of DoF of a free-flyer joint.
  static const Eigen::Index free_flyer_nq_;

  /// @brief This is the number of DoF in the tangent space of a free-flyer
  ///        joint.
  static const Eigen::Index free_flyer_nv_;

 private:
  /**
   * @brief Parse the joint moving names given by the user and build the
   * rigid body models accordingly.
   *
   * @param moving_joint_names
   * @param moving_joint_names
   * @param moving_joint_ids
   * @param locked_joint_ids
   */
  LINEAR_FEEDBACK_CONTROLLER_PRIVATE bool parse_moving_joint_names(
      const pinocchio::Model& pinocchio_model_complete,
      const std::vector<std::string>& moving_joint_names,
      const std::vector<std::string>& controlled_joint_names);

 public:
  using SharedPtr = std::shared_ptr<RobotModelBuilder>;
  using ConstSharedPtr = std::shared_ptr<const RobotModelBuilder>;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_ROBOT_MODEL_BUILDER_HPP
