#ifndef LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_HPP

// Standard C++
#include <memory>

// Rigid body dynamics
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

// ROS C++ api
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

// ROS Messages.
#include <linear_feedback_controller_msgs/Sensor.h>
#include <linear_feedback_controller_msgs/Control.h>
#include <linear_feedback_controller_msgs/eigen_conversions.hpp>

// PAL roscontrol controller containing their estimator.
#include <pal_base_ros_controller/base_robot_with_estimator_controller.h>

// local include
#include "linear_feedback_controller/min_jerk.hpp"
#include "linear_feedback_controller/averaging_filter.hpp"


namespace linear_feedback_controller
{

/**
 * @brief This class has for purpose to connect Whole Body Model Predictive
 * Controllers from https://github.com/loco-3d/crocoddyl and
 * https://github.com/MeMory-of-MOtion/sobec to the low level controller.
 *
 * This part of the controller interpolate the controls from the Ricatti gains
 * and the feed-forward terms. It runs at the robot low level frequency.
 *
 * In essence it computes the fooling joint torques \f$ \tau \f$:
 *
 * \f[
 *  \tau = K_{feedback} * (x^{des} - x^{act}) + \tau_0
 * \f]
 *
 * With \f$ K_{feedback} \f$ being the feedback gain matrix potentially
 * extracted from the Ricatti gains, \f$ x^{des} \f$ and \f$ x^{act} \f$ being
 * respectively the desired and actual state of the controller, and finally
 * \f$ \tau_0 \f$ the feed-forward term.
 *
 * This class inherits from the pal_base_ros_controller::BaseRobotWithEsimatorController
 * class which is a PAL-ROBOTICS class that pre-instantiate a base estimator.
 * Hence base data which are available in this controller come from this
 * estimator.
 */
class LinearFeedbackController : public pal_base_ros_controller::BaseRobotWithEsimatorController
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct a new LinearFeedbackController.
   */
  LinearFeedbackController();

  /**
   * @brief Destroy the Linear Feedback Controller.
   */
  ~LinearFeedbackController();

  /**
   * @brief Load the controller. Instantiate all memory and parse the ROS params
   *
   * @param node_handle this is the ROS object allowing us to use the middleware
   * ROS. It is responsible for the creation of the "Subscribers" and
   * "Publishers".
   * @return true if everything went fine,
   * @return false otherwise.
   */
  bool loadEtras(ros::NodeHandle& node_handle) override;

  /**
   * @brief Update the current controller.
   *
   * @param time current time.
   * @param period roscontrol period, on Talos 2kHz.
   */
  void updateExtras(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief Starting the controller, meaning starting to compute \f$ \tau \f$.
   *
   * @param time the current time.
   */
  void startingExtras(const ros::Time& time) override;

  /**
   * @brief Stopping the controller, falling back to a default controller.
   *
   * @param time
   */
  void stoppingExtra(const ros::Time& time) override;

private:  // Private methods.
  /**
   * @brief Parse the joint moving names given by the user and build the
   * rigid body models accordingly.
   *
   * @param in_moving_joint_names
   * @param moving_joint_names
   * @param moving_joint_ids
   * @param locked_joint_ids
   */
  bool parseMovingJointNames(const std::vector<std::string>& in_moving_joint_names,
                             std::vector<std::string>& moving_joint_names,
                             std::vector<long unsigned int>& moving_joint_ids,
                             std::vector<long unsigned int>& locked_joint_ids);
  /**
   * @brief Acquire the control from the external controller.
   *
   * @param msg
   */
  void controlSubscriberCallback(const linear_feedback_controller_msgs::Control& msg);

  /**
   * @brief Parse the ROS parameters.
   *
   * @return true
   * @return false
   */
  bool parseRosParams(ros::NodeHandle& node_handle);

  /**
   * @brief Filter the initial state during 1 second in order to start with
   * clean data.
   */
  void filterInitialState();

  /**
   * @brief Acquire the sensor data and fill in the sensor message.
   * The indexing of the ROS message is in the order of the moving_joint_names_.
   * Which means that it is in the order of the pinocchio indexing.
   * We use a map pin_to_hwi_ in order to map the data from the hardware
   * interface indexing to the pinnochio one.
   */
  void acquireSensorAndPublish();

  /** @brief Computes an ordinary pd controller that freezes the robot.
   * it sets the pd_desired_torque_ internal variable.
   */
  void pd_controller();

  /**
   * @brief Computes the linear feedback control from the messages received
   * by the rostopic. It sets the lf_desired_torque_ internal variable.
   */
  void lf_controller();

public:  // Setters and getters
  /**
   * @brief Get the moving joint names.
   *
   * @return const std::vector<std::string>&
   */
  const std::vector<std::string>& getMovingJointNames() const
  {
    return moving_joint_names_;
  }

  /**
   * @brief Get the moving joint ids.
   *
   * @return const std::vector<long unsigned int>&
   */
  const std::vector<long unsigned int>& getMovingJointIds() const
  {
    return moving_joint_ids_;
  }

  /**
   * @brief Get the locked joint ids.
   *
   * @return const std::vector<long unsigned int>&
   */
  const std::vector<long unsigned int>& getLockedJointIds() const
  {
    return locked_joint_ids_;
  }

  /**
   * @brief Get the URDF string.
   *
   * @return const std::string&
   */
  const std::string& getUrdf()
  {
    return in_urdf_;
  }

  /**
   * @brief Get the torque offset.
   *
   * @return const std::vector<long unsigned int>&
   */
  const std::vector<double>& getTorqueOffsets() const
  {
    return in_torque_offsets_;
  }

  /**
   * @brief Get the From PD To LF Duration.
   * 
   * @return const std::vector<double>& 
   */
  const double& getFromPDToLFDuration() const
  {
    return from_pd_to_lf_duration_;
  }

private:  // Members
  /// @brief String containing the model of the robot in xml/urdf format.
  std::string in_urdf_;
  /// @brief String containing the extras of the model of the robot.
  std::string in_srdf_;
  /// @brief List of names that correspond to the joints moving by the MPC.
  std::vector<std::string> in_moving_joint_names_;
  /// @brief Are we using a robot that has a free-flyer?
  bool in_robot_has_free_flyer_;
  /** @brief Duration in second for the switch between the initial pd controller
   * to the linear feedback one. */
  double from_pd_to_lf_duration_;
  /** @brief Joint torque offsets based on the state of the hardware.
   *  They are used in the hardware interface indexing. */
  std::vector<double> in_torque_offsets_;

  /// @brief Pinocchio (Rigid body dynamics robot model).
  pinocchio::Model pinocchio_model_complete_;
  pinocchio::Model pinocchio_model_reduced_;

  /// @brief Moving joint ids sorted in the urdf order.
  std::vector<pinocchio::JointIndex> moving_joint_ids_;
  /// @brief Sort the moving joint names using the urdf order.
  std::vector<std::string> moving_joint_names_;
  /// @brief Sort the locked (position moving) joint names using the urdf order.
  std::vector<pinocchio::JointIndex> locked_joint_ids_;
  /// @brief Mapping from the pinocchio indexing to the hardware interface.
  std::map<int, int> pin_to_hwi_;

  /// @brief Initial whole body configuration setup in the SRDF file.
  Eigen::VectorXd q_default_complete_;

  /// @brief Actual robot state publisher.
  std::shared_ptr<realtime_tools::RealtimePublisher<linear_feedback_controller_msgs::Sensor> > sensor_publisher_;

  /// @brief  ROS sensor message data.
  linear_feedback_controller_msgs::Sensor ros_sensor_msg_;
  linear_feedback_controller_msgs::Eigen::Sensor eigen_sensor_msg_;

  /// @brief Actual robot state publisher.
  ros::Subscriber control_subscriber_;

  /// @brief  ROS sensor message data.
  linear_feedback_controller_msgs::Eigen::Control eigen_control_msg_;

  /// @brief Configuration around which the controller is linearized.
  Eigen::VectorXd desired_configuration_;

  /// @brief Generalized velocity around which the controller is linearized.
  Eigen::VectorXd desired_velocity_;

  /// @brief Configuration around which the controller is linearized.
  Eigen::VectorXd measured_configuration_;

  /// @brief Generalized velocity around which the controller is linearized.
  Eigen::VectorXd measured_velocity_;

  /// @brief PD controller desired torque.
  Eigen::VectorXd pd_desired_torque_;

  /// @brief Linear feedback controller desired torque.
  Eigen::VectorXd lf_desired_torque_;

  /// @brief Weighted desired torque between the PD and the LF controllers.
  Eigen::VectorXd weighted_desired_torque_;

  /// @brief Initial joint torque.
  Eigen::VectorXd initial_torque_;

  AveragingFilter<Eigen::VectorXd> initial_torque_filter_;

  /// @brief Initial joint position, used to freeze the robot around this value.
  Eigen::VectorXd initial_position_;

  AveragingFilter<Eigen::VectorXd> initial_position_filter_;

  /**
   *  @brief Difference between the measured and the desired state,
   *  \$f x = [ q^T, \dot{q}^T ]^T \$f
   *
   */
  Eigen::VectorXd diff_state_;

  /** @brief Mutex used in the update function in order to prevent updates of
   * the user control while the desired joint torque is computed. This mutex
   * is also used in the callback function of the subscriber.
   */
  std::timed_mutex mutex_;

  /// @brief Time during which we wait for the mutex to get unlocked.
  int ms_mutex_;

  /// @brief List of end-effector swinging, hence not in contact.
  std::vector<std::string> desired_swing_ids_;

  /// @brief List of end-effector in contact, hence not swinging.
  std::vector<std::string> desired_stance_ids_;

  // /// @brief Allow to reconfigure online the PD gains used during the initialization.
  // ddynamic_reconfigure::DDynamicReconfigurePtr dd_reconfigure_;

  /// @brief Arm P gain of the PD control of the joints when the robot is standing still.
  double p_arm_gain_;

  /// @brief Arm D gain of the PD control of the joints when the robot is standing still.
  double d_arm_gain_;

  /// @brief Torso P gain of the PD control of the joints when the robot is standing still.
  double p_torso_gain_;

  /// @brief Torso D gain of the PD control of the joints when the robot is standing still.
  double d_torso_gain_;

  /// @brief Leg P gain of the PD control of the joints when the robot is standing still.
  double p_leg_gain_;

  /// @brief Leg D gain of the PD control of the joints when the robot is standing still.
  double d_leg_gain_;

  /// @brief Min jerk for control switch.
  MinJerk min_jerk_;

  /// @brief Time from which we compute the linear feedback control.
  ros::Time init_lfc_time_;
};

template <class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
  os << "vector size: " << v.size() << "\n";
  for (std::size_t i = 0; i < v.size(); ++i)
  {
    os << "[" << i << "] " << v[i] << "\n";
  }
  os << std::endl;
  return os;
}

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_HPP
