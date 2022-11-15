#ifndef LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_HPP

// Standard C++
// #include <memory>

// Rigid body dynamics
// #include <pinocchio/fwd.hpp>
// #include <pinocchio/multibody/model.hpp>

// ROS C++ api
#include <ros/ros.h>
// #include <realtime_tools/realtime_publisher.h>

// export this lib as plugin
#include <pluginlib/class_list_macros.h>

// ROS Messages.
// #include <linear_feedback_controller_msgs/Sensor.h>
// #include <linear_feedback_controller_msgs/Control.h>

// PAL roscontrol controller containing their estimator.
#include <pal_base_ros_controller/base_robot_with_estimator_controller.h>

// #include <boost/algorithm/string.hpp>
// #include <algorithm>
// #include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/parsers/srdf.hpp>
// #include <pinocchio/algorithm/joint-configuration.hpp>
// #include <tf/transform_broadcaster.h>
// #include <tf2_msgs/TFMessage.h>
// #include <pinocchio/algorithm/model.hpp>

//
// #include <realtime_tools/realtime_publisher.h>
// #include <sensor_msgs/JointState.h>
// #include <memmo_trajectory_controller/JointStateLPF.h>
// #include <ddynamic_reconfigure/ddynamic_reconfigure.h>
// #include <nav_msgs/Odometry.h>

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
   * @brief Construct a new LinearFeedbackController object.
   */
  LinearFeedbackController();

  /**
   * @brief Destroy the Linear Feedback Controller object.
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
          // /**
          //  * @brief Parse the joint moving names given by the user and build the
          //  * rigid body models accordingly.
          //  *
          //  * @param in_moving_joint_names
          //  * @param moving_joint_names
          //  * @param moving_joint_ids
          //  * @param locked_joint_ids
          //  */
          // void parseMovingJointNames(const std::vector<std::string>& in_moving_joint_names,
          //                            std::vector<std::string>& moving_joint_names,
          //                            std::vector<long unsigned int>& moving_joint_ids,
          //                            std::vector<long unsigned int>& locked_joint_ids);
  // /**
  //  * @brief Acquire the control from the external controller.
  //  *
  //  * @param msg
  //  */
  // void controlSubscriberCallback(const linear_feedback_controller_msgs::Control& msg);

  // /**
  //  * @brief Parse the ROS parameters.
  //  *
  //  * @return true
  //  * @return false
  //  */
  // bool parseRosParams(ros::NodeHandle& node_handle, std::string& urdf, std::string& srdf,
  //                     std::vector<std::string>& moving_joint_names,
  //                     bool& robot_has_free_flyer, std::vector<double>& torque_offsets);

  // /**
  //  * @brief Filter the initial state during 1 second in order to start with
  //  * clean data.
  //  */
  // void filterInitialState();

public:  // Setters and getters
  // /**
  //  * @brief Get the moving joint names object.
  //  *
  //  * @return const std::vector<std::string>&
  //  */
  // const std::vector<std::string>& getMovingJointNames() const
  // {
  //   return moving_joint_names_;
  // }

  // /**
  //  * @brief Get the moving joint ids object.
  //  *
  //  * @return const std::vector<long unsigned int>&
  //  */
  // const std::vector<long unsigned int>& getMovingJointIds() const
  // {
  //   return moving_joint_ids_;
  // }

  // /**
  //  * @brief Get the locked joint ids object.
  //  *
  //  * @return const std::vector<long unsigned int>&
  //  */
  // const std::vector<long unsigned int>& getLockedJointIds() const
  // {
  //   return locked_joint_ids_;
  // }

  /**
   * @brief Get the URDF string.
   *
   * @return const std::string&
   */
  const std::string& getUrdf()
  {
    return in_urdf_;
  }

  // /**
  //  * @brief Get the torque offset.
  //  *
  //  * @return const std::vector<long unsigned int>&
  //  */
  // const std::vector<double>& getTorqueOffsets() const
  // {
  //   return in_torque_offsets_;
  // }

private:  // Members
  /// @brief String containing the model of the robot in xml/urdf format.
  std::string in_urdf_;
  // /// @brief String containing the extras of the model of the robot.
  // std::string in_srdf_;
  // /// @brief List of names that correspond to the joints moving by the MPC.
  // std::vector<std::string> in_moving_joint_names_;
  // /// @brief Are we using a robot that has a free-flyer?
  // bool in_robot_has_free_flyer_;
  // /// @brief Joint torque offsets based on the state of the hardware.
  // std::vector<double> in_torque_offsets_;

  // /// @brief Pinocchio (Rigid body dynamics robot model).
  // pinocchio::Model pinocchio_model_complete_;
  // pinocchio::Model pinocchio_model_reduced_;

  // /// @brief Moving joint ids sorted in the urdf order.
  // std::vector<pinocchio::JointIndex> moving_joint_ids_;
  // /// @brief Sort the moving joint names using the urdf order.
  // std::vector<std::string> moving_joint_names_;
  // /// @brief Sort the locked (position moving) joint names using the urdf order.
  // std::vector<pinocchio::JointIndex> locked_joint_ids_;

  // /// @brief Initial whole body configuration setup in the SRDF file.
  // Eigen::VectorXd q_default_complete_;

  // /// @brief Actual robot state publisher.
  // // std::shared_ptr<realtime_tools::RealtimePublisher<linear_feedback_controller_msgs::Sensor> > sensor_publisher_;

  // /// @brief  ROS sensor message data.
  // linear_feedback_controller_msgs::Sensor ros_sensor_msg_;

  // /// @brief Actual robot state publisher.
  // // ros::Subscriber control_subscriber_;

  // /// @brief Initial joint effort measured.
  // std::vector<double> initial_torque_;

  // /// @brief Initial joint position.
  // std::vector<double> initial_position_;

  // /// @brief Desired joint torque.
  // std::vector<double> desired_torque_;

  // /// @brief Measured joint torque.
  // std::vector<double> actual_torque_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_LINEAR_FEEDBACK_CONTROLLER_HPP


// std::timed_mutex mutex_;
// std::vector<std::string> moving_joint_names_;
// std::map<std::string, int> actual_state_map_;
// Eigen::VectorXd desired_effort_;
// Eigen::VectorXd desired_pos_;
// Eigen::VectorXd desired_vel_;
// Eigen::VectorXd actual_pos_;
// Eigen::VectorXd actual_vel_;
// Eigen::VectorXd actual_tau_;
// Eigen::VectorXd desired_tau_;
// Eigen::VectorXd diff_state_;
// Eigen::MatrixXd riccati_gains_;
// memmo_trajectory_controller::JointStateLPF desired_state_;
// std::vector<double> initial_torque_;
// std::vector<double> initial_position_;
// std::vector<double> desired_torque_;
// std::vector<double> actual_torque_;
// std::vector<double> torque_offsets_;

// ddynamic_reconfigure::DDynamicReconfigurePtr dd_reconfigure_;
// double p_arm_gain_;
// double d_arm_gain_;
// double p_torso_gain_;
// double d_torso_gain_;
// double p_leg_gain_;
// double d_leg_gain_;

// boost::shared_ptr<realtime_tools::RealtimePublisher<linear_feedback_controller_msgs::JointState>>
// joint_states_pub_;
// boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> base_state_pub_;
// sensor_msgs::JointState actual_js_state_; nav_msgs::Odometry actual_base_state_;

// pal_statistics::RegistrationsRAII registered_variables_;
// pal_robot_tools::TimeProfilerPtr tp_;
// int ms_mutex_;

// Eigen::Vector3d base_linear_vel_;
// Eigen::Vector3d base_angular_vel_;
// Eigen::Quaterniond base_orientation_;
// Eigen::Vector3d base_position_;
// std::deque<std::vector<double>> v_measures_;
// std::deque<std::vector<double>> tau_measures_;
// double nb_measure_;
