#ifndef LINEAR_FEEDBACK_CONTROLLER_CONTROLLER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_CONTROLLER_HPP

#include <chrono>
#include <map>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "pinocchio/algorithm/rnea.hpp"

// local include
#include "linear_feedback_controller/lf_controller.hpp"
#include "linear_feedback_controller/pd_controller.hpp"
#include "linear_feedback_controller/robot_model_builder.hpp"
#include "linear_feedback_controller/time.hpp"
#include "linear_feedback_controller/visibility.hpp"

namespace linear_feedback_controller {

struct LINEAR_FEEDBACK_CONTROLLER_PUBLIC ControllerParameters {
  std::string urdf;
  std::vector<std::string> moving_joint_names;
  std::vector<double> p_gains;
  std::vector<double> d_gains;
  std::vector<std::string> controlled_joint_names;
  bool robot_has_free_flyer;
  Duration pd_to_lf_transition_duration;
};

/**
 * @brief This class has for purpose to connect Whole Body Model Predictive
 * Controllers from https://github.com/loco-3d/crocoddyl or
 * https://github.com/Simple-Robotics/aligator and to the low level controller.
 *
 * This part of the controller interpolates the controls using the Ricatti gains
 * and the feed-forward terms. It runs at the robot low level frequency.
 *
 * In essence it computes the following joint torques \f$ \tau \f$:
 *
 * \f[
 *  \tau = K_{feedback} * (x^{des} - x^{act}) + \tau_0
 * \f]
 *
 * With \f$ K_{feedback} \f$ being the feedback gain matrix a.k.a the Ricatti
 * gains, \f$ x^{des} \f$ and \f$ x^{act} \f$ being
 * respectively the desired and actual state of the controller, and finally
 * \f$ \tau_0 \f$ the feed-forward term.
 * Note that \f$ x^{des} \f$ is the point where the control has been linearized.
 * Hence it's the \f$ x_0 \f$ of the optimal control problem.
 *
 */
class LINEAR_FEEDBACK_CONTROLLER_PUBLIC LinearFeedbackController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Sensor = linear_feedback_controller_msgs::Eigen::Sensor;
  using Control = linear_feedback_controller_msgs::Eigen::Control;

  LinearFeedbackController();
  ~LinearFeedbackController();

  bool load(const ControllerParameters& params);

  bool set_initial_state(Eigen::VectorXd const& tau_init,
                         Eigen::VectorXd const& jq_init);

  /**
   * @brief
   *
   * @param time
   * @param sensor
   * @param control
   * @return const Eigen::VectorXd&
   */
  const Eigen::VectorXd& compute_control(
      const TimePoint& time, const Sensor& sensor, const Control& control,
      const bool remove_gravity_compensation_effort);

  RobotModelBuilder::ConstSharedPtr get_robot_model() const;

 private:
  ControllerParameters params_; /*! @brief Parameters of the controller. */
  /// @brief Control to be sent to the low-level controller.
  Eigen::VectorXd control_;

  /// @brief Rigid body model of the robot.
  RobotModelBuilder::SharedPtr robot_model_builder_;
  /// @brief A simple PD controller to hold the robot still at the beginning.
  PDController pd_controller_;
  /// @brief The actual linear feedback controller.
  LFController lf_controller_;
  /// @brief Time at which we received the first control.
  TimePoint first_control_received_time_;
  /// @brief Robot generalized coordinates.
  Eigen::VectorXd robot_configuration_;
  /// @brief Robot generalized velocity coordinates.
  Eigen::VectorXd robot_velocity_;
  /// @brief Robot generalized tangent space 0 coordinates.
  Eigen::VectorXd robot_velocity_null_;
  /// @brief Initial torque used in the PD (and optionally subtracted).
  Eigen::VectorXd tau_init_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_CONTROLLER_HPP
