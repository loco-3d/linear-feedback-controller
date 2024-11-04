#ifndef LINEAR_FEEDBACK_CONTROLLER_CONTROLLER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_CONTROLLER_HPP

#include <chrono>

#include "Eigen/Core"

// local include
#include "linear_feedback_controller/contact_detector.hpp"
#include "linear_feedback_controller/lf_controller.hpp"
#include "linear_feedback_controller/min_jerk.hpp"
#include "linear_feedback_controller/pd_controller.hpp"
#include "linear_feedback_controller/robot_model_builder.hpp"
#include "linear_feedback_controller/time.hpp"

namespace linear_feedback_controller {

struct ControllerParameters {
  std::string urdf;
  std::string srdf;
  std::vector<std::string> moving_joint_names;
  std::vector<double> p_gains;
  std::vector<double> d_gains;
  std::vector<std::string> controlled_joint_names;
  std::string default_configuration_name;
  bool robot_has_free_flyer;
  std::vector<ContactDetector::Parameters> contact_detector_params;
  Duration from_pd_to_lf_duration;
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
class LinearFeedbackController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Sensor = linear_feedback_controller_msgs::Eigen::Sensor;
  using Control = linear_feedback_controller_msgs::Eigen::Control;

  LinearFeedbackController();
  ~LinearFeedbackController();

  bool load(const ControllerParameters& params);

  bool configure(const Eigen::VectorXd& tau_init,
                 const Eigen::VectorXd& jq_init);

  /**
   * @brief
   *
   * @param time
   * @param sensor
   * @param control
   * @return const Eigen::VectorXd&
   */
  const Eigen::VectorXd& compute_control(TimePoint time, Sensor sensor,
                                         Control control);

  RobotModelBuilder::ConstSharedPtr getRobotModel() const;

 private:
  ControllerParameters params_; /*! @brief Parameters of the controller. */
  /// @brief Control to be sent to the low-level controller.
  Eigen::VectorXd control_;

  /// @brief Rigid body model of the robot.
  RobotModelBuilder::SharedPtr robot_model_builder_;
  /// @brief Some contact detectors in case the robot has a free-flyer.
  std::vector<ContactDetector> contact_detectors_;
  /// @brief A simple PD controller to hold the robot still at the beginning.
  PDController pd_controller_;
  /// @brief A simple PD controller to hold the robot still at the beginning.
  LFController lf_controller_;
  /// @brief Smoother for the switch between the PD and the LFC.
  MinJerk min_jerk_;
  /// @brief Time at which we received the first control.
  TimePoint first_control_received_time_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_CONTROLLER_HPP
