#ifndef LINEAR_FEEDBACK_CONTROLLER_POLYNOMIAL_HPP
#define LINEAR_FEEDBACK_CONTROLLER_POLYNOMIAL_HPP

#include <array>

namespace linear_feedback_controller
{

/// Polynomial used for X,Y and Theta trajectories.
class MinJerk
{
public:
  /// @brief Construct a new Min Jerk object.
  MinJerk();

  /** @brief Destroy the Min Jerk object. */
  ~MinJerk();

  /// Set the parameters
  void setParameters(double end_time, double end_pos);

  /// \brief Set parameters considering initial position, velocity, acceleration,
  /// and final position, velocity and acceleration
  void setParameters(double end_time, double start_pos, double start_speed, double start_acc,
                     double end_pos, double end_speed, double end_acc);

  /*! Compute the value. */
  double compute(double t);

  /*! Compute the value of the derivative. */
  double computeDerivative(double t);

  /*! Compute the value of the second derivative. */
  double computeSecDerivative(double t);

  /*! Compute the value of the third derivative (jerk). */
  double computeJerk(double t);

  /*! Get the coefficients. */
  void getCoefficients(std::array<double, 5> &coeffs) const;

  /*! Set the coefficients. */
  void setCoefficients(const std::array<double, 5> &lCoefficients);

private:
  /// @brief Starting time of the min jerk trajectory
  double start_time_;
  /// @brief starting position.
  double start_pos_;
  /// @brief Starting first derivative.
  double start_speed_;
  /// @brief Starting second derivative.
  double start_acc_;
  /// @brief Ending time of the min jerk trajectory
  double end_time_;
  /// @brief Ending position.
  double end_pos_;
  /// @brief Ending first derivative.
  double end_speed_;
  /// @brief Ending second derivative.
  double end_acc_;
  /// Vector of coefficients.
  std::array<double, 5> coeffs_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_POLYNOMIAL_HPP
