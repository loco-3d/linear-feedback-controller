#ifndef LINEAR_FEEDBACK_CONTROLLER_CONTACT_DETECTOR_HPP
#define LINEAR_FEEDBACK_CONTROLLER_CONTACT_DETECTOR_HPP

#include <Eigen/Core>

namespace linear_feedback_controller {

class ContactDetector {
 public:
  /**
   * @brief State of the contact:
   *  - NO_CONTACT: There is no contact.
   *  - GOING_TO_CONTACT: The contact is settling down but not active yet.
   *  - ACTIVE_CONTACT: There is contact.
   *  - RELEASING_CONTACT: The contact is starting to get released.
   */
  enum ContactState {
    NO_CONTACT = 0,
    GOING_TO_CONTACT,
    ACTIVE_CONTACT,
    RELEASING_CONTACT
  };

 public:
  /**
   * @brief Construct a new Averaging Filter.
   */
  ContactDetector();

  /**
   * @brief Destroy the Averaging Filter.
   */
  ~ContactDetector();

  /**
   * @brief Detects if we have contact at the point where the wrench is
   * measured.
   *
   * @param wrench
   * @return true
   * @return false
   */
  bool detectContact(Eigen::Matrix<double, 6, 1> wrench);

  /**
   * @brief Set the Lower Threshold.
   *
   * @param threshold
   */
  void setLowerThreshold(double threshold) { lower_threshold_ = threshold; }

  /**
   * @brief Get the Lower Threshold.
   */
  const double& getLowerThreshold() { return lower_threshold_; }

  /**
   * @brief Set the Upper Threshold.
   * @param threshold
   */
  void setUpperThreshold(double threshold) { upper_threshold_ = threshold; }

  /**
   * @brief Get the Upper Threshold.
   */
  const double& getUpperThreshold() { return upper_threshold_; }

  /**
   * @brief Set the Threshold Contact Counter.
   *
   * @param threshold
   */
  void setThresholdContactCounter(int threshold) {
    threshold_contact_counter_ = threshold;
  }

  /**
   * @brief Get the Threshold Contact Counter.
   * @return const int&
   */
  const int& getThresholdContactCounter() { return threshold_contact_counter_; }

 private:
  double lower_threshold_;
  double upper_threshold_;
  int contact_counter_;
  int threshold_contact_counter_;
  ContactState contact_state_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_CONTACT_DETECTOR_HPP
