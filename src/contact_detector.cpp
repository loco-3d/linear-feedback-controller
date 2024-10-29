#include "linear_feedback_controller/contact_detector.hpp"

namespace linear_feedback_controller {

ContactDetector::ContactDetector()
    : contact_counter_(0), contact_state_(NO_CONTACT) {
  params_.lower_threshold_ = 0.0;
  params_.upper_threshold_ = 0.0;
  params_.threshold_contact_counter_ = 0;
}

ContactDetector::~ContactDetector() {}

bool ContactDetector::detectContact(Eigen::Matrix<double, 6, 1> wrench) {
  // Compute state of contact
  bool force_above_threshold(wrench.head<3>().norm() >=
                             params_.upper_threshold_);
  bool force_below_threshold(wrench.head<3>().norm() <=
                             params_.lower_threshold_);
  ContactState res = contact_state_;

  switch (contact_state_) {
    case NO_CONTACT:
      if (force_above_threshold) {
        // first detection, switch to next state and increase counter
        contact_counter_ = 1;
        res = GOING_TO_CONTACT;
      } else {
        // otherwise remain in state NO_CONTACT
        res = NO_CONTACT;
      }
      break;
    case GOING_TO_CONTACT:
      if (force_above_threshold) {
        // increase counter
        ++contact_counter_;
        if (contact_counter_ >= params_.threshold_contact_counter_) {
          // if number of iteration reached, switch to ACTIVE_CONTACT.
          res = ACTIVE_CONTACT;
          contact_counter_ = 0;
        } else {
          res = GOING_TO_CONTACT;
        }
      } else {
        // Return to state NO_CONTACT
        res = NO_CONTACT;
        contact_counter_ = 0;
      }
      break;
    case ACTIVE_CONTACT:
      if (force_below_threshold) {
        // Go to state RELEASING_CONTACT
        contact_counter_ = 1;
        res = RELEASING_CONTACT;
      } else {
        // Stay in this state
        res = ACTIVE_CONTACT;
      }
      break;
    case RELEASING_CONTACT:
      if (force_below_threshold) {
        // increase counter
        ++contact_counter_;
        if (contact_counter_ >= params_.threshold_contact_counter_) {
          // if number of iteration reached, switch to NO_CONTACT.
          res = NO_CONTACT;
          contact_counter_ = 0;
        } else {
          res = RELEASING_CONTACT;
        }
      } else {
        contact_counter_ = 0;
        res = ACTIVE_CONTACT;
      }
      break;
  };

  contact_state_ = res;
  if (contact_state_ == ACTIVE_CONTACT || contact_state_ == RELEASING_CONTACT) {
    return true;
  } else {
    return false;
  }
}

}  // namespace linear_feedback_controller
