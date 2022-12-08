/* ContactDetectors object for trajectories. */

#include "linear_feedback_controller/contact_detector.hpp"

namespace linear_feedback_controller {

ContactDetector::ContactDetector()
    : lower_threshold_(0.0),
      upper_threshold_(0.0),
      contact_counter_(0),
      threshold_contact_counter_(10),
      contact_state_(NO_CONTACT) {}

ContactDetector::~ContactDetector() {}

bool ContactDetector::detectContact(Eigen::Matrix<double, 6, 1> wrench) {
  // Compute state of contact
  bool force_above_threshold(wrench.head<3>().norm() >= upper_threshold_);
  bool force_below_threshold(wrench.head<3>().norm() <= lower_threshold_);
  ContactState res;

  switch (contactState_) {
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
        if (contact_counter_ >= threshold_contact_counter_) {
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
        if (contact_counter_ >= threshold_contact_counter_) {
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

  contactState_ = res;
  if (contact_state_ == ACTIVE_CONTACT || contact_state_ == RELEASING_CONTACT) {
    return true;
  } else {
    return false;
  }
}

}  // namespace linear_feedback_controller
