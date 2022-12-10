/* MinJerks object for trajectories. */

#include "linear_feedback_controller/min_jerk.hpp"

#include <iostream>

namespace linear_feedback_controller {

MinJerk::MinJerk()
    : start_time_(0.0),
      start_pos_(0.0),
      start_speed_(0.0),
      start_acc_(0.0),
      end_time_(0.0),
      end_pos_(0.0),
      end_speed_(0.0),
      end_acc_(0.0),
      coeffs_({{0.0, 0.0, 0.0, 0.0, 0.0}})

{}

MinJerk::~MinJerk() {}

double MinJerk::compute(double t) {
  if (t >= end_time_) {
    return end_pos_;
  } else if (t <= start_time_) {
    return start_pos_;
  } else {
    double r = 0.0;
    double pt = 1.0;
    for (std::size_t i = 0; i < coeffs_.size(); i++) {
      r += coeffs_[i] * pt;
      pt *= t;
    }
    return r;
  }
}

double MinJerk::computeDerivative(double t) {
  if (t >= end_time_) {
    return end_speed_;
  } else if (t <= start_time_) {
    return start_speed_;
  } else {
    double r = 0, pt = 1;
    for (std::size_t i = 1; i < coeffs_.size(); i++) {
      r += i * coeffs_[i] * pt;
      pt *= t;
    }
    return r;
  }
}

double MinJerk::computeSecDerivative(double t) {
  if (t >= end_time_) {
    return end_acc_;
  } else if (t <= start_time_) {
    return start_acc_;
  } else {
    double r = 0, pt = 1;
    for (std::size_t i = 2; i < coeffs_.size(); i++) {
      r += i * (i - 1) * coeffs_[i] * pt;
      pt *= t;
    }
    return r;
  }
}

double MinJerk::computeJerk(double t) {
  if (t >= end_time_) {
    return computeJerk(end_time_);
  } else if (t <= start_time_) {
    return computeJerk(start_time_);
  } else {
    double r = 0, pt = 1;
    for (std::size_t i = 3; i < coeffs_.size(); i++) {
      r += i * (i - 1) * (i - 2) * coeffs_[i] * pt;
      pt *= t;
    }
    return r;
  }
}

void MinJerk::getCoefficients(std::array<double, 5> &coeffs) const {
  coeffs = coeffs_;
}

void MinJerk::setCoefficients(const std::array<double, 5> &lCoefficients) {
  coeffs_ = lCoefficients;
}

void MinJerk::setParameters(double end_time, double end_pos) {
  setParameters(end_time, 0.0 /*start_pos*/, 0.0 /*start_speed*/,
                0.0 /*start_acc*/, end_pos, 0.0 /*end_speed*/, 0.0 /*end_acc*/);
}

void MinJerk::setParameters(double end_time, double start_pos,
                            double start_speed, double start_acc,
                            double end_pos, double end_speed, double end_acc) {
  // copy the argument internally;
  end_time_ = end_time;
  start_pos_ = start_pos;
  start_speed_ = start_speed;
  start_acc_ = start_acc;
  end_pos_ = end_pos;
  end_speed_ = end_speed;
  end_acc_ = end_acc;

  // Lower order coeffs.
  coeffs_[0] = start_time_;
  coeffs_[1] = start_speed_;
  coeffs_[2] = start_acc_ * 0.5;

  // Higher order coeffs.
  double tmp = end_time_ * end_time_ * end_time_;
  if (tmp == 0.0) {
    coeffs_[3] = 0.0;
    coeffs_[4] = 0.0;
    coeffs_[5] = 0.0;
  } else {
    coeffs_[3] =
        -(1.5 * start_acc_ * end_time_ * end_time_ -
          0.5 * end_acc_ * end_time_ * end_time_ +
          6.0 * start_speed * end_time_ + 4.0 * end_speed_ * end_time_ +
          10.0 * start_pos_ - 10.0 * end_pos_) /
        tmp;
    tmp = tmp * end_time_;
    coeffs_[4] =
        (1.5 * start_acc_ * end_time_ * end_time_ -
         end_acc_ * end_time_ * end_time_ + 8.0 * start_speed * end_time_ +
         7.0 * end_speed_ * end_time_ + 15.0 * start_pos_ - 15.0 * end_pos_) /
        tmp;
    tmp = tmp * end_time_;
    coeffs_[5] =
        -(0.5 * start_acc_ * end_time_ * end_time_ -
          0.5 * end_acc_ * end_time_ * end_time_ +
          3.0 * start_speed * end_time_ + 3.0 * end_speed_ * end_time_ +
          6.0 * start_pos_ - 6.0 * end_pos_) /
        tmp;
  }
}

}  // namespace linear_feedback_controller
