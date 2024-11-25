#include "linear_feedback_controller/min_jerk.hpp"

#include <cmath>
#include <iostream>
#include <limits>

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
      coeffs_({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})

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
    for (std::size_t i = 0; i < coeffs_.size(); ++i) {
      r += coeffs_[i] * pt;
      pt *= t;
    }
    return r;
  }
}

double MinJerk::compute_derivative(const double t) {
  if (t >= end_time_) {
    return end_speed_;
  } else if (t <= start_time_) {
    return start_speed_;
  } else {
    double r = 0, pt = 1;
    for (std::size_t i = 1; i < coeffs_.size(); ++i) {
      r += i * coeffs_[i] * pt;
      pt *= t;
    }
    return r;
  }
}

double MinJerk::compute_sec_derivative(const double t) {
  if (t >= end_time_) {
    return end_acc_;
  } else if (t <= start_time_) {
    return start_acc_;
  } else {
    double r = 0, pt = 1;
    for (std::size_t i = 2; i < coeffs_.size(); ++i) {
      r += i * (i - 1) * coeffs_[i] * pt;
      pt *= t;
    }
    return r;
  }
}

double MinJerk::compute_jerk(const double t) {
  if (t >= end_time_) {
    return compute_jerk(end_time_);
  } else if (t <= start_time_) {
    return compute_jerk(start_time_);
  } else {
    double r = 0.0;
    double pt = 1.0;
    for (std::size_t i = 3; i < coeffs_.size(); ++i) {
      r += i * (i - 1) * (i - 2) * coeffs_[i] * pt;
      pt *= t;
    }
    return r;
  }
}

std::array<double, 6> & MinJerk::get_coefficients() const {
  return coeffs_;
}

void MinJerk::set_coefficients(const std::array<double, 6> &lCoefficients) {
  coeffs_ = lCoefficients;
}

void MinJerk::set_parameters(const double end_time, const double end_pos) {
  set_parameters(end_time, 0.0 /*start_pos*/, 0.0 /*start_speed*/,
                 0.0 /*start_acc*/, end_pos, 0.0 /*end_speed*/,
                 0.0 /*end_acc*/);
}

void MinJerk::set_parameters(const double end_time, const double start_pos,
                             const double start_speed, const double start_acc,
                             const double end_pos, const double end_speed, const double end_acc) {
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
  if (std::abs(tmp) <= std::numeric_limits<double>::epsilon()) {
    coeffs_[3] = 0.0;
    coeffs_[4] = 0.0;
    coeffs_[5] = 0.0;
  } else {
    coeffs_[3] =
        -(end_time_ * (12 * start_speed_ + 8 * end_speed_) + 20 * start_pos_ +
          std::pow(end_time_, 2) * (3 * start_acc_ - end_acc_) -
          20 * end_pos_) /
        (2 * std::pow(end_time_, 3));
    coeffs_[4] =
        (end_time_ * (16 * start_speed_ + 14 * end_speed_) + 30 * start_pos_ +
         std::pow(end_time_, 2) * (3 * start_acc_ - 2 * end_acc_) -
         30 * end_pos_) /
        (2 * std::pow(end_time_, 4));
    coeffs_[5] =
        -(end_time_ * (6 * start_speed_ + 6 * end_speed_) + 12 * start_pos_ +
          std::pow(end_time_, 2) * (start_acc_ - end_acc_) - 12 * end_pos_) /
        (2 * std::pow(end_time_, 5));
  }
}

}  // namespace linear_feedback_controller
