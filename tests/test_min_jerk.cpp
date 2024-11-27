#include <gtest/gtest.h>

#include <random>

#include "linear_feedback_controller/min_jerk.hpp"

using namespace linear_feedback_controller;

class MinJerkTest : public ::testing::Test {
 protected:
  void SetUp() override {
    start_time_ = 0.0;
    end_time_ = 1.0;
    range_time_ = end_time_ - start_time_;
    dt_ = range_time_ / 1000.0;

    // Random
    double lower_bound = -1e3;
    double upper_bound = 1e3;
    std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
    std::default_random_engine re{static_cast<long unsigned int>(time(0))};

    start_pos_ = unif(re);
    end_pos_ = unif(re);
  }

  void TearDown() override {}

  double start_time_;
  double end_time_;

  double start_pos_;
  double end_pos_;
  double range_time_;
  double dt_;
};
class DISABLED_MinJerkTest : public MinJerkTest {};

TEST_F(MinJerkTest, checkConstructor) { MinJerk obj; }

TEST_F(MinJerkTest, checkComputeExtremes) {
  MinJerk obj;
  obj.set_parameters(end_time_, start_pos_,
                     /*start_speed*/ 0.0,
                     /*start_acc*/ 0.0, end_pos_,
                     /*end_speed*/ 0.0,
                     /*end_acc*/ 0.0);

  // Test below min.
  for (double t = start_time_ - range_time_; t < start_time_; t += dt_) {
    ASSERT_NEAR(obj.compute(t), start_pos_, 1e-8);
    ASSERT_NEAR(obj.compute_derivative(t), 0.0, 1e-8);
    ASSERT_NEAR(obj.compute_derivative(t), 0.0, 1e-8);
    ASSERT_NEAR(obj.compute_sec_derivative(t), 0.0, 1e-8);
    ASSERT_NEAR(obj.compute_sec_derivative(t), 0.0, 1e-8);
  }
  // Test below max.
  for (double t = end_time_; t < end_time_ + range_time_; t += dt_) {
    ASSERT_NEAR(obj.compute(t), end_pos_, 1e-8);
    ASSERT_NEAR(obj.compute_derivative(t), 0.0, 1e-8);
    ASSERT_NEAR(obj.compute_derivative(t), 0.0, 1e-8);
    ASSERT_NEAR(obj.compute_sec_derivative(t), 0.0, 1e-8);
    ASSERT_NEAR(obj.compute_sec_derivative(t), 0.0, 1e-8);
  }
  // Test extremities
  ASSERT_NEAR(obj.compute(start_time_), start_pos_, 1e-8);
  ASSERT_NEAR(obj.compute(end_time_), end_pos_, 1e-8);
  ASSERT_NEAR(obj.compute_derivative(start_time_), 0.0, 1e-8);
  ASSERT_NEAR(obj.compute_derivative(end_time_), 0.0, 1e-8);
  ASSERT_NEAR(obj.compute_sec_derivative(start_time_), 0.0, 1e-8);
  ASSERT_NEAR(obj.compute_sec_derivative(end_time_), 0.0, 1e-8);
}

TEST_F(MinJerkTest, checkComputeBetween) {
  start_pos_ = 0;
  end_pos_ = 1;

  MinJerk obj;
  obj.set_parameters(end_time_, start_pos_,
                     /*start_speed*/ 0.0,
                     /*start_acc*/ 0.0, end_pos_,
                     /*end_speed*/ 0.0,
                     /*end_acc*/ 0.0);

  // Test extremities
  ASSERT_NEAR(obj.compute(start_time_), start_pos_, 1e-8);
  ASSERT_NEAR(obj.compute(end_time_), end_pos_, 1e-8);
  ASSERT_NEAR(obj.compute_derivative(start_time_), 0.0, 1e-8);
  ASSERT_NEAR(obj.compute_derivative(end_time_), 0.0, 1e-8);
  ASSERT_NEAR(obj.compute_sec_derivative(start_time_), 0.0, 1e-8);
  ASSERT_NEAR(obj.compute_sec_derivative(end_time_), 0.0, 1e-8);
  // Test between min.
  for (double t = start_time_; t <= end_time_; t += dt_) {
    ASSERT_LE(obj.compute(t), std::max(end_pos_, start_pos_))
        << "start_time_ = " << start_time_ << std::endl
        << "end_time_ = " << end_time_ << std::endl
        << "t = " << t << std::endl
        << "start_pos_ = " << start_pos_ << std::endl
        << "end_pos_ = " << end_pos_ << std::endl
        << "obj.compute(t) = " << obj.compute(t) << std::endl
        << "std::max(end_pos_, start_pos_) = " << std::max(end_pos_, start_pos_)
        << std::endl;
    ASSERT_GE(obj.compute(t), std::min(end_pos_, start_pos_))
        << "start_time_ = " << start_time_ << std::endl
        << "end_time_ = " << end_time_ << std::endl
        << "t = " << t << std::endl
        << "start_pos_ = " << start_pos_ << std::endl
        << "end_pos_ = " << end_pos_ << std::endl
        << "obj.compute(t) = " << obj.compute(t) << std::endl
        << "std::min(end_pos_, start_pos_) = " << std::min(end_pos_, start_pos_)
        << std::endl;
  }
}

TEST_F(DISABLED_MinJerkTest, checkComputeBetweenRandom) {
  MinJerk obj;
  obj.set_parameters(end_time_, start_pos_,
                     /*start_speed*/ 0.0,
                     /*start_acc*/ 0.0, end_pos_,
                     /*end_speed*/ 0.0,
                     /*end_acc*/ 0.0);

  // Test extremities
  ASSERT_NEAR(obj.compute(start_time_), start_pos_, 1e-8);
  ASSERT_NEAR(obj.compute(end_time_), end_pos_, 1e-8);
  ASSERT_NEAR(obj.compute_derivative(start_time_), 0.0, 1e-8);
  ASSERT_NEAR(obj.compute_derivative(end_time_), 0.0, 1e-8);
  ASSERT_NEAR(obj.compute_sec_derivative(start_time_), 0.0, 1e-8);
  ASSERT_NEAR(obj.compute_sec_derivative(end_time_), 0.0, 1e-8);
  // Test between min.
  for (double t = start_time_; t <= end_time_; t += dt_) {
    ASSERT_LE(obj.compute(t), std::max(end_pos_, start_pos_))
        << "start_time_ = " << start_time_ << std::endl
        << "end_time_ = " << end_time_ << std::endl
        << "t = " << t << std::endl
        << "start_pos_ = " << start_pos_ << std::endl
        << "end_pos_ = " << end_pos_ << std::endl
        << "obj.compute(t) = " << obj.compute(t) << std::endl
        << "std::max(end_pos_, start_pos_) = " << std::max(end_pos_, start_pos_)
        << std::endl;
    ASSERT_GE(obj.compute(t), std::min(end_pos_, start_pos_))
        << "start_time_ = " << start_time_ << std::endl
        << "end_time_ = " << end_time_ << std::endl
        << "t = " << t << std::endl
        << "start_pos_ = " << start_pos_ << std::endl
        << "end_pos_ = " << end_pos_ << std::endl
        << "obj.compute(t) = " << obj.compute(t) << std::endl
        << "std::min(end_pos_, start_pos_) = " << std::min(end_pos_, start_pos_)
        << std::endl;
  }
}
