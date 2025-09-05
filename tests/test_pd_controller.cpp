#include <vector>

#include "Eigen/Core"
#include "gtest/gtest.h"
#include "linear_feedback_controller/pd_controller.hpp"

// Basic fixture
class PDControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    controller_ = std::make_unique<linear_feedback_controller::PDController>();
  }

  std::unique_ptr<linear_feedback_controller::PDController> controller_;
};

// Verify that controller does not throw exception
TEST_F(PDControllerTest, ConstructorDoesNotThrow) {
  ASSERT_NE(controller_, nullptr);
}

// Validate gains configuration with std::vector and effects.
TEST_F(PDControllerTest, SetGainsWithStdVector) {
  std::vector<double> p_gains = {10.0, 20.0};
  std::vector<double> d_gains = {1.0, 2.0};
  controller_->set_gains(p_gains, d_gains);

  // call compute_control because we do not have getters.
  Eigen::VectorXd tau_ref(2), q_ref(2), q(2), v(2);
  tau_ref << 0.0, 0.0;
  q_ref << 1.0, 1.0;
  q << 0.5, 0.5;
  v << 0.1, 0.2;
  controller_->set_reference(tau_ref, q_ref);

  const Eigen::VectorXd& tau = controller_->compute_control(q, v);

  // Kp * (q_ref - q) - Kd * v
  // tau[0] = 10.0 * (1.0 - 0.5) - 1.0 * 0.1 = 5.0 - 0.1 = 4.9
  // tau[1] = 20.0 * (1.0 - 0.5) - 2.0 * 0.2 = 10.0 - 0.4 = 9.6
  EXPECT_NEAR(tau(0), 4.9, 1e-9);
  EXPECT_NEAR(tau(1), 9.6, 1e-9);
}

// Verify error on gain size
TEST_F(PDControllerTest, SetGainsWithMismatchedSizesThrows) {
  Eigen::VectorXd p_gains(2);
  Eigen::VectorXd d_gains(3);
  EXPECT_THROW(controller_->set_gains(p_gains, d_gains), std::invalid_argument);

  std::vector<double> p_gains_vec = {1.0, 2.0};
  std::vector<double> d_gains_vec = {1.0};
  EXPECT_THROW(controller_->set_gains(p_gains_vec, d_gains_vec),
               std::invalid_argument);
}

// Verify error on references size
TEST_F(PDControllerTest, SetReferenceWithMismatchedSizesThrows) {
  Eigen::VectorXd tau_ref(2);
  Eigen::VectorXd q_ref(3);
  EXPECT_THROW(controller_->set_reference(tau_ref, q_ref),
               std::invalid_argument);
}

// Verify error on NaN and INF gains values using Eigen
TEST_F(PDControllerTest, SetGainsWithSpecialValuesThrows) {
  Eigen::VectorXd p_gains = Eigen::VectorXd::Ones(3);
  Eigen::VectorXd d_gains = Eigen::VectorXd::Ones(3);

  double inf = std::numeric_limits<double>::infinity();
  double nan = std::numeric_limits<double>::quiet_NaN();

  // Test with NaN
  p_gains(1) = nan;
  EXPECT_THROW(controller_->set_gains(p_gains, d_gains), std::invalid_argument);

  // Test with Infinity
  p_gains(1) = 1.0;  // reset
  d_gains(2) = inf;
  EXPECT_THROW(controller_->set_gains(p_gains, d_gains), std::invalid_argument);
}

// Verify error on NaN and INF gains values using vectors
TEST_F(PDControllerTest, SetGainsFromStdVectorWithSpecialValuesThrows) {
  std::vector<double> p_gains_vec = {1.0, 1.0, 1.0};
  std::vector<double> d_gains_vec = {1.0, 1.0, 1.0};

  double inf = std::numeric_limits<double>::infinity();
  double nan = std::numeric_limits<double>::quiet_NaN();

  // Test with NaN
  p_gains_vec[1] = nan;
  EXPECT_THROW(controller_->set_gains(p_gains_vec, d_gains_vec),
               std::invalid_argument);

  // Test with Infinity
  p_gains_vec[1] = 1.0;  // reset
  d_gains_vec[2] = inf;
  EXPECT_THROW(controller_->set_gains(p_gains_vec, d_gains_vec),
               std::invalid_argument);
}

// Verify error on NaN and INF references values
TEST_F(PDControllerTest, SetReferenceWithSpecialValuesThrows) {
  Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd tau_ref = Eigen::VectorXd::Zero(3);

  double inf = std::numeric_limits<double>::infinity();
  double nan = std::numeric_limits<double>::quiet_NaN();

  q_ref(0) = nan;
  EXPECT_THROW(controller_->set_reference(tau_ref, q_ref),
               std::invalid_argument);

  q_ref(0) = 0.0;
  tau_ref(1) = inf;
  EXPECT_THROW(controller_->set_reference(tau_ref, q_ref),
               std::invalid_argument);
}

// TODO : empty_vec -> should it be checked ?
TEST_F(PDControllerTest, SetGainsWithEmptyVectors) {
  Eigen::VectorXd empty_vec;

  EXPECT_NO_THROW(controller_->set_gains(empty_vec, empty_vec));
}

// Fixture for computation tests.
class PDControllerComputationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    controller_ = std::make_unique<linear_feedback_controller::PDController>();
    dof_ = 3;

    p_gains_.resize(dof_);
    d_gains_.resize(dof_);
    tau_ref_.resize(dof_);
    q_ref_.resize(dof_);

    p_gains_ << 10.0, 20.0, 30.0;
    d_gains_ << 1.0, 2.0, 3.0;
    tau_ref_ << 5.0, 6.0, 7.0;
    q_ref_ << 1.0, 1.0, 1.0;

    controller_->set_gains(p_gains_, d_gains_);
    controller_->set_reference(tau_ref_, q_ref_);
  }

  std::unique_ptr<linear_feedback_controller::PDController> controller_;
  int dof_;
  Eigen::VectorXd p_gains_, d_gains_, tau_ref_, q_ref_;
};

// Basic computation test
TEST_F(PDControllerComputationTest, ComputeControlMultiDimensional) {
  Eigen::VectorXd q(dof_);
  Eigen::VectorXd v(dof_);
  q << 0.5, 0.8, 0.9;
  v << 0.1, -0.2, 0.3;

  const Eigen::VectorXd& tau = controller_->compute_control(q, v);

  // tau = tau_ref + Kp * (q_ref - q) - Kd * v
  // tau[0] = 5.0 + 10.0 * (1.0 - 0.5) - 1.0 * 0.1   = 5.0 + 5.0 - 0.1 = 9.9
  // tau[1] = 6.0 + 20.0 * (1.0 - 0.8) - 2.0 * (-0.2) = 6.0 + 4.0 + 0.4 = 10.4
  // tau[2] = 7.0 + 30.0 * (1.0 - 0.9) - 3.0 * 0.3   = 7.0 + 3.0 - 0.9 = 9.1
  ASSERT_EQ(tau.size(), dof_);
  EXPECT_NEAR(tau(0), 9.9, 1e-9);
  EXPECT_NEAR(tau(1), 10.4, 1e-9);
  EXPECT_NEAR(tau(2), 9.1, 1e-9);

  // Check that method return only a reference of internal vector.
  const Eigen::VectorXd& tau2 = controller_->compute_control(q, v);
  EXPECT_EQ(&tau, &tau2);
}

// case if no error on q and v.
TEST_F(PDControllerComputationTest, ComputeControlZeroError) {
  Eigen::VectorXd q = q_ref_;
  Eigen::VectorXd v(dof_);
  v.setZero();

  const Eigen::VectorXd& tau = controller_->compute_control(q, v);

  // tau = tau_ref + Kp * 0 - Kd * 0 = tau_ref
  ASSERT_EQ(tau.size(), dof_);
  for (int i = 0; i < dof_; ++i) {
    EXPECT_NEAR(tau(i), tau_ref_(i), 1e-9);
  }
}

// Verify exception on (q, v) size.
TEST_F(PDControllerComputationTest,
       ComputeControlWithMismatchedInputSizesThrows) {
  Eigen::VectorXd q_wrong_size(dof_ - 1);
  Eigen::VectorXd v_correct_size(dof_);
  v_correct_size.setZero();

  Eigen::VectorXd q_correct_size(dof_);
  Eigen::VectorXd v_wrong_size(dof_ + 1);
  q_correct_size.setZero();

  EXPECT_THROW(controller_->compute_control(q_wrong_size, v_correct_size),
               std::invalid_argument);
  EXPECT_THROW(controller_->compute_control(q_correct_size, v_wrong_size),
               std::invalid_argument);
}
