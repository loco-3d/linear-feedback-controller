#include <memory>

#include "linear_feedback_controller/robot_model_builder.hpp"
using linear_feedback_controller::RobotModelBuilder;

#include "linear_feedback_controller/lf_controller.hpp"
using linear_feedback_controller::LFController;

#include "gtest/gtest.h"

class LfControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {}

  void SetUp() override {}

  void TearDown() override {}

  static void TearDownTestSuite() {}
};

TEST_F(LfControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LFController(); });
}

TEST_F(LfControllerTest, InitializeNullptr) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({
    // Null ?
    ctrl.initialize(nullptr);
  });
}

TEST_F(LfControllerTest, InitializeEmptyModel) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({
    // Empty Robot model
    ctrl.initialize(std::make_shared<RobotModelBuilder>());
  });
}
