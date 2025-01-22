#include "linear_feedback_controller/linear_feedback_controller.hpp"
using linear_feedback_controller::ControllerParameters;
using linear_feedback_controller::LinearFeedbackController;

#include "gtest/gtest.h"

TEST(LinearFeedbackControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LinearFeedbackController{}; });
}

TEST(LinearFeedbackControllerTest, LoadEmptyParams) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_FALSE(ctrl.load({}));
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadNoURDF) {
  auto ctrl = LinearFeedbackController{};
  auto params = ControllerParameters{};
  // TODO
  EXPECT_FALSE(ctrl.load(params));
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadSizeMismatch) {
  auto ctrl = LinearFeedbackController{};
  auto params = ControllerParameters{};
  // TODO
  EXPECT_FALSE(ctrl.load(params));
}

TEST(LinearFeedbackControllerTest, DISABLED_LoadNegativeDuration) {
  auto ctrl = LinearFeedbackController{};
  auto params = ControllerParameters{};
  // TODO
  EXPECT_FALSE(ctrl.load(params));
}

TEST(LinearFeedbackControllerTest, Load) {
  auto ctrl = LinearFeedbackController{};
  auto params = ControllerParameters{};
  // TODO
  EXPECT_TRUE(ctrl.load(params));
}
