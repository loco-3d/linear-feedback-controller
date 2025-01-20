#include "linear_feedback_controller/linear_feedback_controller.hpp"
using linear_feedback_controller::LinearFeedbackController;

#include "gtest/gtest.h"

TEST(LinearFeedbackControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LinearFeedbackController{}; });
}

TEST(LinearFeedbackControllerTest, LoadEmptyParams) {
  auto ctrl = LinearFeedbackController{};
  EXPECT_FALSE(ctrl.load({}));
}
