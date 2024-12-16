#include "example-robot-data/path.hpp"
#include "gtest/gtest.h"
#include "linear_feedback_controller/lf_controller.hpp"

using namespace linear_feedback_controller;

TEST(LfControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LFController(); });
}
