#include <memory>
#include <string_view>

#include "gtest/gtest.h"
#include "linear_feedback_controller/lf_controller.hpp"

using namespace linear_feedback_controller;

// EXAMPLE_ROBOT_DATA_MODEL_DIR is a compile definition imported when linking to
// example-robot-data
constexpr auto ROBOT_MODEL_DIR_PATH =
    std::string_view{EXAMPLE_ROBOT_DATA_MODEL_DIR};

TEST(LfControllerTest, Ctor) {
  EXPECT_NO_THROW({ auto ctrl = LFController(); });
}

TEST(LfControllerTest, InitializeNullptr) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({
    // Null ?
    ctrl.initialize(nullptr);
  });
}

TEST(LfControllerTest, InitializeEmptyModel) {
  auto ctrl = LFController();
  EXPECT_ANY_THROW({
    // Empty Robot model
    ctrl.initialize(std::make_shared<RobotModelBuilder>());
  });
}
