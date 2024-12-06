#include "gtest/gtest.h"

#include <memory>

#include "linear_feedback_controller/pd_controller.hpp"

using namespace linear_feedback_controller;

class UninitializedPdControllerTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}

  std::unique_ptr<PDController> pd_ctrl_ptr_ = nullptr;
};

using PdControllerTest = UninitializedPdControllerTest;
TEST_F(PdControllerTest, Ctor)
{
  EXPECT_NO_THROW({ pd_ctrl_ptr_ = std::make_unique<PDController>(); });
}
