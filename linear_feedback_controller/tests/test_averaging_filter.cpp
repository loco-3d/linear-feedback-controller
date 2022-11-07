#include <gtest/gtest.h>
#include <fstream>
#include <sstream>

#include "linear_feedback_controller/averaging_filter.hpp"

using namespace linear_feedback_controller;

class AveragingFilterTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  void TearDown() override {}
};
class DISABLED_AveragingFilterTest : public AveragingFilterTest {};

TEST_F(AveragingFilterTest, checkConstructor) { AveragingFilter obj; }

TEST_F(AveragingFilterTest, checkDefaultBehaviorWhenNoDataSent) {
  AveragingFilter obj;
  for (unsigned int i = 0; i < 5; ++i) {
    ASSERT_EQ(obj.getFilteredData(), 0.0);
  }
}