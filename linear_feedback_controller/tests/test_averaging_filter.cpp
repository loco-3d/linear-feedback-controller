#include <gtest/gtest.h>
#include <fstream>
#include <sstream>

#include "linear_feedback_controller/averaging_filter.hpp"

using namespace linear_feedback_controller;

class AveragingFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
};
class DISABLED_AveragingFilterTest : public AveragingFilterTest
{
};

TEST_F(AveragingFilterTest, checkConstructor)
{
  AveragingFilter obj;
}

TEST_F(AveragingFilterTest, checkDefaultBehaviorWhenNoDataSent)
{
  AveragingFilter obj;
  for (unsigned int i = 0; i < 5; ++i)
  {
    ASSERT_EQ(obj.getFilteredData(), 0.0);
  }
}

TEST_F(AveragingFilterTest, checkSimpleValues)
{
  unsigned int max_size = 5;
  AveragingFilter obj;
  obj.setMaxSize(max_size);
  for (unsigned int i = 1; i <= max_size; ++i)
  {
    obj.acquire(static_cast<double>(i));
  }
  double average = (max_size + 1) * max_size * 0.5 / max_size;
  ASSERT_EQ(obj.getFilteredData(), 3.0);
  ASSERT_EQ(obj.getFilteredData(), average);
}

TEST_F(AveragingFilterTest, checkMaxSizeGetterAndSetter)
{
  unsigned int max_size = 5;
  AveragingFilter obj;
  obj.setMaxSize(max_size);
  ASSERT_EQ(obj.getMaxSize(), max_size);
}

TEST_F(AveragingFilterTest, checkFIFOsize)
{
  unsigned int max_size = 5;
  AveragingFilter obj;
  obj.setMaxSize(max_size);
  for (unsigned int i = 1; i <= 2 * max_size; ++i)
  {
    obj.acquire(static_cast<double>(i));
    if(i <= max_size)
    {
      ASSERT_EQ(obj.getBuffer().size(), i);
    }else{
      ASSERT_EQ(obj.getMaxSize(), obj.getBuffer().size());
    }
  }
  ASSERT_EQ(obj.getMaxSize(), max_size);
}

TEST_F(AveragingFilterTest, checkFIFOcontent)
{
  unsigned int max_size = 5;
  AveragingFilter obj;
  obj.setMaxSize(max_size);
  for (unsigned int i = 1; i <= 2 * max_size; ++i)
  {
    obj.acquire(static_cast<double>(i));
  }
  ASSERT_EQ(obj.getFilteredData(), 8.0);
  ASSERT_EQ(obj.getBuffer()[0], 6.0);
  ASSERT_EQ(obj.getBuffer()[1], 7.0);
  ASSERT_EQ(obj.getBuffer()[2], 8.0);
  ASSERT_EQ(obj.getBuffer()[3], 9.0);
  ASSERT_EQ(obj.getBuffer()[4], 10.0);
}