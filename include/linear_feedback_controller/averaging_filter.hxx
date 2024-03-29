#ifndef LINEAR_FEEDBACK_CONTROLLER_AVERAGING_FILTER_HXX
#define LINEAR_FEEDBACK_CONTROLLER_AVERAGING_FILTER_HXX

#include <Eigen/Core>
#include <numeric>
#include <vector>

#include "linear_feedback_controller/averaging_filter.hpp"

namespace linear_feedback_controller {

template <class DataType>
AveragingFilter<DataType>::AveragingFilter() {
  max_size_ = 0;
  last_data_ = 0.0;
  sum_buffer_ = 0.0;
  buffer_.clear();
}

template <>
AveragingFilter<Eigen::VectorXd>::AveragingFilter() {
  max_size_ = 0;
  last_data_ = Eigen::VectorXd::Zero(0);
  buffer_.clear();
}

template <class DataType>
void AveragingFilter<DataType>::acquire(DataType data) {
  last_data_ = data;
  buffer_.push_back(data);
  if (buffer_.size() == 1) {
    sum_buffer_ = last_data_;
  } else {
    sum_buffer_ += last_data_;
  }
  if (buffer_.size() > max_size_) {
    sum_buffer_ -= buffer_.front();
    buffer_.pop_front();
  }
}

template <class DataType>
DataType AveragingFilter<DataType>::getFilteredData() {
  if (buffer_.empty()) {
    return last_data_;
  }
  auto const count = static_cast<double>(buffer_.size());
  return sum_buffer_ / count;
}
}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_AVERAGING_FILTER_HXX
