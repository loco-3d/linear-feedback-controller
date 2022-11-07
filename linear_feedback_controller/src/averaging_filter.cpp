#include <vector>
#include <numeric>
#include "linear_feedback_controller/averaging_filter.hpp"

namespace linear_feedback_controller {
AveragingFilter::AveragingFilter() {
  max_size_ = 0;
  last_data_ = 0.0;
}

void AveragingFilter::acquire(double data) {
  last_data_ = data;
  buffer_.push_back(data);
  if (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }
}

double AveragingFilter::getFilteredData() {
  if (buffer_.empty()) {
    return last_data_;
  }
  auto const count = static_cast<float>(buffer_.size());
  return std::accumulate(buffer_.begin(), buffer_.end(), decltype(buffer_)::value_type(0)) / count;
}
}  // namespace linear_feedback_controller