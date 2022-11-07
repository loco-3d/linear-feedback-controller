#ifndef LINEAR_FEEDBACK_CONTROLLER_AVERAGING_FILTER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_AVERAGING_FILTER_HPP

#include <deque>

namespace linear_feedback_controller {

class AveragingFilter {
 public:
  /**
   * @brief Construct a new Averaging Filter object
   */
  AveragingFilter();

  /**
   * @brief Destroy the Averaging Filter object
   */
  ~AveragingFilter(){};

  /**
   * @brief Add another data in the buffer in a FIFO manner;
   *
   * @param data
   */
  void acquire(double data);

  /**
   * @brief Set the buffer max size.
   *
   * @param size
   */
  void setMaxSize(int size) { max_size_ = size; }

  /**
   * @brief Get the buffer max size.
   *
   * @return int
   */
  std::size_t getMaxSize() { return max_size_; }

  /**
   * @brief Get the filtered data. In case the buffer is not initialize return
   * the last acquired data.
   *
   * @return double
   */
  double getFilteredData();

 private:
  /// @brief Inside buffer;
  std::deque<double> buffer_;

  /// @brief Maximum buffer size;
  std::size_t max_size_;

  /// @brief Last acquired data;
  double last_data_;
};
}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_AVERAGING_FILTER_HPP