#ifndef LINEAR_FEEDBACK_CONTROLLER_AVERAGING_FILTER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_AVERAGING_FILTER_HPP

#include <deque>

namespace linear_feedback_controller {

template <class DataType>
class AveragingFilter {
 public:
  /**
   * @brief Construct a new Averaging Filter object
   */
  AveragingFilter();

  /**
   * @brief Destroy the Averaging Filter object
   */
  ~AveragingFilter() {};

  /**
   * @brief Add another data in the buffer in a FIFO manner;
   *
   * @param data
   */
  void acquire(DataType data);

  /**
   * @brief Set the buffer max size.
   *
   * @param size
   */
  void set_max_size(int size) { max_size_ = size; }

  /**
   * @brief Get the buffer max size.
   *
   * @return int
   */
  const std::size_t& get_max_size() const { return max_size_; }

  /**
   * @brief Get the buffer.
   *
   * @return int
   */
  const std::deque<DataType>& get_buffer() const { return buffer_; }

  /**
   * @brief Get the filtered data. In case the buffer is not initialize return
   * the last acquired data.
   *
   * @return DataType
   */
  DataType get_filtered_data();

 private:
  /// @brief Inside buffer;
  std::deque<DataType> buffer_;

  /// @brief Sum of the buffer data.
  DataType sum_buffer_;

  /// @brief Maximum buffer size;
  std::size_t max_size_;

  /// @brief Last acquired data;
  DataType last_data_;
};
}  // namespace linear_feedback_controller

#include "linear_feedback_controller/averaging_filter.hxx"

#endif  // LINEAR_FEEDBACK_CONTROLLER_AVERAGING_FILTER_HPP
