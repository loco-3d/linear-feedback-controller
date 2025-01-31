#ifndef LINEAR_FEEDBACK_CONTROLLER__TIME_HPP_
#define LINEAR_FEEDBACK_CONTROLLER__TIME_HPP_

#include <chrono>

namespace linear_feedback_controller {
using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock,
                                          std::chrono::duration<double> >;

using Duration = std::chrono::duration<double>;
}  // namespace linear_feedback_controller

#endif
