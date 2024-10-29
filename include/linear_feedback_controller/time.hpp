#include <chrono>

namespace linear_feedback_controller {
using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock,
                                          std::chrono::duration<double> >;

using Duration = std::chrono::duration<double>;
}  // namespace linear_feedback_controller
