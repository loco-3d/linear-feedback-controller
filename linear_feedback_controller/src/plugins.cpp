#include "linear_feedback_controller/linear_feedback_controller.hpp"

// export this lib as plugin
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(linear_feedback_controller::LinearFeedbackController,
                       controller_interface::ControllerBase)
