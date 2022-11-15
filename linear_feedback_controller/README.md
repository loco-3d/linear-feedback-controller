## linear_feedback_controller

ros controller based on the pal_base_ros_controller package.

### Test

- start docker
- `reset && catkin build linear_feedback_controller -DCMAKE_BUILD_TYPE=Debug && catkin test linear_feedback_controller --verbose`
- `reset && catkin build linear_feedback_controller -DCMAKE_BUILD_TYPE=Debug && catkin run_tests --force-color --verbose linear_feedback_controller`