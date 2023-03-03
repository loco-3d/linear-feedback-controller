## linear_feedback_controller

ros controller based on the pal_base_ros_controller package.

### Test

- start docker
- `catkin config -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DCATKIN_ENABLE_TESTING=ON --install`
- `reset && catkin build linear_feedback_controller && catkin run_tests --verbose linear_feedback_controller`
