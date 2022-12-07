[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/loco-3d/linear-feedback-controller/main.svg)](https://results.pre-commit.ci/latest/github/loco-3d/linear-feedback-controller/main)

# ROS_WBMPC

ROS interface for the Whole Body Model Predictive Controller

contents of the packages:
- ros-wbmpc-robot-interface (optional)
    - eigen interface to ros_server_wbmpc_msgs
    - easier usage of TF (mocap)
    - logger (PAL?)
    - a place to put the crocoddyl code, object usage or inheritance.
- ros-wbmpc-robot (optional)
    - math to parsing the ros msgs from the server and convert the Ricatti gains into torques ref.
- ros_server_wbmpc_msgs (very practical in terms of ros infrastructure)
    - input msg
    - output msg
- roscontrol_pal_wbmpc (mandatory for safe robot usage!!!!)
    - math of the low level for talos (with parametrization)
        - natively using ros_server_wbmpc_msgs as external interface **OR**
        - using the ros_server_wbmpc_client
    - parametrization of the controlled(torque)/fixed(position) joints.
