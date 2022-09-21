# ROS_WBMPC

ROS interface for the Whole Body Model Predictive Controller

contents of the packages:
- ros_server_wbmpc (optional)
    - eigen interface to ros_server_wbmpc_msgs
    - easier usage of TF (mocap)
    - logger (PAL?)
- ros_server_wbmpc_msgs (very practical)
    - input msg
    - output msg
- roscontrol_pal_wbmpc (mandatory for safe robot usage!!!!)
    - math of the low level for talos (with parametrization).
    - usage of ros_server_wbmpc_msgs as external interface.
    - parametrization of the controlled(torque)/fixed(position) joints.
