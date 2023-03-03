[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/loco-3d/linear-feedback-controller/main.svg)](https://results.pre-commit.ci/latest/github/loco-3d/linear-feedback-controller/main)

# linear feedback controller

This repository contains two packages:
- the [linear_feedback_controller](./linear_feedback_controller/README.md)
- the [linear_feedback_controller_msgs](./linear_feedback_controller_msgs/README.md)

These two package are under the license [BSD-2 license](./LICENSE).

Follows a quick description of these package.

## The linear_feedback_controller

In this package we implement a [roscontrol](http://wiki.ros.org/ros_control)
controller. It is based on the [pal_base_ros_controller](https://gitlab.com/pal-robotics/LAAS/pal_base_ros_controller_tutorials)
package.

We implement a RosTopic publisher that sends the state of the robot:
- base configuration
- base velocity
- joint positions
- joint velocities
- joint efforts (torques applied to the joints)

We then subscribe to a RosTopic that sends us controls for the robot:
- A feedback gain matrix
- A feedforward term in torque
- The state which was used to linearize the control.

This allows us, for example,to use it on the Talos (1 and 3) robots with a remote controller
using a whole body model predictive control based on [croccodyl](https://github.com/loco-3d/crocoddyl)

Please check the [README.md](./linear_feedback_controller/README.md) of the package for more details.

## The linear_feedback_controller_msgs

This package contains the external user interface to the linear_feedback_controller
package. It describes the sensor data exchanged in the previously cited RosTopics.

And in particular it offers a very simple ROS/[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
conversion. This is made to facilitate further computations with the Sensor
data. And ease to fill in the Control message.

Please check the [README.md](./linear_feedback_controller_msgs/README.md) of the package for more details.

## Example of usage

### Requirements

For this example you need to have access to the dockers in https://gitlab.laas.fr/gsaurel/docker-pal/.

### Build the repository

- Create the workspace and clone the repository
```
mkdir -p ~/devel/workspace/src
cd ~/devel/workspace/src
git clone --recursive git@github.com:loco-3d/linear-feedback-controller.git
```

- Run the docker and build in terminal(1)
```
cd ~/devel/workspace/src/linear_feedback_controller
make docker_pull_and_run
make build
```

- Run the simulation, terminal(1)
```
make simu
```

- Run the default controller and kill it when the robot is in haflsitting terminal(2)
```
make default_ctrl
```

- Run the linear feedback controller terminal (2)
```
make lf_ctrl
```

- Run the PD controller using the lfc terminal (3)
```
make pd_ctrl
```
