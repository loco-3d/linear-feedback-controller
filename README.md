[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/loco-3d/linear-feedback-controller/main.svg)](https://results.pre-commit.ci/latest/github/loco-3d/linear-feedback-controller/main)

# linear feedback controller

This repository is aimed to be used with its public API the
[linear_feedback_controller_msgs](https://github.com/loco-3d/linear-feedback-controller-msgs).

These two package are under the license [BSD-2 license](./LICENSE).

The current versions of this 2 package can be found in their respective `package.xml` file.

Follows a quick description of these package.

## The linear_feedback_controller

In this package we implement a [ros2_control](https://control.ros.org/rolling/index.html)
controller.
It is implementing a chainable controller pluggable with any kind of state
estimator.

We implement a ROS 2 action client/server setup.
We send a request with:
- base configuration (only if the robot has a free-flyer, identity otherwise)
- base velocity (only if the robot has a free-flyer, identity otherwise)
- joint positions
- joint velocities
- joint efforts (torques applied to the joints)

We send receive the response in the shape:
- A feedback gain matrix
- A feedforward term in torque
- The state which was used to linearize the control.

This allows us, for example, to use it on the Talos (1 and 3) robots with a remote controller
using a whole body model predictive control based on [croccodyl](https://github.com/loco-3d/crocoddyl)

## The linear_feedback_controller_msgs

[This package](https://github.com/loco-3d/linear-feedback-controller-msgs) contains the external user interface to the linear_feedback_controller
package. It describes the sensor data exchanged in the previously cited RosTopics.

And in particular it offers a very simple ROS/[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
conversion. This is made to facilitate further computations with the Sensor
data. And ease to fill in the Control message.

Please check the [README.md](https://github.com/loco-3d/linear-feedback-controller-msgs/blob/main/README.md) of the package for more details.

## Example of usage

### Requirements

For this example you need to have access to the dockers in https://gitlab.laas.fr/gsaurel/docker-pal/.
This is a private repository so one need to be part of the LAAS-CNRS french laboratory.
Further development will provide a new public image in order to use this code in simulation.

### Build the repository using ROS and docker.

First of all build or pull the docker from https://gitlab.laas.fr/gsaurel/docker-pal/
using the branch `gsaurel`.

```
docker pull gitlab.laas.fr:4567/gsaurel/docker-pal:gsaurel
```

Then run the docker and mount (`-v` docker option) the `src` and `build` folder
of your workspace in order to get some cache between to run of the container:

```
	chown -R :gepetto $(YOUR_WS)  # you might need sudo if you get errors here
	chmod -R g+rwX  $(YOUR_WS)    # you might need sudo if you get errors here
	xhost +local:
	docker run --rm -v $(YOUR_WS)/src:/ws/src -v $(PWD)/Makefile:/ws/Makefile -v /home/$(USER)/devel:/home/user/devel -v $(YOUR_WS)/build:/ws/build --gpus all --net host -e DISPLAY -it gitlab.laas.fr:4567/gsaurel/docker-pal:gsaurel
```

If you need another terminal to connect to the last docker container ran you can use:
```
docker exec --workdir=/ws -u user -it `docker ps --latest --quiet` bash
```

Once connected to docker with multiple terminal (5) please run in order:

- Terminal 1: Start the simulation:
```
reset && catkin build linear_feedback_controller && source install/setup.bash && roslaunch talos_pal_physics_simulator talos_pal_physics_simulator_with_actuators.launch robot:=full_v2
```

- Terminal 2: Spawn the default controller:
```
reset && source install/setup.bash && roslaunch talos_controller_configuration default_controllers.launch
```
in the same terminal, once the robot is in the default pose, kill (`ctrl+C`) the roslaunch

- Terminal 3: Spawn the linear feedback controller:
```
reset && source install/setup.bash && roslaunch linear_feedback_controller talos_linear_feedback_controller.launch simulation:=true default_params:=true
```

- Terminal 4: For recording logs you can use `rosbag`:
```
reset && source install/setup.bash && rosbag record -o src/lfc /linear_feedback_controller/sensor_state /linear_feedback_controller/desired_control
```

- Terminal 5: For starting a very simple PD controller via Feedback gains run:
```
reset && source install/setup.bash && rosrun linear_feedback_controller pd_controller
```

The next paragraph is the same procedure except one can use the Makefile that is
in the root of this repos to do so.

### Build the repository using make

There is also [Makefile](Makefile) to ease the usage of docker and ROS instructions
in order to execute the demo.

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

- Run the default controller and kill it (ctrl+C) when the robot is in half-sitting terminal(2)
```
make docker_connect
make default_ctrl
```

- Run the linear feedback controller terminal (2)
```
make lf_ctrl
```

- Run the PD controller using the lfc terminal (3)
```
make docker_connect
make pd_ctrl
```

### Copyrights and License

See the BSD-2 LICENSE file.
For the main authors see the github repository for the main recent contributors.

Here is the list of the historical main authors:
- Côme Perrot
- Maximilien Naveau

Main maintainers
- Guilhem Saurel (gsaurel@laas.fr)
- Maximilien Naveau (maximilien.naveau@gmail.com)
