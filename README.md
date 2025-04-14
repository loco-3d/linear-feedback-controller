[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/loco-3d/linear-feedback-controller/main.svg)](https://results.pre-commit.ci/latest/github/loco-3d/linear-feedback-controller/main)

# linear feedback controller

This repository is aimed to be used with its public API the
[linear_feedback_controller_msgs](https://github.com/loco-3d/linear-feedback-controller-msgs).

These two packages are under the license [BSD-2 license](./LICENSE).

The current versions of these two packages can be found in their respective
`package.xml` file. There **must** be an associated git tag in a shape of vX.Y.Z.

Follows a quick description of these package.

## The linear_feedback_controller

In this package we implement a [ros2_control](https://control.ros.org/rolling/index.html)
controller.
It is implementing a chainable controller pluggable with any kind of state
estimator.

We implement a ROS 2 topic publisher sending:
- base configuration (only if the robot has a free-flyer, identity otherwise)
- base velocity (only if the robot has a free-flyer, identity otherwise)
- joint positions
- joint velocities
- joint efforts (torques applied to the joints)

We send receive the response via a topic in the shape:
- A feedback gain matrix
- A feedforward term in torque
- The state which was sent before used to linearize the control.

This allows us, for example, to use it on the Talos (1 and 3) robots with a remote controller
using a whole body model predictive control based on [croccodyl](https://github.com/loco-3d/crocoddyl)

## The linear_feedback_controller_msgs

[This package](https://github.com/loco-3d/linear-feedback-controller-msgs) contains the external user interface to the linear_feedback_controller
package. It describes the sensor data exchanged in the previously cited ROS2 topics.

And in particular it offers a very simple ROS/[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
conversion tooling.
And a ROS/[numpy](https://numpy.org/) conversion tooling.
These are made to facilitate further computations with the Sensor data,
and ease to fill in the Control message.

Please check the [README.md](https://github.com/loco-3d/linear-feedback-controller-msgs/blob/main/README.md) of the package for more details.

## Example of usage

This a ROS2 controller so on can simply look at the ROS2 control documentation.
An example of configuration can be found
[in this repository here](./config/tiago_pro_lfc_params.yaml).

The example is extracted from the agimus-demos pacakges:
https://github.com/agimus-project/agimus-demos

And in particular the setup of the LFC is in the `agimus_demos_common` package.

### Build the package.

This package is base on ament_cmake hence one can simply use the standard:

```bash
git clone --recursive git@github.com:loco-3d/linear-feedback-controller.git
cd linear-feedback-controller
make _build
cd _build
cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/your/install/folder
make
make install
```

Or one can use the ROS2 super build system [colcon](https://colcon.readthedocs.io/en/released/):

```bash
mkdir -p workspace/src
cd workspace/src
git clone --recursive git@github.com:loco-3d/linear-feedback-controller.git
cd ../../ # back to workspace.
colcon build
```

One can also use [nix](https://nixos.wiki/wiki/Main_Page) to:
- Check the package (builds and run tests):
```bash
git clone --recursive git@github.com:loco-3d/linear-feedback-controller.git
cd linear-feedback-controller
nix
```
- Build the package:
```bash
git clone --recursive git@github.com:loco-3d/linear-feedback-controller.git
cd linear-feedback-controller
nix build -L
```
- Create a shell in which the LFC can be built:
```bash
git clone --recursive git@github.com:loco-3d/linear-feedback-controller.git
cd linear-feedback-controller
nix develop
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
