[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/loco-3d/linear-feedback-controller/main.svg)](https://results.pre-commit.ci/latest/github/loco-3d/linear-feedback-controller/main)

# linear-feedback-controller

A [ros2_control](https://control.ros.org/rolling/index.html) package implementing linear feedback control strategies for robotic systems. It supports both fixed-base and free-flying robot configurations and is designed to integrate with external model predictive controllers.

The public message interface is defined in [linear_feedback_controller_msgs](https://github.com/loco-3d/linear-feedback-controller-msgs).

Both packages are released under the [BSD-2 license](./LICENSE). Current versions are tracked in their respective `package.xml` files and must have an associated git tag of the form `vX.Y.Z`.

Full documentation is available on [DeepWiki](https://deepwiki.com/loco-3d/linear-feedback-controller).

## Controllers

The package provides three `ros2_control` controller plugins:

| Controller | Base class | Role |
|---|---|---|
| `LinearFeedbackControllerRos` | `ChainableControllerInterface` | Main control loop: PD → linear feedback transition with external MPC |
| `JointStateEstimator` | `ControllerInterface` | Joint state filtering and velocity estimation |
| `PassthroughController` | `ChainableControllerInterface` | Interface renaming / forwarding for hardware–simulation parity |

### LinearFeedbackControllerRos

The main controller implements a two-phase control strategy:

1. **PD phase** — Proportional-derivative control for initial stabilization using configurable per-joint gains (`p_gains`, `d_gains`).
2. **LF phase** — Linear feedback control driven by an external controller (e.g. a whole-body MPC based on [Crocoddyl](https://github.com/loco-3d/crocoddyl)).

The transition between phases is time-based and configurable via `pd_to_lf_transition_duration`.

The controller **publishes** sensor data on a ROS 2 topic:
- Base pose and velocity (free-flying robots; identity for fixed-base robots)
- Joint positions, velocities, and efforts

It **receives** control commands containing:
- A feedback gain matrix
- A feedforward torque term
- The linearization state

### JointStateEstimator

Reads hardware state interfaces, applies exponential smoothing to joint velocities, and writes the filtered values to command interfaces. This makes raw sensor data suitable for downstream chainable controllers.

### PassthroughController

Forwards and renames reference interfaces to command interfaces. Useful when simulation and hardware use different interface names but share the same parameter file.

## Dependencies

| Package | Role |
|---|---|
| [ros2_control](https://control.ros.org/rolling/index.html) | Controller framework and hardware abstraction |
| [pinocchio](https://github.com/stack-of-tasks/pinocchio) | Rigid body dynamics and URDF model loading |
| [Eigen3](https://eigen.tuxfamily.org/) | Linear algebra |
| [linear_feedback_controller_msgs](https://github.com/loco-3d/linear-feedback-controller-msgs) | ROS topic message definitions and Eigen/NumPy conversions |

## Build

### With colcon (recommended)

```bash
mkdir -p workspace/src
git -C workspace/src clone https://github.com/loco-3d/linear-feedback-controller.git
colcon build
```

### With CMake directly

```bash
git clone https://github.com/loco-3d/linear-feedback-controller.git
cd linear-feedback-controller
mkdir _build && cd _build
cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/your/install/folder
make install
```

### With Nix

```bash
git clone https://github.com/loco-3d/linear-feedback-controller.git
cd linear-feedback-controller

# Check (build + tests)
nix flake check -L

# Build only
nix build -L

# Enter development shell
nix develop
```

[nix-direnv](https://github.com/nix-community/nix-direnv/) is also supported for automatic shell activation.

## Configuration

An example YAML configuration for the Tiago Pro robot is available at [`config/tiago_pro_lfc_params.yaml`](./config/tiago_pro_lfc_params.yaml).

Real-world usage can be found in the [agimus-demos](https://github.com/agimus-project/agimus-demos) repository, in particular in the `agimus_demos_common` package.

## License and Authors

BSD-2 — see [LICENSE](./LICENSE).

Historical main authors:
- Côme Perrot
- Maximilien Naveau
- Arthur Valiente

Current maintainers:
- Guilhem Saurel (gsaurel@laas.fr)
- Maximilien Naveau (maximilien.naveau@gmail.com)
- Clément Pène (clement.pene@laas.fr)
