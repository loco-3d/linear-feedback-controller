# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Releases are available on the [github repository](https://github.com/loco-3d/linear-feedback-controller/releases).

## [Unreleased]

## [3.0.0] - 2025-10-23

## added

- A passthrough_controller to simulate controllers on partial interface sets.
  Example in a certain robot there is one controller per joint on the robot
  but not in simulation. This passthrough controller helps the user to create
  a seemless interface for both the real robot and the simulation.

## changed

- Breaking API! The in and out interfaces can now be twicked interface by interface.

## [2.0.0] - 2025-04-14

### Changed

- Breaking change: migration ROS2
- Splitting the ROS vs none ROS part of the code.

### Added

- Added unit-test for the math.

## [1.0.2] - 2023-04-03

### Added

- pal_statistics introspection.
- computation of the ZMP in the controller for debugging
- computation of the position of the pendulum base fro debugging.
- computation of the CoM acceleration from the measured forces.

## [1.0.1] - 2023-03-21

### Fixed

- Fix the averaging filter implementation.

### Changed

- Update the usage of the Eigen API to save some code lines.

## [1.0.0] - 2023-03-17

### Changed

Remove the msgs package and mv the lfc at the root of the git repository.
msgs are now here: https://github.com/loco-3d/linear-feedback-controller-msgs

## [0.1.0] - 2023-03-17

Implementation of linear-feedback-controller in ROS1 for Talos PAL-Robotics robot.

## Git changelogs

[Unreleased]: https://github.com/loco-3d/linear-feedback-controller/compare/v3.0.0...HEAD
[3.0.0]: https://github.com/loco-3d/linear-feedback-controller/compare/v2.0.0...v3.0.0
[2.0.0]: https://github.com/loco-3d/linear-feedback-controller/compare/v1.0.2...v2.0.0
[1.0.2]: https://github.com/loco-3d/linear-feedback-controller/compare/v1.0.1...v1.0.2
[1.0.1]: https://github.com/loco-3d/linear-feedback-controller/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/loco-3d/linear-feedback-controller/compare/v0.1.0...v1.0.0
[0.1.0]: https://github.com/loco-3d/linear-feedback-controller/releases/tag/v0.1.0
