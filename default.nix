{
  lib,
  stdenv,
  cmake,
  fmt,
  python3Packages,
  rosPackages,
  linear-feedback-controller-msgs
}:
stdenv.mkDerivation {
  pname = "linear-feedback-controller";
  version = "1.0.2";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./cmake
      ./CMakeLists.txt
      ./config
      ./controller_plugins.xml
      ./include
      ./launch
      ./LICENSE
      ./package.xml
      ./src
      ./tests
    ];
  };

  nativeBuildInputs = [
    cmake
    fmt
    python3Packages.python
    rosPackages.jazzy.ament-cmake
    rosPackages.jazzy.ament-cmake-auto
    rosPackages.jazzy.ament-lint-auto
    rosPackages.jazzy.eigen3-cmake-module # this is a mistake on jazzy
    rosPackages.jazzy.generate-parameter-library-py
    rosPackages.jazzy.pluginlib
  ];

  propagatedBuildInputs = [
    fmt
    linear-feedback-controller-msgs
    python3Packages.pinocchio
    python3Packages.example-robot-data
    rosPackages.jazzy.control-toolbox
    rosPackages.jazzy.controller-interface
    rosPackages.jazzy.nav-msgs
    rosPackages.jazzy.pal-statistics
    rosPackages.jazzy.parameter-traits
    rosPackages.jazzy.realtime-tools
    rosPackages.jazzy.rclcpp-lifecycle
  ];

  # revert https://github.com/lopsided98/nix-ros-overlay/blob/develop/distros/rosidl-generator-py-setup-hook.sh
  # as they break tests
  postConfigure = ''
    cmake $cmakeDir -DCMAKE_SKIP_BUILD_RPATH:BOOL=OFF
  '';

  doCheck = true;

  # generate_parameter_library_markdown complains that build/doc exists
  # ref. https://github.com/PickNikRobotics/generate_parameter_library/pull/212
  enableParallelBuilding = false;

  # some dependency started to leak qtPreHook
  dontWrapQtApps = true;

  meta = {
    description = "RosControl linear feedback controller with pal base estimator and RosTopics external interface.";
    homepage = "https://github.com/loco-3d/linear-feedback-controller";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
    platforms = lib.platforms.linux;
  };
}
