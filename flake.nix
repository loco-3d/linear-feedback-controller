{
  description = "RosControl linear feedback controller with pal base estimator and RosTopics external interface.";

  inputs = {
    gazebros2nix.url = "github:gepetto/gazebros2nix";
    flake-parts.follows = "gazebros2nix/flake-parts";
    nixpkgs.follows = "gazebros2nix/nixpkgs";
    nix-ros-overlay.follows = "gazebros2nix/nix-ros-overlay";
    systems.follows = "gazebros2nix/systems";
    treefmt-nix.follows = "gazebros2nix/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gazebros2nix.flakeModule
          {
            gazebros2nix.rosPackages = {
              linear-feedback-controller = _final: _ros-final: {
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
              };
            };
          }
        ];
        perSystem =
          { self', ... }:
          {
            packages.default = self'.packages.ros-rolling-linear-feedback-controller;
          };
      }
    );
}
