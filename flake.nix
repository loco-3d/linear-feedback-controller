{
  description = "RosControl linear feedback controller with pal base estimator and RosTopics external interface.";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import inputs.systems;
      imports = [ inputs.gepetto.flakeModule ];
      perSystem =
        { lib, pkgs, ... }:
        {
          packages =
            let
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
            in
            lib.filterAttrs (_n: v: v.meta.available && !v.meta.broken) (rec {
              inherit (pkgs.rosPackages.humble) gtest-vendor ament-cmake-gtest;
              inherit (pkgs) gtest;
              default = humble-linear-feedback-controller;
              humble-linear-feedback-controller =
                pkgs.rosPackages.humble.linear-feedback-controller.overrideAttrs
                  (super: {
                    inherit src;
                    nativeBuildInputs = (super.nativeBuildInputs or [ ]) ++ [
                      pkgs.jrl-cmakemodules
                      pkgs.kcov
                    ];
                    checkInputs = (super.checkInputs or [ ]) ++ [ pkgs.gtest ];
                  });
              jazzy-linear-feedback-controller =
                pkgs.rosPackages.jazzy.linear-feedback-controller.overrideAttrs
                  (super: {
                    inherit src;
                    nativeBuildInputs = (super.nativeBuildInputs or [ ]) ++ [
                      pkgs.jrl-cmakemodules
                      pkgs.kcov
                    ];
                    checkInputs = (super.checkInputs or [ ]) ++ [ pkgs.gtest ];
                  });
            });
        };
    };
}
