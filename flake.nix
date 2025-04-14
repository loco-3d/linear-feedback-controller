{
  description = "RosControl linear feedback controller with pal base estimator and RosTopics external interface.";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [ "x86_64-linux" ];
      imports = [ inputs.treefmt-nix.flakeModule ];
      perSystem =
        {
          lib,
          pkgs,
          system,
          self',
          ...
        }:
        {
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            overlays = [
              inputs.nix-ros-overlay.overlays.default
              inputs.gepetto.overlays.default
            ];
          };
          checks = lib.mapAttrs' (n: lib.nameValuePair "package-${n}") self'.packages;
          packages = {
            default = self'.packages.humble-linear-feedback-controller;
            humble-linear-feedback-controller = pkgs.rosPackages.humble.linear-feedback-controller.overrideAttrs {
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
            jazzy-linear-feedback-controller = pkgs.rosPackages.jazzy.linear-feedback-controller.overrideAttrs {
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
          treefmt.programs = {
            deadnix.enable = true;
            nixfmt.enable = true;
          };
        };
    };
}
