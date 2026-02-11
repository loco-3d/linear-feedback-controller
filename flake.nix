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
      { lib, self, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gazebros2nix.flakeModule
          { gazebros2nix-pkgs.overlays = [ self.overlays.default ]; }
        ];
        flake.overlays.default =
          _final: prev:
          let
            scope = _ros-final: ros-prev: {
              linear-feedback-controller = ros-prev.linear-feedback-controller.overrideAttrs {
                postPatch = "";
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
          in
          {
            rosPackages = prev.rosPackages // {
              humble = prev.rosPackages.humble.overrideScope scope;
              jazzy = prev.rosPackages.jazzy.overrideScope scope;
              kilted = prev.rosPackages.kilted.overrideScope scope;
              rolling = prev.rosPackages.rolling.overrideScope scope;
            };
          };
        perSystem =
          { pkgs, ... }:
          {
            packages = lib.filterAttrs (_n: v: v.meta.available && !v.meta.broken) (rec {
              default = rolling-linear-feedback-controller;
              humble-linear-feedback-controller = pkgs.rosPackages.humble.linear-feedback-controller;
              jazzy-linear-feedback-controller = pkgs.rosPackages.jazzy.linear-feedback-controller;
              kilted-linear-feedback-controller = pkgs.rosPackages.kilted.linear-feedback-controller;
              rolling-linear-feedback-controller = pkgs.rosPackages.rolling.linear-feedback-controller;
            });
          };
      }
    );
}
