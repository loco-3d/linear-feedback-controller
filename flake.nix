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
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, self, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          { gepetto-pkgs.overlays = [ self.overlays.default ]; }
        ];
        flake.overlays.default =
          _final: prev:
          let
            override = _ros-final: ros-prev: {
              linear-feedback-controller = ros-prev.linear-feedback-controller.overrideAttrs {
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
              humble = prev.rosPackages.humble.overrideScope override;
              jazzy = prev.rosPackages.jazzy.overrideScope override;
            };
          };
        perSystem =
          { pkgs, ... }:
          {
            packages = lib.filterAttrs (_n: v: v.meta.available && !v.meta.broken) (rec {
              default = humble-linear-feedback-controller;
              humble-linear-feedback-controller = pkgs.rosPackages.humble.linear-feedback-controller;
              jazzy-linear-feedback-controller = pkgs.rosPackages.jazzy.linear-feedback-controller;
            });
          };
      }
    );
}
