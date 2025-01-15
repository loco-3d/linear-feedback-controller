{
  description = "RosControl linear feedback controller with pal base estimator and RosTopics external interface.";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    linear-feedback-controller-msgs = {
      url = "github:loco-3d/linear-feedback-controller-msgs/humble-devel";
      inputs.nix-ros-overlay.follows = "nix-ros-overlay";
    };
  };

  outputs =
    { linear-feedback-controller-msgs, nix-ros-overlay, self, ... }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nix-ros-overlay.inputs.nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in
      {
        packages = {
          default = self.packages.${system}.linear-feedback-controller;
          linear-feedback-controller = pkgs.callPackage ./default.nix {
            inherit (linear-feedback-controller-msgs.packages.${system}) linear-feedback-controller-msgs;
          };
        };
      }
    );
}
