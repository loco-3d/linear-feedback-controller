{
  description = "RosControl linear feedback controller with pal base estimator and RosTopics external interface.";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosOverrideAttrs.linear-feedback-controller = {
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

        # TEMPORARY: pin to the fork branch carrying Control.next_states
        # (loco-3d/linear-feedback-controller-msgs#81, not merged upstream
        # yet) -- remove this override once that PR is merged, CI will then
        # go back to the nixpkgs-released version automatically.
        rosOverrideAttrs.linear-feedback-controller-msgs = {
          src = builtins.fetchGit {
            url = "https://github.com/clementPene/linear-feedback-controller-msgs.git";
            ref = "feat/control-next-state";
            rev = "4ba955feed438d2b8a0b4a9afac6ba490279a97f";
          };
        };
      }
    );
}
