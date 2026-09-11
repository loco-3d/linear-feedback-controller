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
      }
    );
}
