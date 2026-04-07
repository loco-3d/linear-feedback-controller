{
  description = "RosControl linear feedback controller with pal base estimator and RosTopics external interface.";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    systems.follows = "gepetto/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          {
            flakoboros.rosOverrideAttrs.linear-feedback-controller = _: _: {
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
        ];
      }
    );
}
