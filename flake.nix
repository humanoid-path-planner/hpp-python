{
  description = "python bindings for HPP, based on boost python";

  inputs = {
    gepetto.url = "github:gepetto/nix";

    # https://github.com/humanoid-path-planner/hpp-core/pull/434 is required
    hpp-core.url = "github:humanoid-path-planner/hpp-core";
    hpp-core.inputs.gepetto.follows = "gepetto";

    # https://github.com/humanoid-path-planner/hpp-constraints/pull/282 is required
    hpp-constraints.url = "github:humanoid-path-planner/hpp-constraints";
    hpp-constraints.inputs.gepetto.follows = "gepetto";
  };

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overlays = [
          inputs.hpp-core.overlays.flakoboros
          inputs.hpp-constraints.overlays.flakoboros
        ];
        pyOverrideAttrs.hpp-python = {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions [
              ./CMakeLists.txt
              ./doc
              ./include
              ./package.xml
              ./src
              ./tests
            ];
          };
        };
      }
    );
}
