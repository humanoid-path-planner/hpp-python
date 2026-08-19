{
  description = "python bindings for HPP, based on boost python";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    hpp-core = {
      url = "github:humanoid-path-planner/hpp-core";
      inputs.gepetto.follows = "gepetto";
    };
  };

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overlays = [ inputs.hpp-core.overlays.flakoboros ];
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
