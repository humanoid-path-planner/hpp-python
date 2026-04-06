{
  description = "python bindings for HPP, based on boost python";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";

    # https://github.com/humanoid-path-planner/hpp-manipulation/pull/262
    hpp-manipulation.url = "github:humanoid-path-planner/hpp-manipulation";
    hpp-manipulation.inputs.gepetto.follows = "gepetto";

    # https://github.com/humanoid-path-planner/hpp-core/pull/429
    hpp-core.url = "github:humanoid-path-planner/hpp-core";
    hpp-core.inputs.gepetto.follows = "gepetto";
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
            flakoboros = {
              overlays = [
                inputs.hpp-core.overlays.flakoboros
                inputs.hpp-manipulation.overlays.flakoboros
              ];
              pyOverrideAttrs.hpp-python =
                _: python-final:
                (super: {
                  buildInputs = [ python-final.boost ] ++ super.buildInputs;
                  propagatedBuildInputs = super.propagatedBuildInputs ++ [
                    python-final.lxml
                  ];
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
                });
            };
          }
        ];
      }
    );
}
