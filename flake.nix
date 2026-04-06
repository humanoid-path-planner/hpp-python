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
              overlays = [ inputs.hpp-manipulation.overlays.default ];
              pyOverrideAttrs.hpp-python =
                _: python-final:
                (super: {
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
