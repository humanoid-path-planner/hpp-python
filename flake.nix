{
  description = "python bindings for HPP, based on boost python";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    gazebros2nix.follows = "gepetto/gazebros2nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
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
            gazebros2nix.pyOverrides.hpp-python =
              _final: python-final:
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
          }
        ];
      }
    );
}
