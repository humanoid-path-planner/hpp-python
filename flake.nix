{
  description = "python bindings for HPP, based on boost python";

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
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import inputs.systems;
      imports = [ inputs.gepetto.flakeModule ];
      perSystem =
        {
          lib,
          pkgs,
          self',
          ...
        }:
        {
          apps = lib.filterAttrs (_n: v: v.program.meta.available && !v.program.meta.broken) {
            default = {
              type = "app";
              program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
            };
          };
          packages = {
            default = self'.packages.hpp-python;
            hpp-python = pkgs.python3Packages.hpp-python.overrideAttrs (super: {
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
              propagatedBuildInputs = (super.propagatedBuildInputs or [ ]) ++ [
                pkgs.python3Packages.hpp-tutorial
              ];
            });
          };
        };
    };
}
