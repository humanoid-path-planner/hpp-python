{
  description = "python bindings for HPP, based on boost python";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/refs/pull/362956/head";
    # we need https://github.com/humanoid-path-planner/hpp-pinocchio/pull/223
    hpp-core = {
      url = "github:humanoid-path-planner/hpp-core/devel";
      inputs.flake-parts.follows = "flake-parts";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    hpp-pinocchio = {
      url = "github:humanoid-path-planner/hpp-pinocchio/devel";
      inputs.flake-parts.follows = "flake-parts";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        {
          pkgs,
          self',
          system,
          ...
        }:
        {
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            overlays = [
              (_super: prev: {
                hpp-core = prev.hpp-core.overrideAttrs {
                  inherit (inputs.hpp-core.packages.${system}.hpp-core) src;
                };
                hpp-pinocchio = prev.hpp-pinocchio.overrideAttrs {
                  inherit (inputs.hpp-pinocchio.packages.${system}.hpp-pinocchio) src;
                };
              })
            ];
          };
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
          packages = {
            default = self'.packages.hpp-python;
            hpp-python = pkgs.python3Packages.hpp-python.overrideAttrs {
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./include
                  ./package.xml
                  ./src
                  ./tests
                ];
              };
            };
          };
        };
    };
}
