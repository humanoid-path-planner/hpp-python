{
  description = "python bindings for HPP, based on boost python";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/refs/pull/362956/head";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        { pkgs, self', ... }:
        {
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
