{
  description = "ROS2 execution utilities for HPP-generated trajectories";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        pyOverrideAttrs.hpp-exec = {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions [
              ./CMakeLists.txt
              ./doc
              ./hpp_exec
              ./include
              ./package.xml
              ./robots
              ./tests
            ];
          };
        };
      }
    );
}
