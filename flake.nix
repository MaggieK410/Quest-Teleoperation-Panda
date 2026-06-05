{
  description = "Teleoperation on the Panda Robot with Quest 3";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosDistros = [ "humble" ];
        rosShellDistro = "humble";
        rosOverrideAttrs.quest-control = {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions [
              ./quest_control
            ];
          };
        };
      }
    );
}
