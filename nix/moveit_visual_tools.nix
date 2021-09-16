{ pkgs }:
    let
        moveit_pkgs = import ./moveit_package.nix {pkgs=pkgs;};
    in
        with pkgs; with rosPackages.noetic; buildRosPackage {
          pname = "ros-noetic-moveit-visual-tools";
          version = "3.6.0-r1";

          src = builtins.fetchGit {
            url = "git@github.com:ros-planning/moveit_visual_tools.git";
            rev = "91cecd2ea23611d0f6be85a1085aa9ce634efc69";
          };

          buildType = "catkin";
          propagatedBuildInputs = [
                cmake-modules
                geometry-msgs
                graph-msgs
                moveit_pkgs.moveit_core
                moveit_pkgs.moveit_ros_planning
                roscpp
                roslint
                rviz-visual-tools
                std-msgs
                tf2-eigen
                tf2-ros
                trajectory-msgs
                visualization-msgs
          ];
          nativeBuildInputs = [ catkin ];

          meta = {
            description = ''Helper functions for displaying and debugging MoveIt data in Rviz via published markers'';
            license = with lib.licenses; [ bsdOriginal ];
          };
        }