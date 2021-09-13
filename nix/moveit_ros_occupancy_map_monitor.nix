{ pkgs }:
with pkgs; with rosPackages.noetic; buildRosPackage {
  pname = "ros-noetic-moveit-occupancy-map_monitor";
  version = "custom-master";

  src = /home/werner/catkin_ws/src/moveit;

  buildType = "catkin";
  propagatedBuildInputs = [ dynamic-reconfigure
        (import ./moveit_core.nix {pkgs=pkgs;})
        catkin
        eigen
        geometric-shapes
        moveit-msgs
        octomap
        ompl
        pluginlib
        pluginlib
        rosconsole
        roscpp
        rosunit
        tf2
        tf2-ros
        ];
  nativeBuildInputs = [ catkin ];

  configurePhase = ''
              cmake moveit_ros/occupancy_map_monitor -DCMAKE_INSTALL_PREFIX=$out
          '';

  meta = {
    description = ''MoveIt interface to OMPL'';
    license = with lib.licenses; [ bsdOriginal ];
  };
}