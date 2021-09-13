{ pkgs }:
with pkgs; with rosPackages.noetic; buildRosPackage {
  pname = "ros-moveit-ros-planning";
  version = "master";

  src = /home/werner/catkin_ws/src/moveit;

  buildType = "catkin";

  propagatedBuildInputs = [
        actionlib
        catkin
        dynamic-reconfigure
        eigen
        message-filters
        (import ./moveit_core.nix {pkgs=pkgs;})
        (import ./moveit_ros_occupancy_map_monitor.nix {pkgs=pkgs;})
        moveit-msgs
        moveit-resources-panda-moveit-config
        pluginlib
        rosconsole
        roscpp
        rostest
        srdfdom
        tf2
        tf2-eigen
        tf2-geometry-msgs
        tf2-msgs
        tf2-ros
        urdf
        ];
  nativeBuildInputs = [ catkin ];

  configurePhase = ''
              cmake moveit_ros/planning -DCMAKE_INSTALL_PREFIX=$out
          '';

  meta = {
    description = ''MoveIt Ros Planning'';
    license = with lib.licenses; [ bsdOriginal ];
  };
}