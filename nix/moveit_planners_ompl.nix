{ pkgs }:
with pkgs; with rosPackages.noetic; buildRosPackage {
  pname = "ros-noetic-moveit-planners-ompl";
  version = "1.1.5-r1";

  src = /home/werner/catkin_ws/src/moveit;

  buildType = "catkin";
  checkInputs = [
        moveit-resources-fanuc-description
        moveit-resources-panda-description
        moveit-resources-pr2-description
        rostest
        rosunit
        tf2-eigen ];
  propagatedBuildInputs = [ dynamic-reconfigure
        moveit-core
        moveit-ros-planning
        ompl
        pluginlib
        rosconsole
        roscpp
        tf2 ];
  nativeBuildInputs = [ catkin ];

  configurePhase = ''
              cmake moveit_planners/ompl -DCMAKE_INSTALL_PREFIX=$out
          '';

  meta = {
    description = ''MoveIt interface to OMPL'';
    license = with lib.licenses; [ bsdOriginal ];
  };
}