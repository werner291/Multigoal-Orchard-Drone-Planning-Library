{ pkgs }:
with pkgs; with rosPackages.noetic; buildRosPackage {
  pname = "ros-moveit-core";
  version = "custom-master";

  src = /home/werner/catkin_ws/src/moveit;

  buildType = "catkin";

  propagatedBuildInputs = [ dynamic-reconfigure
        angles
        assimp
        boost
        bullet
        catkin
        console-bridge
        eigen
        eigen-stl-containers
        fcl
        geometric-shapes
        geometry-msgs
        kdl-parser
        moveit-msgs
        moveit-resources-panda-moveit-config
        moveit-resources-pr2-description
        octomap
        octomap-msgs
        ompl
        orocos-kdl
        pkg-config
        pluginlib
        pybind11-catkin
        random-numbers
        rosconsole
        roscpp
        roslib
        rostime
        rosunit
        sensor-msgs
        shape-msgs
        srdfdom
        std-msgs
        tf2
        tf2-eigen
        tf2-geometry-msgs
        tf2-kdl
        trajectory-msgs
        urdf
        urdfdom
        urdfdom-headers
        visualization-msgs
        xmlrpcpp
        ];

  nativeBuildInputs = [ catkin ];

  configurePhase = ''
              cmake moveit_core -DCMAKE_INSTALL_PREFIX=$out -DCMAKE_BUILD_TYPE=Debug
          '';

  meta = {
    description = ''MoveIt Core'';
    license = with lib.licenses; [ bsdOriginal ];
  };
}