{pkgs} :
    let mkMoveItPackage = { name, subdir, internal_deps } :
       with pkgs; with rosPackages.noetic; buildRosPackage {
         pname = name;
         version = "master";

         src = /home/werner/catkin_ws/src/moveit;

         buildType = "catkin";

         propagatedBuildInputs = [
               actionlib
               angles
               assimp
               boost
               bullet
               catkin
               console-bridge
               dynamic-reconfigure
               eigen
               eigen
               eigen-stl-containers
               fcl
               geometric-shapes
               geometry-msgs
               kdl-parser
               message-filters
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
               roscpp
               roslib
               rostest
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
               tf2-msgs
               tf2-ros
               trajectory-msgs
               urdf
               urdf
               urdfdom
               urdfdom-headers
               visualization-msgs
               xmlrpcpp
               ] ++ internal_deps;
         nativeBuildInputs = [ catkin ];

         configurePhase = ''
                     cmake ${subdir} -DCMAKE_INSTALL_PREFIX=$out
                 '';

         meta = {
           description = ''MoveIt '' ++ name;
           license = with lib.licenses; [ bsdOriginal ];
         };
       };
in
    rec {
        moveit_core = mkMoveItPackage { name = "Core"; subdir = "moveit_core"; internal_deps = []; };
        moveit_planners_ompl = mkMoveItPackage { name = "Planners_OMPL"; subdir = "moveit_planners/ompl"; internal_deps = [ moveit_core moveit_ros_planning ]; };
        moveit_ros_planning = mkMoveItPackage { name = "ROS_Planning"; subdir = "moveit_ros/planning"; internal_deps = [ moveit_core moveit_ros_occupancy_map_monitor ]; };
        moveit_ros_occupancy_map_monitor = mkMoveItPackage { name = "ROS_Occupancy_map_monitor"; subdir = "moveit_ros/occupancy_map_monitor"; internal_deps = [ moveit_core ]; };
    }