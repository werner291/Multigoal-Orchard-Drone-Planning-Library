{ pkgs }:
let
  mkMoveItPackage = { name, subdir, internal_deps }:
    with pkgs; with rosPackages.noetic; buildRosPackage {
      pname = name;
      version = "master";

      src = fetchGit {
        url = "https://github.com/ros-planning/moveit.git";
        rev = "5ddb027492f16f714fb22d80e012372f1e262d9e";

      };

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
        eigenpy
        eigen-stl-containers
        fcl
        geometric-shapes
        geometry-msgs
        kdl-parser
        message-filters
        moveit-msgs
        moveit-resources-panda-moveit-config
        moveit-resources-pr2-description
        warehouse-ros
        octomap
        octomap-msgs
        ompl
        orocos-kdl
        pkg-config
        pluginlib
        pybind11-catkin
        python3
        random-numbers
        rosconsole
        rospy
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
  moveit_core = mkMoveItPackage { name = "Core"; subdir = "moveit_core"; internal_deps = [ ]; };
  moveit_planners_ompl = mkMoveItPackage { name = "Planners_OMPL"; subdir = "moveit_planners/ompl"; internal_deps = [ moveit_core moveit_ros_planning ]; };
  moveit_ros_planning = mkMoveItPackage { name = "ROS_Planning"; subdir = "moveit_ros/planning"; internal_deps = [ moveit_core moveit_ros_occupancy_map_monitor ]; };
  moveit_ros_planning_interface = mkMoveItPackage { name = "ROS_Planning_Interface"; subdir = "moveit_ros/planning_interface"; internal_deps = [ moveit_ros_manipulation moveit_move_group moveit_ros_planning moveit_warehouse ]; };
  moveit_warehouse = mkMoveItPackage { name = "ROS_Warehouse"; subdir = "moveit_ros/warehouse"; internal_deps = [ moveit_ros_planning ]; };
  moveit_ros_occupancy_map_monitor = mkMoveItPackage { name = "ROS_Occupancy_map_monitor"; subdir = "moveit_ros/occupancy_map_monitor"; internal_deps = [ moveit_core ]; };
  moveit_move_group = mkMoveItPackage { name = "Move_Group"; subdir = "moveit_ros/move_group"; internal_deps = [ moveit_core moveit_kinematics moveit_ros_planning ]; };
  moveit_kinematics = mkMoveItPackage { name = "Kinematics"; subdir = "moveit_kinematics"; internal_deps = [ moveit_core moveit_ros_planning ]; };
  moveit_ros_robot_interaction = mkMoveItPackage { name = "Robot Interaction"; subdir = "moveit_ros/robot_"; internal_deps = [ moveit_core moveit_ros_planning moveit_move_group ]; };
  moveit_ros_manipulation = mkMoveItPackage { name = "Manipulation"; subdir = "moveit_ros/manipulation"; internal_deps = [ moveit_core moveit_ros_planning moveit_move_group ]; };
  moveit_ros_visualization = mkMoveItPackage { name = "Visualization"; subdir = "moveit_ros/visualization"; internal_deps = [ moveit_core moveit_ros_perception pkgs.glew ]; };
  moveit_ros_perception = mkMoveItPackage {
    name = "Perception";
    subdir = "moveit_ros/perception";
    internal_deps = [
      moveit_core
      pkgs.glew
      pkgs.freeglut
      pkgs.rosPackages.noetic.cv-bridge
      moveit_ros_occupancy_map_monitor
      moveit_ros_planning
      pkgs.rosPackages.noetic.nodelet
      moveit_ros_planning_interface
    ];
  };
}
