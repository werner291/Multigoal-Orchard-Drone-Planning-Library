{ pkgs } :
    let moveit_pkgs = (import ./moveit_package.nix {pkgs=pkgs;});
    in with pkgs; with rosPackages.noetic; buildRosPackage {
        pname = "ros-noetic-robowflex";
        version = "rev-master";

#        src = fetchGit {
#            url = "https://github.com/KavrakiLab/robowflex.git";
#            rev = "ec92fefa75e82e9609076bd3ea96bf71c393c340";
#        };
        src=/home/werner/catkin_ws/src/robowflex;

        buildType = "catkin";
        checkInputs = [ ];
        propagatedBuildInputs = [ 
            assimp
            moveit_pkgs.moveit_core
            moveit_pkgs.moveit_ros_planning
            boost
            bullet 
            console-bridge 
            eigen 
            eigen-stl-containers
            fcl 
            geometric-shapes 
            geometry-msgs 
            kdl-parser
            moveit-msgs
            octomap
            octomap-msgs
            pybind11-catkin
            random-numbers
            rosconsole
            roslib
            rostime
            sensor-msgs
            shape-msgs
            srdfdom
            std-msgs
            tf2-eigen
            tf2-geometry-msgs
            trajectory-msgs
            urdf
            urdfdom
            urdfdom-headers
            visualization-msgs
            xmlrpcpp 
            ];
        nativeBuildInputs = [ 
            cmake
            catkin pkg-config 
            libyamlcpp
            hdf5
            hdf5-cpp
            rosbag
            ];
            
        configurePhase = ''
            cmake robowflex_library -DCMAKE_INSTALL_PREFIX=$out
        '';


        meta = {
            description = ''Wrapper library for MoveIt.'';
            #license = with lib.licenses; [ bsdOriginal ];
        };
    }
