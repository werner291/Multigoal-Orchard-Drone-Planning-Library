let 
    tb = builtins.fetchTarball {
        name = "nix-ros-overlay"; 
        url = "https://github.com/lopsided98/nix-ros-overlay/archive/be22697212debf9912e75bb5424285bfbbfbbe41.tar.gz"; 
    };
    pkgs = import tb {};
in
    with pkgs; with rosPackages.noetic; buildRosPackage {
        pname = "ros-noetic-robowflex";
        version = "rev-master";

        #src = fetchGit {
        #    url = "https://github.com/werner291/robowflex.git";
        #    rev = "aa6299f6b99bf9b9bc9a1c972b89994b45530541";
        #};
        # src = /home/werner/catkin_ws/robowflex/robowflex; # See https://github.com/KavrakiLab/robowflex/pull/245
        src = /home/werner/catkin_ws/robowflex;

        buildType = "catkin";
        checkInputs = [ ];
        propagatedBuildInputs = [ 
            moveit-core
            moveit-ros-planning
            
            assimp boost bullet console-bridge eigen eigen-stl-containers fcl geometric-shapes geometry-msgs kdl-parser moveit-msgs octomap octomap-msgs pybind11-catkin random-numbers rosconsole roslib rostime sensor-msgs shape-msgs srdfdom std-msgs tf2-eigen tf2-geometry-msgs trajectory-msgs urdf urdfdom urdfdom-headers visualization-msgs xmlrpcpp 
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
            cmake robowflex/robowflex_library -DCMAKE_INSTALL_PREFIX=$out
        '';


        meta = {
            description = ''Wrapper library for MoveIt.'';
            #license = with lib.licenses; [ bsdOriginal ];
        };
    }
