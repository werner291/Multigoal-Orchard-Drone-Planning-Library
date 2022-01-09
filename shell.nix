let 
    tb = builtins.fetchTarball { name = "nix-ros-overlay"; url = "https://github.com/lopsided98/nix-ros-overlay/archive/3f30908bc180ecab76d64f71b779c40f8b106949.tar.gz";};
    pkgs = import tb { };
    robowflex_library = import nix/robowflex_library.nix { pkgs = pkgs; };
    robowflex_ompl = import nix/robowflex_ompl.nix { pkgs = pkgs; };
    moveit_pkgs = (import nix/moveit_package.nix {pkgs=pkgs;});
in
    with pkgs; with rosPackages.noetic;
        mkShell {
            nativeBuildInputs = [
                moveit_pkgs.moveit_core
                moveit_pkgs.moveit_planners_ompl
                #moveit_pkgs.moveit_warehouse
                #moveit_pkgs.moveit_move_group
                #moveit_pkgs.moveit_ros_planning_interface
                (import nix/moveit_visual_tools.nix {pkgs=pkgs;})
                #robowflex_library
                #robowflex_ompl
                #warehouse-ros
                #rviz-visual-tools
                jsoncpp
                catkin
                cmake
                eigen-conversions
                pythonPackages.numpy 
                pythonPackages.notebook
                pythonPackages.pandas
                pythonPackages.matplotlib
            ];

		    shellHook = ''
      export TMPDIR="/tmp"
    '';

        }
