let 
    tb = builtins.fetchTarball { name = "nix-ros-overlay"; url = "https://github.com/lopsided98/nix-ros-overlay/archive/5a881cc7b5a96be946b6d360bb1a19c7ef07c524.tar.gz";};
    pkgs = import tb { };
#    robowflex_library = import nix/robowflex_library.nix { pkgs = pkgs; };
#    robowflex_ompl = import nix/robowflex_ompl.nix { pkgs = pkgs; };
#    moveit_pkgs = (import nix/moveit_package.nix {pkgs=pkgs;});
in
    with pkgs; with rosPackages.noetic;
        mkShell {
            nativeBuildInputs = [
                rviz-visual-tools
		        vtk
                catkin
                cling
                cmake
                eigen-conversions
                jsoncpp
                moveit-core
                moveit-planners-ompl
                moveit-ros-visualization
                moveit-visual-tools
                pythonPackages.matplotlib
                pythonPackages.notebook
                pythonPackages.numpy
                pythonPackages.pandas
            ];

		    shellHook = ''
      export TMPDIR="/tmp"
    '';

        }
