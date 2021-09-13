let 
    tb = builtins.fetchTarball { name = "nix-ros-overlay"; url = "https://github.com/lopsided98/nix-ros-overlay/archive/be22697212debf9912e75bb5424285bfbbfbbe41.tar.gz";};
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
        moveit_pkgs.moveit_ros_planning
          robowflex_library
          robowflex_ompl
          rviz-visual-tools
          jsoncpp
          catkin
          cmake
  ];
}
