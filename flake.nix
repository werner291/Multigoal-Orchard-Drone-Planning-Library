{
  description = "My ROS Project Build Environment";
  nixConfig.bash-prompt = "[ros] ";
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-23.05";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
  };

  outputs = { self, nixpkgs, nix-ros-overlay }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; overlays = [ nix-ros-overlay.overlays.default (import ./replace-ompl.nix) ]; };
      moveit-src = fetchGit { url="https://github.com/ros-planning/moveit2.git"; rev="ce240bd0c50f108124289acd137a7aca8bab8f48"; };
      moveit-core = pkgs.rosPackages.rolling.moveit-core.overrideAttrs (_: {version="master"; src="${moveit-src}/moveit_core";});
      moveit-planners-ompl = ((pkgs.rosPackages.rolling.moveit-planners-ompl.overrideAttrs (_: {version="master"; src="${moveit-src}/moveit_planners/ompl";})).override { moveit-core = moveit-core; });
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        name = "My ROS Project Build Environment";
        packages = with pkgs; with pkgs.rosPackages.rolling; [
            moveit-core
            ompl
	    pkg-config
#            moveit-planners-ompl
            (python3.withPackages(ps: with ps; [ matplotlib jupyterlab numpy pandas pybind11 seaborn ]))
            cmake
            jsoncpp
            range-v3
            cgal_5
            gmp
            mpfr
            or-tools
            re2
            CoinMP
            glpk
            vtk
	    eigen
	    pandoc
	    tbb # For the parallel for_each
        ];
      };
    };
}
