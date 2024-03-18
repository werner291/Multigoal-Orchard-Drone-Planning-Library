{
  description = "My ROS Project Build Environment";
  nixConfig.bash-prompt = "[ros] ";
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-23.11";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in {
      devShells.x86_64-linux.default = pkgs.mkShell {
        name = "My ROS Project Build Environment";
        packages = with pkgs; [
            #ompl
	        pkg-config
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
            glxinfo
	        fcl
	        boost
	        mpfr
		zlib
		assimp
        ];
      };
    };
}
