{
  description = "Multigoal Orchard Drone Planning Library";

  nixConfig.bash-prompt = "[ros $(pwd)]$ ";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-23.11";
  };

  outputs = { self, nixpkgs, ... }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in
    {

      formatter.x86_64-linux = nixpkgs.legacyPackages.x86_64-linux.nixpkgs-fmt;

      packages.x86_64-linux.default = pkgs.stdenv.mkDerivation {
        name = "visualizations";
        src = ./.;
        nativeBuildInputs = with pkgs; [
          cmake
          ninja
          pkg-config
        ];
        buildInputs = with pkgs; [
          boost # A set of common tools for C++ development, like an unofficial standard library
          eigen # Useful tools for linear algebra; pretty heavy package, prefer to use our own math library when possible.
          fcl # A library for collision detection; it's the same one MoveIt uses internally.
          or-tools # A set of tools for combinatorial optimization; we use it for the TSP solver.
          CoinMP # Extra dependency for OR-Tools; it's a solver for linear programming problems.
          glpk # Another solver for linear programming problems, also a dependency for OR-Tools. (Should these be upstreamed as dependencies to nixpkgs OR-Tools?)
          vtk # A visualization library for rendering all the visuals.
          assimp # A library for importing 3D models; we use it to import the fruit tree models and the drone model.
          cgal_5 # The Computational Geometry Algorithms Library; we use it for the convex hull geodesics and such.
          range-v3 # A library for working with ranges in C++. I'd like to migrate to C++ 20, actually, if possible.
          jsoncpp # Simple JSON library, mainly for outputting statistics.
          tbb # A library for parallelizing for_each loops.
          gmp # A library for arbitrary precision arithmetic; used by CGAL.
          mpfr # A library for arbitrary precision floating point arithmetic; used by CGAL.
          gtest # Google test
          zlib
          re2
          flann # A library for fast approximate nearest neighbors.
          lz4 # Unlisted dependency of flann.
          (python3.withPackages (ps: with ps; [matplotlib pandas numpy]))
        ];

        configurePhase = ''
          cmake . -GNinja -DCMAKE_BUILD_TYPE=Release -DENABLE_VISUALIZATION=ON -DENABLE_EXPERIMENTS=ON -DFORCE_NIXOS=ON -DROBOTS_DIRECTORY=$out/test_robots
        '';

        buildPhase = ''
          ninja benchmarks_and_visualizations
        '';

        installPhase = ''
          ls
          mkdir -p $out/bin
          cp -r visualizations $out/bin
          cp -r benchmarks $out/bin
          cp -r test_robots $out
        '';
      };
    };
}
