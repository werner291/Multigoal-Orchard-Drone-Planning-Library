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
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        name = "My ROS Project Build Environment";
        buildInputs = with pkgs; with pkgs.rosPackages.rolling; [
          moveit-core
          moveit-planners-ompl
          pythonPackages.matplotlib
            pythonPackages.notebook
            pythonPackages.numpy
            pythonPackages.pandas
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
        ];
      };
    };
}
