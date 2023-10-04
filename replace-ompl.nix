# See: https://github.com/lopsided98/nix-ros-overlay/issues/311

self: super:

let
  myOmpl = super.rosPackages.rolling.ompl.overrideAttrs ({ patches ? [ ], ... }: {
    version="Fix patch";
    patches = patches ++ [
      # Use full install paths for pkg-config
      (self.fetchpatch {
        url = "https://github.com/hacker1024/ompl/commit/1ddecbad87b454ac0d8e1821030e4cf7eeff2db2.patch";
        hash = "sha256-sAQLrWHoR/DhHk8TtUEy8E8VXqrvtXl2BGS5UvElJl8=";
      })
    ];
  });
in
{
  rosPackages = super.rosPackages // {
    rolling = super.rosPackages.rolling // {
      ompl = myOmpl;
      moveit-planners-ompl = super.rosPackages.rolling.moveit-planners-ompl.override {
        ompl = myOmpl;
      };
    };
  };
}