# Multigoal Agricultural Drone Planning Library (MgADPL)

## Description

Multigoal Drone Planning Library (mgdpl) is a project developed to facilitate multi-goal drone planning. This C++ project leverages several libraries such as Eigen, JSONcpp, Boost, Bullet, ompl, among others to perform advanced drone planning tasks. This project is mainly built using CMake, which handles the configuration and building of the project.

## Prerequisites

The following libraries are required for building this project:

- Eigen
- JSONcpp
- FCL
- Boost
- Bullet
- OMPL
- Range-v3
- Qhull
- CGAL
- VTK
- Qt5 (Core, Widgets, Concurrent)
- Ament CMake
- rclcpp
- moveit_core
- moveit_msgs
- moveit_ros_planning
- moveit_planners_ompl
- shape_msgs

## Build Instructions

Make sure you have installed all the required libraries mentioned above.

1. Clone the repository:

```
git clone https://github.com/<your-repo>/mgdpl.git
```

2. Navigate to the directory:

```
cd mgdpl
```

3. Create a build directory and navigate to it:

```
mkdir build
cd build
```

4. Run CMake:

```
cmake ..
```

5. Build the project:

```
make
```

Replace `<experiment-name>` with the name of the experiment you want to run. The available experiments include "shellpath", "shellpath_shell_inflation", "shellpath_shell_bias", "shell_comparison", "tsp_over_prm_vs_sphereshell", "shell_comparison_sphere_armangle", "path_length_prediction", "shell_type_comparison", "initial_orbit_comparison", "exploration", "ignoring_vs_passive_discovery", "non_approach_experiment", "dynamic_visualized", "visualize_prm", "static_visualized", among others.

Please note that some experiments also require the visualization library. For these experiments, you'll need to have the corresponding dependencies installed.

## Contributing

If you want to contribute, please follow the standard GitHub flow. Fork the project, make your changes, and submit a pull request.

## License

All rights reserved at the moment. Please contact the project authors before using or redistributing this software.

## Contact

For further questions or if you encounter any issues, please contact the project authors.
