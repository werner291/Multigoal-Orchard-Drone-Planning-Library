cmake_minimum_required(VERSION 3.8)
project(MgODPL) # Multigoal Orchard Drone Planning Library

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(VTK_QT_VERSION "6" CACHE STRING "Qt Version")

# Add an option to enable/disable VTK
option(ENABLE_VISUALIZATION "Enable VISUALizATION support" OFF)

# Add an option to enable/disable building the experiment_state_tools
option(ENABLE_EXPERIMENTS "Enable EXPERIMENTS" OFF)

# Add an option to build tests or not.
option(ENABLE_TESTS "Enable TESTS" OFF)

# Add an option for Python byndings.
option(ENABLE_PYTHON_BINDINGS "Enable PYTHON BINDINGS" OFF)

# Add an option to specify the directory where the robots are to be found.
# By default, this is just the source dir, but that assumes that we still have the source available, rather than the installed package.
option(ROBOTS_DIRECTORY "Where the robots are to be found.")
if (NOT ROBOTS_DIRECTORY) # Workaround for an apparent bug where CMake somehow sets OFF even if a default value is given.
    set(ROBOTS_DIRECTORY "${CMAKE_SOURCE_DIR}/test_robots")
endif ()

# Force the detection of NixOS. (TODO: I'd rather just detect the nix package manager; can I do that?)
option(FORCE_NIXOS "Force NixOS" OFF)

# Detect if we're on NixOS
if (EXISTS /etc/NIXOS OR FORCE_NIXOS)
    message(STATUS "NixOS detected")
    set(NIXOS TRUE)
else ()
    set(NIXOS FALSE)
endif ()

if (NIXOS)
    find_package(ortools REQUIRED)
    set(ORTOOLS_LIBRARIES ortools)

    if (ENABLE_PYTHON_BINDINGS)
        # Weird bug where CLion seems too substitute its own Python3 version when called from its context.
        # Here, I'm forcing it to use the flake.nix version.
        find_package(Python3 3.11.8 EXACT REQUIRED COMPONENTS Interpreter Development)
        find_package(pybind11 REQUIRED)
    endif ()
else ()
    set(ORTOOLS_LIBRARIES ${CMAKE_SOURCE_DIR}/ortools/lib/libortools.so)

    if (ENABLE_PYTHON_BINDINGS)
        find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
        find_package(pybind11 REQUIRED)
    endif ()
endif ()

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)
find_package(Boost REQUIRED)
find_package(range-v3 REQUIRED)

find_file(DEBIAN_FOUND debian_version debconf.conf
        PATHS /etc
)

set(CGAL_DATA_DIR "UNUSED_STOP_WARNING")
find_package(CGAL REQUIRED)

find_package(jsoncpp REQUIRED)

find_package(assimp REQUIRED)

include_directories(
        ${FCL_INCLUDE_DIRS}
        ${JSONCPP_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GTEST_INCLUDE_DIRS}
        ${BULLET_INCLUDE_DIRS}
        #        ${OMPL_INCLUDE_DIRS}
        ${QHULL_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
        ortools/include
        miniball/cpp/main
)


link_libraries(
        tbb
        #        ${EIGEN_LIBRARIES}
        ${FCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${BULLET_LIBRARIES}
        ${VTK_LIBRARIES}
        jsoncpp_lib
        assimp
        #        tbb
        ${CGAL_LIBRARIES}
        #        ${CMAKE_SOURCE_DIR}/ortools/lib/libortools.so
        #        moveit_ompl_interface
        ${ORTOOLS_LIBRARIES}
        flann
        lz4 # For flann
)

###########
## Build ##
###########

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
SET(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

add_compile_definitions(PARALLEL) # Enable parallelism in some parts of the code.
add_compile_definitions(ROBOTS_DIR="${ROBOTS_DIRECTORY}") # Forwards the robots directory to the code.

# A small library featuring some vector math (not depending on Eigen)
# and some general-purpose geometry utilities.
add_library(math_utils
        src/math/AABBGrid.h
        src/math/AABBGrid.cpp
        src/math/AABB.h
        src/math/Vec3.h
        src/math/grid_utils.h
        src/math/grid_utils.cpp
        src/math/ParametricLine.cpp
        src/math/ParametricLine.h
        src/math/Segment3d.cpp
        src/math/Segment3d.h
        src/math/intersections.cpp
        src/math/intersections.h
        src/math/Triangle.h
        src/math/OpenPyramid.h
        src/math/Plane.cpp
        src/math/Plane.h
        src/math/RangeInclusive.h
        src/math/RangeInclusive.cpp
        src/math/Ray.h
        src/math/Ray.cpp
        src/math/DomainSlice.h
        src/math/Triangle.cpp
        src/math/aabb_of.cpp
        src/math/aabb_of.h
        src/math/Polar.cpp
        src/math/Polar.h
        src/math/Quaternion.cpp
        src/math/Quaternion.h
        src/math/Transform.cpp
        src/math/Transform.h
)

add_library(visibility
        src/visibility/GridVec.h
        src/visibility/GridVec.cpp
        src/visibility/voxel_visibility.h
        src/visibility/voxel_visibility.cpp
        src/visibility/Octree.cpp
        src/visibility/Octree.h
        #        src/visibility/octree_visibility.cpp
        src/visibility/octree_visibility.h
        src/visibility/visibility_geometry.cpp
        src/visibility/visibility_geometry.h
)

target_link_libraries(visibility math_utils)

add_library(planning
        src/planning/RobotAlgorithm.h
        #    src/planning/BlindlyMoveToNextFruit.h
        #    src/planning/BlindlyMoveToNextFruit.cpp
        src/planning/NextTargetSelector.cpp
        src/planning/NextTargetSelector.h
        src/planning/MonadicBehavior.h
        src/planning/cgal_chull_shortest_paths.cpp
        src/planning/cgal_chull_shortest_paths.h
        src/planning/ConvexHullSpace.cpp
        src/planning/ConvexHullSpace.h
        src/planning/RobotModel.cpp
        src/planning/RobotModel.h
        src/planning/RobotState.cpp
        src/planning/LatitudeLongitudeGrid.h
        src/planning/LatitudeLongitudeGrid.cpp
        src/planning/scanline.cpp
        src/planning/scanline.h
        src/planning/spherical_geometry.cpp
        src/planning/collision_detection.cpp
        src/planning/collision_detection.h
        src/planning/state_tools.cpp
        src/planning/state_tools.h
        src/planning/goal_sampling.cpp
        src/planning/goal_sampling.h
        src/planning/shell_path.cpp
        src/planning/shell_path.h
        src/planning/RobotPath.h
        src/planning/visitation_order.h
        src/planning/visitation_order.cpp
        src/planning/ApproachPath.h
        src/planning/approach_path_planning.cpp
        src/planning/approach_path_planning.h
        src/planning/shell_path_assembly.cpp
        src/planning/shell_path_assembly.h
        src/planning/probing_motions.cpp
        src/planning/probing_motions.h
        src/planning/RobotPath.cpp
        src/planning/swept_volume_ccd.cpp
        src/planning/swept_volume_ccd.h
        src/planning/scanning_motions.cpp
        src/planning/scanning_motions.h
        src/planning/Mesh.h
        src/planning/Mesh.cpp
        src/planning/RandomNumberGenerator.cpp
        src/planning/RandomNumberGenerator.h
        src/planning/fcl_utils.cpp
        src/planning/fcl_utils.h
        src/planning/local_optimization.cpp
        src/planning/local_optimization.h
        src/planning/traveling_salesman.h
        src/planning/traveling_salesman.cpp
        src/planning/tsp_over_prm.cpp
        src/planning/tsp_over_prm.h
        src/planning/GroupIndexTable.cpp
        src/planning/GroupIndexTable.h
        src/planning/nearest_neighbours/NearestNeighborsGNAT.h
        src/planning/nearest_neighbours/NearestNeighbors.h
        src/planning/nearest_neighbours/GreedyKCenters.h
        src/planning/RobotPathFn.cpp
        src/planning/RobotPathFn.h
        src/planning/ParametricInfiniteCone.cpp
        src/planning/ParametricInfiniteCone.h
)

target_link_libraries(planning math_utils)

#target_compile_options(visibility PRIVATE -O3)

if (ENABLE_EXPERIMENTS)
    # A library that is mostly useful for running the experiment_state_tools.
    add_library(experiment_utils
            src/experiment_utils/TreeMeshes.h
            src/experiment_utils/TreeMeshes.cpp
            src/experiment_utils/mesh_connected_components.cpp
            src/experiment_utils/mesh_connected_components.h
            src/experiment_utils/snake_path.cpp
            src/experiment_utils/snake_path.h
            src/experiment_utils/MeshOcclusionModel.cpp
            src/experiment_utils/MeshOcclusionModel.h
            #            src/planning/IncrementalGoalStateGenerator.cpp
            #            src/planning/IncrementalGoalStateGenerator.h
            src/planning/fcl_forward_declarations.h
            src/experiment_utils/procedural_robot_models.cpp
            src/experiment_utils/procedural_robot_models.h
            src/experiment_utils/shapes.cpp
            src/experiment_utils/shapes.h
            src/experiment_utils/positioned_shape.cpp
            src/experiment_utils/positioned_shape.h
            src/experiment_utils/leaf_scaling.cpp
            src/experiment_utils/leaf_scaling.h
            src/experiment_utils/surface_points.cpp
            src/experiment_utils/surface_points.h
            src/experiment_utils/scan_paths.cpp
            src/experiment_utils/scan_paths.h
            src/experiment_utils/scan_path_generators.cpp
            src/experiment_utils/scan_path_generators.h
            src/experiment_utils/procedural_fruit_placement.cpp
            src/experiment_utils/procedural_fruit_placement.h
            src/experiment_utils/prompting.cpp
            src/experiment_utils/prompting.h
            src/experiment_utils/declarative/SensorModelParameters.h
            src/experiment_utils/joint_distances.cpp
            src/experiment_utils/joint_distances.h
            src/experiment_utils/point_scanning_evaluation.cpp
            src/experiment_utils/point_scanning_evaluation.h
            src/experiment_utils/declarative_environment.cpp
            src/experiment_utils/declarative_environment.h
            src/experiment_utils/parameter_space.cpp
            src/experiment_utils/parameter_space.h
            src/experiment_utils/parametric_paths.cpp
            src/experiment_utils/parametric_paths.h
            src/experiment_utils/tree_models.cpp
            src/experiment_utils/tree_models.h
            src/experiment_utils/LoadedTreeModel.cpp
            src/experiment_utils/LoadedTreeModel.h
            src/experiment_utils/declarative/to_json.cpp
            src/experiment_utils/declarative/fruit_models.h
            src/experiment_utils/mesh_from_dae.cpp
            src/experiment_utils/mesh_from_dae.h
            src/experiment_utils/tracing.cpp
            src/experiment_utils/tracing.h
            src/experiment_utils/declarative/local_optimization.h
            src/experiment_utils/tree_benchmark_data.cpp
            src/experiment_utils/tree_benchmark_data.h
    )

    target_compile_definitions(experiment_utils PRIVATE MYSOURCE_ROOT="${CMAKE_SOURCE_DIR}")

    target_link_libraries(experiment_utils math_utils visibility)

    # Get the latest abbreviated commit hash of the working branch
    execute_process(
            COMMAND git log -1 --format=%h
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
            OUTPUT_VARIABLE GIT_HASH
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    add_compile_definitions(GIT_HASH="${GIT_HASH}")

    add_executable(benchmarks
            src/benchmarks/main.cpp
            src/benchmarks/goal_sample_success_rate.cpp
            src/benchmarks/tree_stats.cpp
            src/benchmarks/sampling_difficulty.cpp
            src/benchmarks/probing_motions.cpp
    )

    target_link_libraries(benchmarks math_utils visibility experiment_utils planning)
endif ()

if (ENABLE_VISUALIZATION)

    message(STATUS "Building with visualization")

    find_package(VTK REQUIRED)
    include_directories(${VTK_INCLUDE_DIRS})

    # A library for vizualizing the experiment_state_tools.
    add_library(${PROJECT_NAME}_visualisation
            src/visualization/vtk.h
            src/visualization/vtk.cpp
            src/visualization/SimpleVtkViewer.h
            src/visualization/SimpleVtkViewer.cpp
            src/visualization/VideoRecorder.h
            src/visualization/VideoRecorder.cpp
            src/visualization/VtkFunctionalCallback.cpp
            src/visualization/VtkFunctionalCallback.h
            src/visualization/camera_controls.h
            src/visualization/camera_controls.cpp
            src/visualization/VtkPolyLineVisualization.cpp
            src/visualization/VtkPolyLineVisualization.h
            src/visualization/VtkLineSegmentVizualization.cpp
            src/visualization/VtkLineSegmentVizualization.h
            src/visualization/VtkTriangleSetVisualization.h
            src/visualization/VtkTriangleSetVisualization.cpp
            src/visualization/voxels.cpp
            src/visualization/voxels.h
            src/visualization/quick_markers.h
            src/visualization/quick_markers.cpp
            src/visualization/scannable_points.cpp
            src/visualization/scannable_points.h
            src/visualization/robot_state.cpp
            src/visualization/robot_state.h
            src/visualization/declarative.cpp
            src/visualization/ladder_trace.cpp
            src/visualization/ladder_trace.h
            src/visualization/Throttle.cpp
            src/visualization/Throttle.h
            src/visualization/RunQueue.cpp
            src/visualization/RunQueue.h
    )

    target_link_directories(${PROJECT_NAME}_visualisation PUBLIC ${VTK_LIBRARY_DIRS})
    target_link_libraries(${PROJECT_NAME}_visualisation ${VTK_LIBRARIES} experiment_utils)

    add_executable(vizcube_lightweight src/experiments/cubeviz.cpp)
    target_link_libraries(vizcube_lightweight math_utils ${PROJECT_NAME}_visualisation experiment_utils)

    add_executable(no_preknowledge src/experiments/no_preknowledge.cpp)
    target_link_libraries(no_preknowledge math_utils ${PROJECT_NAME}_visualisation experiment_utils planning)

    add_executable(approach_planning src/experiments/approach_planning.cpp)
    target_link_libraries(approach_planning math_utils ${PROJECT_NAME}_visualisation experiment_utils planning)

    add_executable(goal_construction src/experiments/goal_construction.cpp)
    target_link_libraries(goal_construction math_utils ${PROJECT_NAME}_visualisation experiment_utils planning)

    add_executable(latlongrid_vs_sampling src/experiments/latlongrid_vs_sampling.cpp)
    target_link_libraries(latlongrid_vs_sampling math_utils ${PROJECT_NAME}_visualisation experiment_utils planning)

    add_executable(collision_detection_mwe src/experiments/collision_detection_mwe.cpp)

    add_executable(latitude_sweep_vizualization src/experiments/latitude_sweep_vizualization.cpp)
    target_link_libraries(latitude_sweep_vizualization math_utils ${PROJECT_NAME}_visualisation experiment_utils planning)

    add_executable(new_integration src/experiments/new_integration.cpp)
    target_link_libraries(new_integration math_utils ${PROJECT_NAME}_visualisation experiment_utils planning)

    add_executable(sphere_scannable_points_orbit src/experiments/sphere_scannable_points_orbit.cpp)
    target_link_libraries(sphere_scannable_points_orbit math_utils experiment_utils planning)

    add_executable(point_scanning src/experiments/point_scanning.cpp)
    target_link_libraries(point_scanning math_utils ${PROJECT_NAME}_visualisation experiment_utils planning)

    add_executable(visualizations
            src/visualizations/viz_main.cpp
            src/visualizations/creases.cpp
            src/visualizations/fruit_scan_fullpath.cpp
            src/visualizations/fruit_scan_points.cpp
            src/visualizations/tree_variations.cpp
            src/visualizations/probing.cpp
            src/visualizations/swept_volume.cpp
            src/visualizations/robot_variations.cpp
            src/visualizations/declarative.cpp
            # src/visualizations/swapper.cpp
            src/visualizations/reacharound_motions.cpp
            src/visualizations/tsp_over_prm.cpp
            src/visualizations/shortcutting.cpp
            src/visualizations/goal_samples.cpp
            src/visualizations/scanning_motions.cpp
    )

    target_link_libraries(visualizations math_utils ${PROJECT_NAME}_visualisation experiment_utils planning)

endif ()

if (ENABLE_TESTS)

    include(GoogleTest)

    add_executable(${PROJECT_NAME}_tests
            test/test.cpp
            #        test/math/AABBGrid_test.cpp
            #        test/math/AABB_test.cpp
            #        test/experiment_utils/voxel_visibility_test.cpp
            #        test/math/intersection_test.cpp
            #        test/math/lp_test.cpp
            test/planning/spherical_geomety_test.cpp
            test/planning/LatitudeLongitudeGridTests.cpp
            src/experiment_utils/declarative/PointScanExperiment.h
            src/experiment_utils/declarative/to_json.cpp
            src/visualization/declarative.cpp
            src/visualization/declarative.h
    )

    target_link_libraries(${PROJECT_NAME}_tests math_utils experiment_utils planning gtest)
    gtest_discover_tests(${PROJECT_NAME}_tests)
endif () # ENABLE_TESTS

if (ENABLE_PYTHON_BINDINGS)
    message(STATUS "Building with Python bindings")
    pybind11_add_module(pymgodpl src/python_bindings.cpp)
    target_link_libraries(pymgodpl PRIVATE math_utils visibility planning experiment_utils ${PROJECT_NAME}_visualisation ${VTK_LIBRARIES})
    ament_target_dependencies(pymgodpl PUBLIC ${AMENT_DEPS})
endif ()

add_custom_target(benchmarks_and_visualizations)
add_dependencies(benchmarks_and_visualizations benchmarks visualizations)