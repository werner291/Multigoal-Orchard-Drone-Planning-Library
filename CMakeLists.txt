cmake_minimum_required(VERSION 3.8)
project(MgODPL) # Multigoal Orchard Drone Planning Library

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(VTK_QT_VERSION "6" CACHE STRING "Qt Version")

# Add an option to enable/disable VTK
option(ENABLE_VISUALIZATION "Enable VISUALizATION support" OFF)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(fcl REQUIRED)
find_package(Boost REQUIRED)
find_package(Bullet REQUIRED)
find_package(ompl REQUIRED)
find_package(range-v3 REQUIRED)
find_package(Qhull REQUIRED)

set(CGAL_DATA_DIR "UNUSED_STOP_WARNING")
find_package(CGAL REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(shape_msgs REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${FCL_INCLUDE_DIRS}
        ${JSONCPP_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GTEST_INCLUDE_DIRS}
        ${BULLET_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
        ${QHULL_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
        #{CGAL_INCLUDE_DIRS}
        ortools/include
        miniball/cpp/main
)

link_libraries(
        ${EIGEN_LIBRARIES}
        ${FCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${BULLET_LIBRARIES}
        ${OMPL_LIBRARIES}
        ${VTK_LIBRARIES}
        qhullcpp
        HACD # Note: the HACD library is included Bullet's "extra" libraries
        jsoncpp_lib
        tbb
        ${CGAL_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/ortools/lib/libortools.so
)

###########
## Build ##
###########

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#SET(CMAKE_CXX_FLAGS_DEBUG "-g -Og")
#SET(CMAKE_C_FLAGS_DEBUG "-O0")

add_library(${PROJECT_NAME}_shared
        src/BulletContinuousMotionValidator.cpp
        src/BulletContinuousMotionValidator.h
        src/CurrentPathState.cpp
        src/CurrentPathState.h
        src/DirectApproachVariantSampler.cpp
        src/DirectApproachVariantSampler.h
        src/DistanceHeuristics.cpp
        src/DistanceHeuristics.h
        src/DronePathLengthObjective.cpp
        src/DronePathLengthObjective.h
        src/DroneStateConstraintSampler.cpp
        src/DroneStateSampler.cpp
        src/DroneStateSampler.h
        src/DynamicGoalVisitationEvaluation.cpp
        src/DynamicGoalVisitationEvaluation.h
        src/DynamicGoalsetPlanningProblem.cpp
        src/DynamicGoalsetPlanningProblem.h
        src/DynamicMultiGoalPlanner.cpp
        src/DynamicMultiGoalPlanner.h
        src/EndEffectorOnShellGoal.cpp
        src/EndEffectorOnShellGoal.h
        src/ExperimentVisualTools.cpp
        src/ExperimentVisualTools.h
        src/GreatCircleMetric.cpp
        src/GreatCircleMetric.h
        src/HashedSpatialIndex.cpp
        src/HashedSpatialIndex.h
        src/IncrementalTSPMethods.cpp
        src/IncrementalTSPMethods.h
        src/InformedBetweenTwoDroneStatesSampler.h
        src/InformedManipulatorDroneSampler.cpp
        src/InformedRobotStateSampler.cpp
        src/InformedRobotStateSampler.h
        src/LeavesCollisionChecker.cpp
        src/LeavesCollisionChecker.h
        src/ORToolsTSPMethods.cpp
        src/ORToolsTSPMethods.h
        src/PathInterrupt.cpp
        src/PathInterrupt.h
        src/RobotPath.cpp
        src/RobotPath.h
        src/RunPlannerThreaded.cpp
        src/RunPlannerThreaded.h
        src/SamplerWrapper.cpp
        src/SamplerWrapper.h
        src/SingleGoalPlannerMethods.cpp
        src/SingleGoalPlannerMethods.h
        src/StreamingConvexHull.cpp
        src/StreamingConvexHull.h
        src/TimedCostConvergenceTerminationCondition.cpp
        src/TimedCostConvergenceTerminationCondition.h
        src/TreeMeshes.cpp
        src/TreeMeshes.h
        src/TriangleAABB.cpp
        src/TriangleAABB.h
        src/ompl_custom.cpp
        src/ompl_custom.h
        src/planner_allocators.cpp
        src/planner_allocators.h
        src/planners/CachingDynamicPlanner.cpp
        src/planners/CachingDynamicPlanner.h
        src/planners/ChangeAccumulatingPlannerAdapter.cpp
        src/planners/ChangeAccumulatingPlannerAdapter.h
        src/planners/ChangeIgnoringReplannerAdapter.cpp
        src/planners/ChangeIgnoringReplannerAdapter.h
        src/planners/DynamicMultiGoalPlannerOmplToMoveitAdapter.cpp
        src/planners/DynamicMultiGoalPlannerOmplToMoveitAdapter.h
        src/planners/InitialOrbitPlanner.cpp
        src/planners/InitialOrbitPlanner.h
        src/planners/MultiGoalPlanner.cpp
        src/planners/MultiGoalPlanner.h
        src/planners/MultigoalPrmStar.cpp
        src/planners/MultigoalPrmStar.h
        src/planners/ShellPathPlanner.cpp
        src/planners/ShellPathPlanner.h
        src/planners/shell_path_planner/Construction.cpp
        src/planners/shell_path_planner/Construction.h
        src/planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.cpp
        src/planners/shell_path_planner/OrderingMethod.h
        src/AppleTreePlanningScene.cpp
        src/AppleTreePlanningScene.h
        src/probe_retreat_move.cpp
        src/probe_retreat_move.h
        src/procedural_tree_generation.cpp
        src/run_experiment.cpp
        src/run_experiment.h
        src/shell_space/CGALMeshShell.cpp
        src/shell_space/CGALMeshShell.h
        src/shell_space/CuttingPlaneConvexHullShell.cpp
        src/shell_space/CuttingPlaneConvexHullShell.h
        src/shell_space/CylinderShell.cpp
        src/shell_space/CylinderShell.h
        src/shell_space/MoveItShellSpace.cpp
        src/shell_space/MoveItShellSpace.h
        src/shell_space/OmplShellSpace.cpp
        src/shell_space/OmplShellSpace.h
        src/shell_space/SphereShell.cpp
        src/shell_space/SphereShell.h
        src/shell_space/WorkspaceShell.cpp
        src/shell_space/WorkspaceShell.h
        src/utilities/JsonChunkedLogger.cpp
        src/utilities/JsonChunkedLogger.h
        src/utilities/MeshOcclusionModel.cpp
        src/utilities/MeshOcclusionModel.h
        src/utilities/alpha_shape.cpp
        src/utilities/alpha_shape.h
        src/utilities/convex_hull.cpp
        src/utilities/convex_hull.h
        src/utilities/discoverability_specifications.cpp
        src/utilities/enclosing_sphere.cpp
        src/utilities/enclosing_sphere.cpp
        src/utilities/enclosing_sphere.h
        src/utilities/enclosing_sphere.h
        src/utilities/experiment_utils.cpp
        src/utilities/experiment_utils.h
        src/utilities/general_utilities.cpp
        src/utilities/general_utilities.h
        src/utilities/geogebra.cpp
        src/utilities/geogebra.h
        src/utilities/goal_events.cpp
        src/utilities/goal_events.h
        src/utilities/goal_events.h
        src/utilities/json_utils.cpp
        src/utilities/json_utils.h
        src/utilities/load_mesh.cpp
        src/utilities/load_mesh.h
        src/utilities/math_utils.cpp
        src/utilities/math_utils.h
        src/utilities/mesh_utils.cpp
        src/utilities/mesh_utils.h
        src/utilities/moveit.cpp
        src/utilities/moveit.cpp
        src/utilities/moveit.h
        src/utilities/moveit.h
        src/utilities/moveit_conversions.cpp
        src/utilities/moveit_conversions.h
        src/utilities/msgs_utilities.cpp
        src/utilities/msgs_utilities.h
        src/utilities/ompl_tools.cpp
        src/utilities/ompl_tools.h
        src/utilities/run_experiments.cpp
        src/utilities/run_experiments.h
        src/utilities/shape_generation.cpp
        src/utilities/shape_generation.h
        src/utilities/trajectory_primitives.cpp
        src/utilities/trajectory_primitives.h
        src/utilities/traveling_salesman.cpp
        src/utilities/traveling_salesman.h
        src/utilities/vector_utils.h
        src/RunPlannerThreaded.cpp
        src/utilities/trajectoryTiming.cpp
        src/utilities/trajectoryTiming.h
        src/OrbitPath.h
        src/OrbitPath.cpp
        src/Instrumented.h
        src/SimpleIncrementalTSPMethods.cpp
        src/SimpleIncrementalTSPMethods.h
        src/utilities/size_t_iota.h
        src/shell_space/HorizontalOrientedBoundingBox.cpp
        src/shell_space/HorizontalOrientedBoundingBox.h
        src/shell_space/DendriticConvexHullShell.cpp
        src/shell_space/DendriticConvexHullShell.h
        src/utilities/delaunay.cpp
        src/utilities/delaunay.h
        src/PathLengthPredictor.cpp
        src/PathLengthPredictor.h
        src/PathLengthPredictor.h
        src/shell_space/path_util.h
        src/shell_space/path_util.cpp
        src/goal_optimization.cpp
        src/goal_optimization.cpp
        src/utilities/midpoint_pull.cpp
        src/utilities/midpoint_pull.h
        src/orchards.cpp
        src/orchards.h
        src/static_problem_generation.cpp
        src/static_problem_generation.h
        src/run_static.cpp
        src/run_static.h
        src/utilities/link_trace.cpp
        src/utilities/link_trace.h
        src/quickplan.cpp
        src/quickplan.h
        src/utilities/optimization_meta.h
        src/dynamic_goalset_experiment.cpp
        src/dynamic_goalset_experiment.h
        src/CanSeeApple.cpp
        src/CanSeeApple.h
        src/pure/shell_path_planning.h
        src/pure/DynamicShellPathPlanner.h
        src/pure/moveit_impl/path.h
        src/pure/path.h
        src/pure/ompl_impl/path.h
        src/pure/Shell.h
        src/pure/shell_space_impl/MinimumEnclosingShell.cpp
        src/pure/shell_space_impl/MinimumEnclosingSphereShell.h
        src/pure/ShellConfigurationSpace.h
        src/pure/WorkspaceShell.h
        src/pure/shell_space_impl/CGALMeshShell.cpp
        src/pure/shell_space_impl/CGALMeshShell.h
        src/quickplan.h
        src/utilities/optimization_meta.h
        src/dynamic_goalset_experiment.cpp
        src/dynamic_goalset_experiment.h
        src/CanSeeApple.cpp
        src/CanSeeApple.h
        src/utilities/fcl_util.cpp
        src/utilities/fcl_util.h src/pure/moveit_impl/ShellConfigurationFromShellSurface.h src/pure/goal_region.h src/pure/end_effector_near_point_goal_region.h src/pure/approach_planning.h src/pure/motion.h src/pure/configuration_space.h src/pure/moveit_impl/MoveitConfigurationSpace.cpp src/pure/moveit_impl/MoveitConfigurationSpace.h src/pure/configuration.h src/pure/collision_checking.h src/pure/moveit_impl/EndEffectorOnPointGoalRegion.h src/pure/moveit_impl/approach_planning.h src/pure/moveit_impl/approach_planning.h src/pure/distance_matrix.h)

if (ENABLE_VISUALIZATION)
    find_package(VTK REQUIRED)

    find_package(Qt5 COMPONENTS Core Widgets Concurrent REQUIRED)
    set(CMAKE_AUTOMOC ON)

    add_library(${PROJECT_NAME}_visualization
            src/RobotCameraTracker.cpp
            src/RobotCameraTracker.h
            src/apple_status_color_coding.cpp
            src/apple_status_color_coding.h
            src/utilities/vtk.cpp
            src/utilities/vtk.h
            src/visualization/SimpleVtkViewer.cpp
            src/visualization/SimpleVtkViewer.h
            # src/visualization/SimulatedSensor.cpp
            # src/visualization/SimulatedSensor.h
            src/visualization/StatusTextViz.cpp
            src/visualization/StatusTextViz.h
            src/visualization/VideoRecorder.cpp
            src/visualization/VideoRecorder.h
            src/visualization/Viewer.cpp
            src/visualization/Viewer.h
            src/visualization/VisualizationSpecifics.cpp
            src/visualization/VisualizationSpecifics.h
            src/visualization/VtkRobotModel.cpp
            src/visualization/VtkRobotModel.h
            src/visualization/camera_controls.cpp
            src/visualization/camera_controls.h
            src/visualization/orchard_actors.cpp
            src/visualization/orchard_actors.h
            src/visualization/shell_visualization.cpp
            src/visualization/shell_visualization.h
            src/visualization/visualize_dynamic.cpp
            src/visualization/visualize_dynamic.h
            src/visualization/visualize_path.cpp
            src/visualization/visualize_path.h
            src/visualization/AppleIDSelector.cpp
            src/visualization/AppleIDSelector.h
            src/visualization/ShellWrapper.h
            src/visualization/ObservableInt.cpp
            src/visualization/ObservableInt.h
            src/visualization/chain_qt_futures.cpp
            src/visualization/chain_qt_futures.h
            src/visualization/TreeMeshesLoader.cpp
            src/visualization/TreeMeshesLoader.h
            src/visualization/ActorsVisibilityWidget.cpp
            src/visualization/ActorsVisibilityWidget.h
            src/visualization/numeric_combobox.cpp
            src/visualization/numeric_combobox.h
            src/visualization/path_visualization.cpp
            src/visualization/path_visualization.h
            src/visualization/LabeledActors.cpp
            src/visualization/LabeledActors.h
            src/visualization/compute_prm.cpp
            src/visualization/compute_prm.h
            )

    target_link_libraries(${PROJECT_NAME}_visualization ${PROJECT_NAME}_shared Qt5::Core Qt5::Widgets Qt5::Concurrent)
    ament_target_dependencies(${PROJECT_NAME}_visualization ${AMENT_DEPS})


endif ()

target_compile_definitions(${PROJECT_NAME}_shared PRIVATE MYSOURCE_ROOT="${CMAKE_SOURCE_DIR}")

set(AMENT_DEPS "moveit_core" "moveit_msgs" "shape_msgs" "moveit_planners_ompl" "rclcpp" "ompl")

ament_target_dependencies(${PROJECT_NAME}_shared ${AMENT_DEPS})


function(experiment name)
    add_executable(${PROJECT_NAME}_experiment_${name} src/experiments/${name}.cpp)
    target_link_libraries(${PROJECT_NAME}_experiment_${name} ${PROJECT_NAME}_shared)
endfunction()

experiment("shellpath")
experiment("shellpath_shell_inflation")
experiment("shellpath_shell_bias")
experiment("shell_comparison")
experiment("tsp_over_prm_vs_sphereshell")
experiment("shell_comparison_sphere_armangle")
experiment("path_length_prediction")
experiment("shell_type_comparison")
experiment("tsp_over_prm_bb")
experiment("dynamic_goalset_icra2024")

if (ENABLE_VISUALIZATION)
    experiment("initial_orbit_comparison")
    target_link_libraries(${PROJECT_NAME}_experiment_initial_orbit_comparison ${PROJECT_NAME}_visualization)

    experiment("exploration")
    target_link_libraries(${PROJECT_NAME}_experiment_exploration ${PROJECT_NAME}_sensing)

    experiment("ignoring_vs_passive_discovery")

    experiment("non_approach_experiment")

    experiment("dynamic_visualized")
    target_link_libraries(${PROJECT_NAME}_experiment_dynamic_visualized ${PROJECT_NAME}_visualization)

    experiment("visualize_prm")
    target_link_libraries(${PROJECT_NAME}_experiment_visualize_prm ${PROJECT_NAME}_visualization)

    experiment("static_visualized")
    target_link_libraries(${PROJECT_NAME}_experiment_static_visualized ${PROJECT_NAME}_visualization Qt5::Core Qt5::Widgets Qt5::Concurrent)

    experiment("vizualized_trees_row")
    target_link_libraries(${PROJECT_NAME}_experiment_vizualized_trees_row ${PROJECT_NAME}_visualization)

    experiment("chull_underpass_visualized")
    target_link_libraries(${PROJECT_NAME}_experiment_chull_underpass_visualized ${PROJECT_NAME}_visualization)

    experiment("photoshoot_tree_models")
    target_link_libraries(${PROJECT_NAME}_experiment_photoshoot_tree_models ${PROJECT_NAME}_visualization)

    experiment("midpoint_pull_optimization")
    target_link_libraries(${PROJECT_NAME}_experiment_midpoint_pull_optimization ${PROJECT_NAME}_visualization)
endif ()

experiment("CGAL_mesh_paths_comparison")

experiment("tsp_noise_effect")

experiment("lci_vs_full_reorder_timing")

experiment("static_new")

include(GoogleTest)

add_executable(${PROJECT_NAME}_tests
        test/test.cpp
        test/pure_stuff.cpp
        #        test/MoveItPathLengthObjectiveTest.cpp
        #        test/math_test.cpp
        #        test/CuttingPlaneConvexHullShell_test.cpp
        #        test/CGALMeshShell_test.cpp
        #        src/utilities/test/math_utils_test.cpp
        #        test/SphereShell_test.cpp
        #        test/StreamingConvexHull_test.cpp
        #        test/VisualiserTest.cpp
        #        test/TimedConvergenceTerminationCondition_test.cpp
        #        test/HashedSpatialIndex_test.cpp
        #        test/LeavesCollisionChecker_test.cpp
        #        test/LeavesCollisionChecker_test.cpp
        #        test/OnlineOrderOptimization_test.cpp
        #        test/StreamingAlphaShape_test.cpp
        #        test/PointOnMeshLookup_test.cpp
        #        test/SurfletVolume_test.cpp
#        test/approachplanning_accuracy.cpp
#        test/CuttingPlaneConvexHullShell_test.cpp
#        test/HorizontalOrientedBoundingBox_test.cpp
#        test/CylinderShell_test.cpp
        )

#target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_shared ${PROJECT_NAME}_visualization gtest)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_shared gtest)
ament_target_dependencies(${PROJECT_NAME}_tests ${AMENT_DEPS})
gtest_discover_tests(${PROJECT_NAME}_tests)