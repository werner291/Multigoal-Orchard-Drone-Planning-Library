cmake_minimum_required(VERSION 3.8)
project(new_planners)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(fcl REQUIRED)
find_package(Boost REQUIRED)
find_package(Bullet REQUIRED)
find_package(ompl REQUIRED)
find_package(range-v3 REQUIRED)
find_package(Qhull REQUIRED)
find_package(CGAL REQUIRED)
find_package(VTK REQUIRED)

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
        ${VTK_LIBRARIES}
)

###########
## Build ##
###########

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Og")
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
        src/EndEffectorOnShellGoal.cpp
        src/EndEffectorOnShellGoal.h
        src/ExperimentVisualTools.cpp
        src/ExperimentVisualTools.h
        src/GreatCircleMetric.cpp
        src/GreatCircleMetric.h
        src/HashedSpatialIndex.cpp
        src/HashedSpatialIndex.h
        src/InformedBetweenTwoDroneStatesSampler.h
        src/InformedManipulatorDroneSampler.cpp
        src/InformedRobotStateSampler.cpp
        src/InformedRobotStateSampler.h
        src/LeavesCollisionChecker.cpp
        src/LeavesCollisionChecker.h
        src/RobotPath.cpp
        src/RobotPath.h
        src/SamplerWrapper.cpp
        src/SamplerWrapper.h
        src/ScannablePointsIndex.cpp
        src/ScannablePointsIndex.h
        src/SingleGoalPlannerMethods.cpp
        src/SingleGoalPlannerMethods.h
        src/StreamingConvexHull.cpp
        src/StreamingConvexHull.h
        src/TimedCostConvergenceTerminationCondition.cpp
        src/TimedCostConvergenceTerminationCondition.h
        src/TreeMeshes.cpp
        src/TreeMeshes.h
        src/experiments/exploration.cpp
        src/exploration/ColorEncoding.cpp
        src/exploration/ColorEncoding.h
        src/exploration/DynamicMeshHullAlgorithm.cpp
        src/exploration/DynamicMeshHullAlgorithm.h
        src/exploration/OnlinePointCloudMotionControlAlgorithm.cpp
        src/exploration/OnlinePointCloudMotionControlAlgorithm.h
        src/exploration/SegmentedPointCloud.cpp
        src/exploration/SegmentedPointCloud.h
        src/exploration/StupidAlgorithm.cpp
        src/exploration/VtkToPointCloud.cpp
        src/exploration/VtkToPointCloud.h
        src/exploration/scan_points.cpp
        src/exploration/scan_points.h
        src/ompl_custom.cpp
        src/ompl_custom.h
        src/planners/MultiGoalPlanner.cpp
        src/planners/MultiGoalPlanner.h
        src/planners/MultigoalPrmStar.cpp
        src/planners/MultigoalPrmStar.h
        src/planners/ShellPathPlanner.cpp
        src/planners/ShellPathPlanner.h
        src/planners/shell_path_planner/Construction.cpp
        src/planners/shell_path_planner/Construction.h
        src/planners/shell_path_planner/OrderingMethod.h
        src/planning_scene_diff_message.cpp
        src/planning_scene_diff_message.h
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
        src/utilities/convex_hull.cpp
        src/utilities/convex_hull.h
        src/utilities/experiment_utils.cpp
        src/utilities/experiment_utils.h
        src/utilities/general_utilities.cpp
        src/utilities/general_utilities.h
        src/utilities/geogebra.cpp
        src/utilities/geogebra.h
        src/utilities/json_utils.cpp
        src/utilities/json_utils.h
        src/utilities/load_mesh.cpp
        src/utilities/load_mesh.h
        src/utilities/math_utils.cpp
        src/utilities/math_utils.h
        src/utilities/moveit.cpp
        src/utilities/moveit.cpp
        src/utilities/moveit.h
        src/utilities/moveit.h
        src/utilities/moveit_conversions.cpp
        src/utilities/moveit_conversions.h
        src/utilities/msgs_utilities.cpp
        src/utilities/msgs_utilities.h
        src/utilities/trajectory_primitives.cpp
        src/utilities/trajectory_primitives.h
        src/utilities/traveling_salesman.cpp
        src/utilities/traveling_salesman.h
        src/utilities/vtk.cpp
        src/utilities/vtk.h
        src/vtk/SimulatedSensor.cpp
        src/vtk/SimulatedSensor.h
        src/vtk/Viewer.cpp
        src/vtk/Viewer.h
        src/vtk/VisualizationSpecifics.cpp
        src/vtk/VisualizationSpecifics.h
        src/vtk/VtkRobotModel.cpp
        src/vtk/VtkRobotModel.h
#        src/StreamingAlphaShape.h
#        src/StreamingAlphaShape.cpp
        src/ApproximateConvexDecompositions.cpp
        src/ApproximateConvexDecompositions.h
        src/BruteForceDedupStreamingAlphashape.cpp
        src/BruteForceDedupStreamingAlphashape.h
        src/StreamingMeshHullAlgorithm.cpp
        src/StreamingMeshHullAlgorithm.h src/utilities/mesh_utils.h src/utilities/mesh_utils.cpp)

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
experiment("exploration")

include(GoogleTest)

add_executable(${PROJECT_NAME}_tests
        test/test.cpp
        test/MoveItPathLengthObjectiveTest.cpp
        test/math_test.cpp
        test/CuttingPlaneConvexHullShell_test.cpp
        test/CGALMeshShell_test.cpp
        src/utilities/test/math_utils_test.cpp
        test/SphereShell_test.cpp
        test/StreamingConvexHull_test.cpp
        test/VisualiserTest.cpp
        test/TimedConvergenceTerminationCondition_test.cpp
        test/HashedSpatialIndex_test.cpp
        test/LeavesCollisionChecker_test.cpp
        test/LeavesCollisionChecker_test.cpp
        test/OnlineOrderOptimization_test.cpp
#        test/StreamingAlphaShape_test.cpp
        )

target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_shared gtest)
ament_target_dependencies(${PROJECT_NAME}_tests ${AMENT_DEPS})
gtest_discover_tests(${PROJECT_NAME}_tests)