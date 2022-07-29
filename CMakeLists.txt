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
        ortools/include
        miniball/cpp/main
)

link_libraries(
        ${EIGEN_LIBRARIES}
        ${FCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${BULLET_LIBRARIES}
        ${OMPL_LIBRARIES}
        qhullcpp
        HACD # Note: the HACD library is included Bullet's "extra" libraries
        jsoncpp_lib
        tbb
        ${CMAKE_SOURCE_DIR}/ortools/lib/libortools.so
)

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#SET(CMAKE_CXX_FLAGS_DEBUG "-O0")
#SET(CMAKE_C_FLAGS_DEBUG "-O0")

add_library(${PROJECT_NAME}_shared
        src/BulletContinuousMotionValidator.cpp
        src/BulletContinuousMotionValidator.h
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
        src/GreatCircleMetric.cpp
        src/GreatCircleMetric.h
        src/InformedBetweenTwoDroneStatesSampler.h
        src/InformedManipulatorDroneSampler.cpp
        src/InformedRobotStateSampler.cpp
        src/InformedRobotStateSampler.h
        src/LeavesCollisionChecker.cpp
        src/LeavesCollisionChecker.h
        src/SamplerWrapper.cpp
        src/SamplerWrapper.h
        src/SingleGoalPlannerMethods.cpp
        src/SingleGoalPlannerMethods.h
        src/SphereShell.cpp
        src/SphereShell.h
        src/TimedCostConvergenceTerminationCondition.cpp
        src/TimedCostConvergenceTerminationCondition.h
        src/UnionGoalSampleableRegion.cpp
        src/UnionGoalSampleableRegion.h
        src/experiment_utils.cpp
        src/experiment_utils.h
        src/general_utilities.cpp
        src/general_utilities.h
        src/json_utils.cpp
        src/json_utils.h
        src/moveit_conversions.cpp
        src/moveit_conversions.h
        src/msgs_utilities.cpp
        src/msgs_utilities.h
        src/ompl_custom.cpp
        src/ompl_custom.h
        src/planners/MultiGoalPlanner.cpp
        src/planners/MultiGoalPlanner.h
        src/planners/MultigoalPrmStar.cpp
        src/planners/MultigoalPrmStar.h
        src/planners/ShellPathPlanner.cpp
        src/planners/ShellPathPlanner.h
        src/planning_scene_diff_message.cpp
        src/planning_scene_diff_message.h
        src/probe_retreat_move.cpp
        src/probe_retreat_move.h
        src/procedural_tree_generation.cpp
        src/robot_path.cpp
        src/robot_path.h
        src/run_experiment.cpp
        src/run_experiment.h
        src/traveling_salesman.cpp
        src/traveling_salesman.h
#        src/NewKnnPlanner.cpp
#        src/NewKnnPlanner.h
        src/ConvexHullShell.cpp src/ConvexHullShell.h src/convex_hull.cpp src/convex_hull.h)

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
experiment("shellpath_convexhull_vs_sphere")
experiment("tsp_over_prm")
experiment("tsp_over_prm_vs_sphereshell")

include(GoogleTest)

add_executable(${PROJECT_NAME}_tests
        test/test.cpp
        test/MoveItPathLengthObjectiveTest.cpp
        )
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_shared gtest)
ament_target_dependencies(${PROJECT_NAME}_tests ${AMENT_DEPS})
gtest_discover_tests(${PROJECT_NAME}_tests)