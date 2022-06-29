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
        ortools/include
)

link_libraries(
        ${EIGEN_LIBRARIES}
        ${FCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${BULLET_LIBRARIES}
        ${OMPL_LIBRARIES}
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
        src/ClearanceDecreaseMinimizationObjective.cpp
        src/ClearanceDecreaseMinimizationObjective.h
        src/DroneStateConstraintSampler.cpp
        src/DroneStateSampler.cpp
        src/DroneStateSampler.h
        src/experiment_utils.cpp
        src/experiment_utils.h
        src/general_utilities.cpp
        src/general_utilities.h
        src/InformedRobotStateSampler.cpp
        src/InformedRobotStateSampler.h
        src/InformedManipulatorDroneSampler.cpp
        src/InformedBetweenTwoDroneStatesSampler.h
        src/InverseClearanceIntegralObjective.cpp
        src/InverseClearanceIntegralObjective.h
        src/json_utils.cpp
        src/json_utils.h
        src/LeavesCollisionChecker.cpp
        src/LeavesCollisionChecker.h
        src/DronePathLengthObjective.cpp
        src/DronePathLengthObjective.h
        src/msgs_utilities.cpp
        src/msgs_utilities.h
        src/multigoal/approach_table.cpp
        src/multigoal/approach_table.h
        src/multigoal/ATNN.cpp
        src/multigoal/ATNN.h
        src/multigoal/ATRandom.cpp
        src/multigoal/ATRandom.h
        src/multigoal/goals_gnat.cpp
        src/multigoal/goals_gnat.h
        src/multigoal/knn.cpp
        src/multigoal/knn.h
        src/multigoal/MetricTwoOpt.cpp
        src/multigoal/MetricTwoOpt.h
        src/multigoal/multi_goal_planners.cpp
        src/multigoal/multi_goal_planners.h
        src/multigoal/PointToPointPlanner.cpp
        src/multigoal/PointToPointPlanner.h
        src/multigoal/RandomizedTwoOpt.cpp
        src/multigoal/RandomizedTwoOpt.h
        src/multigoal/random_order.cpp
        src/multigoal/random_order.h
        src/multigoal/clustering_preselection.cpp
        src/multigoal/clustering_preselection.h
        src/multigoal/visitation_order_strategy.cpp
        src/multigoal/visitation_order_strategy.h
        src/multigoal/in_cluster_distances.cpp
        src/multigoal/in_cluster_distances.h
        src/multigoal/clustering_density.cpp
        src/multigoal/clustering_density.h
        src/multigoal/uknn.cpp
        src/multigoal/uknn.h
        src/ompl_custom.cpp
        src/ompl_custom.h
        src/planning_scene_diff_message.cpp
        src/planning_scene_diff_message.h
        src/preorder_strategy/preorder_strategy.cpp
        src/preorder_strategy/preorder_strategy.h
        src/procedural_tree_generation.cpp
        src/SamplerWrapper.cpp
        src/SamplerWrapper.h
        src/UnionGoalSampleableRegion.cpp
        src/UnionGoalSampleableRegion.h
        src/greatcircle.h
        src/greatcircle.cpp
#        src/agglomerative_clustering.cpp
#        src/agglomerative_clustering.h
        src/MultigoalPrmStar.cpp
        src/MultigoalPrmStar.h
        src/traveling_salesman.h
        src/traveling_salesman.cpp
        src/probe_retreat_move.cpp
        src/probe_retreat_move.h
        src/moveit_conversions.cpp
        src/moveit_conversions.h
        src/SphereShell.cpp
        src/SphereShell.h
        src/EndEffectorOnShellGoal.cpp
        src/EndEffectorOnShellGoal.h
        #        src/ExperimentVisualTools.h
#        src/ExperimentVisualTools.cpp
        src/functional_optional.h
        src/MultigoalPrmStar.cpp
        src/MultigoalPrmStar.h
        src/robot_path.cpp
        src/robot_path.h
        src/NewMultiGoalPlanner.cpp
        src/NewMultiGoalPlanner.h
        src/NewKnnPlanner.cpp
        src/NewKnnPlanner.h
        src/DistanceHeuristics.cpp
        src/DistanceHeuristics.h
        src/SingleGoalPlannerMethods.cpp
        src/SingleGoalPlannerMethods.h
        src/ShellPathPlanner.cpp
        src/ShellPathPlanner.h
        src/run_experiment.cpp
        src/run_experiment.h
        src/MyBitStar.cpp
        src/MyBitStar.h
#        src/MultigoalLazyPrmStar.cpp
#        src/MultigoalLazyPrmStar.h
        )

set(AMENT_DEPS "moveit_core" "moveit_msgs" "shape_msgs" "moveit_planners_ompl" "rclcpp" "ompl")

ament_target_dependencies(${PROJECT_NAME}_shared ${AMENT_DEPS})

#
##message(FATAL_ERROR ${catkin_INCLUDE_DIRS})
#
### Visualization node
#
#add_executable(${PROJECT_NAME}_ptp_correlation experiments/ptp_correlation.cpp)
#
#add_executable(${PROJECT_NAME}_euclidean_tsp experiments/euclidean_tsp.cpp)
#
#add_executable(${PROJECT_NAME}_ptp_comparison experiments/ptp_comparison.cpp)
#target_link_libraries(${PROJECT_NAME}_ptp_comparison ${PROJECT_NAME}_shared)
#
#add_executable(${PROJECT_NAME}_prmstar_keep_or_clear experiments/prmstar_keep_or_clear.cpp)
#target_link_libraries(${PROJECT_NAME}_prmstar_keep_or_clear ${PROJECT_NAME}_shared)
#
#add_executable(${PROJECT_NAME}_greatcircle_correlation experiments/greatcircle_correlation.cpp)
#target_link_libraries(${PROJECT_NAME}_greatcircle_correlation ${PROJECT_NAME}_shared)
#
#add_executable(${PROJECT_NAME}_preorder_experiment experiments/preorder_experiment.cpp)
#target_link_libraries(${PROJECT_NAME}_preorder_experiment ${PROJECT_NAME}_shared)

add_executable(${PROJECT_NAME}_probe_retreat_move experiments/probe_retreat_move.cpp)
target_link_libraries(${PROJECT_NAME}_probe_retreat_move ${PROJECT_NAME}_shared)
ament_target_dependencies(${PROJECT_NAME}_probe_retreat_move ${AMENT_DEPS})

add_executable(${PROJECT_NAME}_prm_multigoal_experiment experiments/prm_multigoal_experiment.cpp)
target_link_libraries(${PROJECT_NAME}_prm_multigoal_experiment ${PROJECT_NAME}_shared)
ament_target_dependencies(${PROJECT_NAME}_prm_multigoal_experiment ${AMENT_DEPS})

#
#add_executable(${PROJECT_NAME}_approach_planning experiments/approach_planning.cpp)
#target_link_libraries(${PROJECT_NAME}_approach_planning ${PROJECT_NAME}_shared)
#
#add_executable(${PROJECT_NAME}_knn experiments/knn.cpp)
#target_link_libraries(${PROJECT_NAME}_knn ${PROJECT_NAME}_shared)
#
#add_executable(${PROJECT_NAME}_generate_tree_data src/generate_tree_data.cpp)
#
##target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_shared)
#
#
##target_link_libraries(${PROJECT_NAME}_tests ${CATKIN_LIBRARIES} ${EIGEN_LIBRARIES} ${FCL_LIBRARIES} ${Boost_LIBRARIES} jsoncpp_lib)
#
include(GoogleTest)

add_executable(${PROJECT_NAME}_tests
        test/test.cpp
        test/MoveItPathLengthObjectiveTest.cpp
        )
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_shared gtest)
ament_target_dependencies(${PROJECT_NAME}_tests ${AMENT_DEPS})
gtest_discover_tests(${PROJECT_NAME}_tests)