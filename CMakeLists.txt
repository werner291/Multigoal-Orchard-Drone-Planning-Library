cmake_minimum_required(VERSION 3.0.2)
project(new_planners)

find_package(catkin
        REQUIRED
        COMPONENTS
        moveit_core
        moveit_ros_planning
#        moveit_ros_planning_interface
        moveit_planners_ompl
        moveit_visual_tools
        rviz_visual_tools
        roscpp
        geometry_msgs
        moveit_msgs
        eigen_conversions
        ompl
        urdf
        )
#find_package(GTest REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

find_package(jsoncpp REQUIRED)

find_package(fcl REQUIRED)

find_package(Boost REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES new_planners
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

###########
## Build ##
###########

set(SHARED_SRC
        src/BetweenMoveItStatesInformedSampler.cpp
        src/BetweenMoveItStatesInformedSampler.h
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
        src/InformedManipulatorDroneSampler.h
        src/InverseClearanceIntegralObjective.cpp
        src/InverseClearanceIntegralObjective.h
        src/json_utils.cpp
        src/json_utils.h
        src/LeavesCollisionChecker.cpp
        src/LeavesCollisionChecker.h
        src/ManipulatorDroneMoveitPathLengthObjective.cpp
        src/ManipulatorDroneMoveitPathLengthObjective.h
        src/msgs_utilities.cpp
        src/msgs_utilities.h
        src/multigoal/approach_table.cpp
        src/multigoal/approach_table.h
        src/multigoal/AT2Opt.cpp
        src/multigoal/AT2Opt.h
        src/multigoal/ATNN.cpp
        src/multigoal/ATNN.h
        src/multigoal/ATRandom.cpp
        src/multigoal/ATRandom.h
        src/multigoal/ClusterTable.cpp
        src/multigoal/ClusterTable.h
        #        src/multigoal/GATMatrix.cpp
        #        src/multigoal/GATMatrix.h
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
        #        src/multigoal/SubtourMovingPlanner.cpp
        #        src/multigoal/SubtourMovingPlanner.h
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
        )

#message(FATAL_ERROR ${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${FCL_INCLUDE_DIRS}
        ${JSONCPP_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GTEST_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${FCL_LIBRARIES} ${Boost_LIBRARIES} jsoncpp_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Visualization node
add_executable(${PROJECT_NAME}_vizualized src/run_vizualized.cpp ${SHARED_SRC})

add_executable(${PROJECT_NAME}_benchmark src/run_benchmarked.cpp ${SHARED_SRC} experiments/bnb_comparison.cpp)

add_executable(${PROJECT_NAME}_at_benchmark experiments/at_benchmark.cpp ${SHARED_SRC})

add_executable(${PROJECT_NAME}_ptp_correlation experiments/ptp_correlation.cpp ${SHARED_SRC})

add_executable(${PROJECT_NAME}_euclidean_tsp experiments/euclidean_tsp.cpp ${SHARED_SRC})

add_executable(${PROJECT_NAME}_generate_tree_data src/generate_tree_data.cpp ${SHARED_SRC})

catkin_add_gtest(${PROJECT_NAME}_tests test/test.cpp
        test/test_utils.h
        test/test_utils.cpp
        test/informed_sampler_test.cpp
#        #test/BulletContinuousMotionValidatorTest.cpp
#        test/JsonTests.cpp
#        #        test/MoveitPathLengthObjectiveTest.cpp
       test/clustering_tests.cpp
#        test/collision_checking_test.cpp
       test/ptp_tests.cpp
        ${SHARED_SRC}
        )

#target_link_libraries(${PROJECT_NAME}_tests ${CATKIN_LIBRARIES} ${EIGEN_LIBRARIES} ${FCL_LIBRARIES} ${Boost_LIBRARIES} jsoncpp_lib)

#include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_tests)
