cmake_minimum_required(VERSION 3.0.2)
project(new_planners)

find_package(catkin
        REQUIRED
        COMPONENTS
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_visual_tools
        rviz_visual_tools
        roscpp
        geometry_msgs
        moveit_msgs
        ompl
        urdf
        robowflex_library
        robowflex_ompl)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

find_package(jsoncpp REQUIRED)

find_package(fcl REQUIRED)


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
        src/procedural_tree_generation.cpp
        src/EndEffectorConstraintSampler.cpp
        src/msgs_utilities.cpp
        src/msgs_utilities.h
        src/build_planning_scene.cpp
        src/build_planning_scene.h
        src/ClearanceDecreaseMinimizationObjective.h
        src/ClearanceDecreaseMinimizationObjective.cpp
        src/make_robot.h
        src/make_robot.cpp
        src/init_planner.h
        src/init_planner.cpp
        src/BulletContinuousMotionValidator.h
        src/BulletContinuousMotionValidator.cpp
        src/InverseClearanceIntegralObjective.cpp
        src/InverseClearanceIntegralObjective.h
        src/MyOMPLPlanner.cpp
        src/MyOMPLPlanner.h
        src/ompl_custom.cpp
        src/ompl_custom.h
        src/UnionGoalSampleableRegion.cpp
        src/UnionGoalSampleableRegion.h
        src/LeavesCollisionChecker.cpp
        src/LeavesCollisionChecker.h
        src/json_utils.cpp
        src/json_utils.h
        src/multigoal/multi_goal_planners.h
        src/multigoal/multi_goal_planners.cpp
        src/multigoal/knn.h
        src/multigoal/knn.cpp
        src/multigoal/random_order.h
        src/multigoal/random_order.cpp
        src/multigoal/uknn.h
        src/multigoal/uknn.cpp
        src/multigoal/cluster_breaking.cpp
        src/multigoal/cluster_breaking.h)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${FCL_INCLUDE_DIRS}
        ${JSONCPP_INCLUDE_DIRS}
)

## Visualization node
add_executable(${PROJECT_NAME}_vizualized
        src/run_vizualized.cpp ${SHARED_SRC})
set_property(TARGET ${PROJECT_NAME}_vizualized PROPERTY CXX_STANDARD 17)

target_link_libraries(${PROJECT_NAME}_vizualized
        ${catkin_LIBRARIES}
        ${EIGEN_LIBRARIES}
        ${FCL_LIBRARIES}
        jsoncpp_lib
        )

## Testing
add_executable(${PROJECT_NAME}_tests
        test/tests.cpp ${SHARED_SRC})
set_property(TARGET ${PROJECT_NAME}_tests PROPERTY CXX_STANDARD 17)

target_link_libraries(${PROJECT_NAME}_tests
        ${catkin_LIBRARIES}
        ${EIGEN_LIBRARIES}
        ${FCL_LIBRARIES}
        jsoncpp_lib
        gtest
        gtest_main
        )