cmake_minimum_required(VERSION 3.8)
project(MgODPL) # Multigoal Orchard Drone Planning Library

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(VTK_QT_VERSION "6" CACHE STRING "Qt Version")

# Add an option to enable/disable VTK
option(ENABLE_VISUALIZATION "Enable VISUALizATION support" OFF)

# Add an option to enable/disable building the experiments
option(ENABLE_EXPERIMENTS "Enable EXPERIMENTS" OFF)

# Add an option to build tests or not.
option(ENABLE_TESTS "Enable TESTS" OFF)

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)
find_package(Boost REQUIRED)
find_package(Bullet REQUIRED)
find_package(ompl REQUIRED)
find_package(range-v3 REQUIRED)
find_package(Qhull REQUIRED)
find_package(ortools REQUIRED)

set(CGAL_DATA_DIR "UNUSED_STOP_WARNING")
find_package(CGAL REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(jsoncpp REQUIRED)

# Ament seems to complain about moveit_planners_ompl exporting a non-existent include directory, see https://github.com/lopsided98/nix-ros-overlay/issues/312
# Is there a way to get this to be at least semi-nonbroken?
set(HACKY_MOVEIT_INCLUDE /nix/store/mg432iy0715cc7fylwg10bsdhsg4qy30-ros-rolling-moveit-planners-ompl-2.8.0-r2/include/moveit_planners)
set(HACKY_MOVEIT_LINK /nix/store/mg432iy0715cc7fylwg10bsdhsg4qy30-ros-rolling-moveit-planners-ompl-2.8.0-r2/)

include_directories(
#        ${EIGEN3_INCLUDE_DIRS}
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
        ${HACKY_MOVEIT_INCLUDE}
)


link_libraries(
#        ${EIGEN_LIBRARIES}
        ${FCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${BULLET_LIBRARIES}
        ${OMPL_LIBRARIES}
        ${VTK_LIBRARIES}
        qhullcpp
        HACD # Note: the HACD library is included Bullet's "extra" libraries
        jsoncpp_lib
#        tbb
        ${CGAL_LIBRARIES}
#        ${CMAKE_SOURCE_DIR}/ortools/lib/libortools.so
        moveit_ompl_interface
)

###########
## Build ##
###########

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Og")
#SET(CMAKE_C_FLAGS_DEBUG "-O0")

set(AMENT_DEPS "moveit_core" "moveit_msgs" "shape_msgs" "rclcpp" "ompl")

# A small library featuring some vector math (not depending on Eigen)
# and some general-purpose geometry utilities.
add_library(math_utils
        src/math/AABBGrid.h
        src/math/AABBGrid.cpp
        src/math/AABB.h
        src/math/Vec3.h
        src/math/grid_utils.h
        src/math/grid_utils.cpp
        src/math/vecmath_utils.h
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
#target_compile_options(visibility PRIVATE -O3)

if (ENABLE_EXPERIMENTS)
    # A library that is mostly useful for running the experiments.
    add_library(experiment_utils
            src/experiment_utils/TreeMeshes.h
            src/experiment_utils/TreeMeshes.cpp
            src/experiment_utils/mesh_connected_components.cpp
            src/experiment_utils/mesh_connected_components.h
            src/experiment_utils/load_mesh.h
            src/experiment_utils/load_mesh.cpp
            src/experiment_utils/load_robot_model.h
            src/experiment_utils/load_robot_model.cpp
    )

    target_compile_definitions(experiment_utils PRIVATE MYSOURCE_ROOT="${CMAKE_SOURCE_DIR}")

    target_link_libraries(experiment_utils math_utils visibility)
    ament_target_dependencies(experiment_utils ${AMENT_DEPS})
endif()

if (ENABLE_VISUALIZATION)

    message(STATUS "Building with visualization")

    find_package(VTK REQUIRED)
    include_directories(${VTK_INCLUDE_DIRS})

    # A library for vizualizing the experiments.
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
    )

    target_link_directories(${PROJECT_NAME}_visualisation PUBLIC ${VTK_LIBRARY_DIRS})
    ament_target_dependencies(${PROJECT_NAME}_visualisation "moveit_core" "moveit_msgs" "shape_msgs" "rclcpp" "ompl")
    target_link_libraries(${PROJECT_NAME}_visualisation ${VTK_LIBRARIES} experiment_utils)

    add_executable(vizcube_lightweight src/experiments/cubeviz.cpp)
    target_link_libraries(vizcube_lightweight math_utils ${PROJECT_NAME}_visualisation experiment_utils)
    ament_target_dependencies(vizcube_lightweight shape_msgs)

    add_executable(no_preknowledge src/experiments/no_preknowledge.cpp)
    target_link_libraries(no_preknowledge math_utils ${PROJECT_NAME}_visualisation experiment_utils)
    ament_target_dependencies(no_preknowledge shape_msgs)

endif ()

include(GoogleTest)

add_executable(${PROJECT_NAME}_tests
        test/test.cpp
        test/math/AABBGrid_test.cpp
        test/math/AABB_test.cpp
        test/experiment_utils/voxel_visibility_test.cpp
        test/math/intersection_test.cpp
        test/math/lp_test.cpp
)

target_link_libraries(${PROJECT_NAME}_tests math_utils experiment_utils gtest)
gtest_discover_tests(${PROJECT_NAME}_tests)
