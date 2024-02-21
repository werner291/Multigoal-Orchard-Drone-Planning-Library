// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#include <vector>
#include <random_numbers/random_numbers.h>

#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/mesh_utils.h"
#include "../visualization/VtkPolyLineVisualization.h"
#include "../visualization/visualization_function_macros.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/scan_path_generators.h"
#include "../visualization/scannable_points.h"
#include "../planning/RobotPath.h"
#include "../planning/probing_motions.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/state_tools.h"
#include "../planning/approach_path_planning.h"
#include "../experiment_utils/fcl_utils.h"

#include <fcl/narrowphase/collision.h>

#include "../planning/collision_detection.h"
#include "../planning/goal_sampling.h"
#include "../planning/shell_path_assembly.h"
#include "../planning/spherical_geometry.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(fruit_scan_fullpath)
{
    // Load the tree meshes
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Initialize a random number generator
    random_numbers::RandomNumberGenerator rng;

    // Define constants for the scannable points
    const size_t NUM_POINTS = 200;
    const double MAX_DISTANCE = 0.3;
    const double MIN_DISTANCE = 0;
    const double MAX_ANGLE = M_PI / 3.0;

    std::vector<ScannablePoints> all_scannable_points;
    std::vector<VtkLineSegmentsVisualization> fruit_points_visualizations;

    viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

    // Iterate over all the fruit meshes in the tree model
    for (const auto& fruit_mesh : tree_model.fruit_meshes)
    {
        // Create the scannable points
        ScannablePoints scannable_points = createScannablePoints(
            rng,
            fruit_mesh, NUM_POINTS, MAX_DISTANCE, MIN_DISTANCE,
            MAX_ANGLE);

        // Add the created scannable points to the vector
        all_scannable_points.push_back(scannable_points);

        // Create the fruit points visualization
        VtkLineSegmentsVisualization fruit_points_visualization = createFruitLinesVisualization(scannable_points);

        // Add the fruit points visualization to the viewer
        viewer.addActor(fruit_points_visualization.getActor());

        // Add the fruit mesh to the viewer
        viewer.addMesh(fruit_mesh, {0.8, 0.8, 0.8}, 1.0);

        fruit_points_visualizations.push_back(std::move(fruit_points_visualization));
    }

    // Set the camera transform for the viewer
    viewer.setCameraTransform({2.0, 1.0, 1.0}, {0.0, 0.0, 0.0});

    const auto& robot = experiments::createProceduralRobotModel();

    robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

    // Create a state outside the tree model.
    RobotState initial_state = fromEndEffectorAndVector(robot, {0, 5, 5}, {0, 1, 1});

    std::vector<SeenPoints> ever_seen;
    ever_seen.reserve(all_scannable_points.size());
    for (const auto& scannable_points : all_scannable_points)
    {
        ever_seen.push_back(SeenPoints::create_all_unseen(scannable_points));
    }

    // Plan the final path as a whole:
    RobotPath final_path = plan_multigoal_path(robot, tree_model, initial_state);

    PathPoint path_point = {0, 0.0};

    // Create an instance of VtkPolyLineVisualization
    VtkPolyLineVisualization end_effector_positions_visualization(1, 0.5, 1); // Red color

    // Add the polyline visualization to the viewer
    viewer.addActor(end_effector_positions_visualization.getActor());

    // Declare a vector to store the end-effector positions
    std::vector<mgodpl::math::Vec3d> end_effector_positions;

    auto robot_viz = mgodpl::vizualisation::vizualize_robot_state(viewer, robot,
                                                                  robot_model::forwardKinematics(
                                                                      robot, initial_state.joint_values,
                                                                      flying_base, initial_state.base_tf));

    auto timerCallback = [&]()
    {
        const auto& start_state = final_path.states[path_point.segment_i];
        const auto& end_state = final_path.states[path_point.segment_i + 1];

        // Segment length:
        double segment_length = equal_weights_max_distance(start_state, end_state);

        // Advance the path point
        bool finished = advancePathPointClamp(final_path, path_point, 0.01, equal_weights_max_distance);

        auto interpolated_state = interpolate(start_state, end_state, path_point.segment_t);

        auto fk = robot_model::forwardKinematics(robot, interpolated_state.joint_values,
                                                 robot.findLinkByName("flying_base"), interpolated_state.base_tf);
        const math::Vec3d ee_pos = fk.forLink(robot.findLinkByName("end_effector")).translation;

        // Add the new end-effector position to the vector
        end_effector_positions.push_back(ee_pos);

        // Update the polyline with the new set of end-effector positions
        end_effector_positions_visualization.updateLine(end_effector_positions);

        // Update the robot's state in the visualization
        update_robot_state(robot, fk, robot_viz);

        size_t newly_seen = 0;

        for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i)
        {
            newly_seen += update_visibility(all_scannable_points[fruit_i], ee_pos, ever_seen[fruit_i]);

            // Update the colors of the fruit points visualization
            fruit_points_visualizations[fruit_i].setColors(generateVisualizationColors(ever_seen[fruit_i]));
        }

        if (finished)
        {
            viewer.stop();

            size_t points_seen = 0;
            size_t points_total = 0;
            for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i)
            {
                points_seen += ever_seen[fruit_i].count_seen();
                points_total += all_scannable_points[fruit_i].surface_points.size();
            }
            std::cout << "Seen " << points_seen << " out of " << points_total << " points" << std::endl;
        }
    };

    viewer.setCameraTransform({5.0, 5.0, 2.0}, {0.0, 0.0, 2.5});

    viewer.addTimerCallback(timerCallback);
    viewer.start();
}

/**
 * @enum Direction
 * @brief An enumeration to represent the direction of scanning motion.
 */
enum class Direction {
    LEFT,   ///< Represents the left direction.
    RIGHT   ///< Represents the right direction.
};

/**
 * @fn RobotPath generateSidewaysScanningMotion(const robot_model::RobotModel& robot, const fcl::CollisionObjectd& treeTrunkObject, const mgodpl::math::Vec3d& fruitCenter, double initialLongitude, double initialLatitude, double scanDistance, Direction direction)
 * @brief Generates a sideways scanning motion for a robot.
 *
 * This function generates a sideways scanning motion for a robot. The motion is generated based on the provided parameters.
 * The direction of the motion can be either left or right, as specified by the Direction enum parameter.
 *
 * The motion is at most half a circle, or until the first collision is detected.
 *
 * @param robot The robot model.
 * @param treeTrunkObject The tree trunk object used for collision detection.
 * @param fruitCenter The center of the fruit to be scanned.
 * @param initialLongitude The initial longitude for the scanning motion.
 * @param initialLatitude The initial latitude for the scanning motion.
 * @param scanDistance The distance to scan from the fruit center.
 * @param direction The direction of the scanning motion (either LEFT or RIGHT).
 *
 * @return A RobotPath object representing the generated scanning motion.
 */
RobotPath generateSidewaysScanningMotion(const robot_model::RobotModel& robot,
                                         const fcl::CollisionObjectd& treeTrunkObject,
                                         const mgodpl::math::Vec3d& fruitCenter,
                                         double initialLongitude,
                                         double initialLatitude,
                                         double scanDistance,
                                         Direction direction)
{
    const int MAX_ITERATIONS = 32;  ///< The maximum number of iterations for the scanning motion.
    double STEP_SIZE = M_PI / (double)MAX_ITERATIONS;  ///< The step size for each iteration of the scanning motion.

    // Adjust the step size based on the direction
    if (direction == Direction::LEFT) {
        STEP_SIZE = -STEP_SIZE;
    }

    std::vector<RobotState> scanningMotionStates;  ///< A vector to store the states of the robot during the scanning motion.
    for (int i = 1; i < MAX_ITERATIONS; ++i)
    {
        auto relativeVertex = spherical_geometry::RelativeVertex{
            .longitude = initialLongitude + i * STEP_SIZE, .latitude = initialLatitude
        }.to_cartesian();

        // If collision-free, add to the list:
        if (!check_robot_collision(robot, treeTrunkObject, fromEndEffectorAndVector(robot,
                                       fruitCenter + relativeVertex.normalized() * scanDistance,
                                       relativeVertex)))
        {
            scanningMotionStates.push_back(fromEndEffectorAndVector(robot,
                                                                    fruitCenter + relativeVertex.normalized() *
                                                                    scanDistance,
                                                                    relativeVertex));
        }
        else
        {
            // Stop generating states after the first collision
            break;
        }
    }
    return RobotPath{scanningMotionStates};  ///< Return the generated scanning motion as a RobotPath object.
}

REGISTER_VISUALIZATION(single_fruit_scan)
{
    // Load the tree meshes
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Initialize a random number generator
    random_numbers::RandomNumberGenerator rng(42);

    // Randomly select one fruit mesh
    int random_index = rng.uniformInteger(0, tree_model.fruit_meshes.size() - 1);
    auto& fruit_mesh = tree_model.fruit_meshes[random_index];

    // Define constants for the scannable points
    const size_t NUM_POINTS = 200;
    const double MAX_DISTANCE = 0.3;
    const double MIN_DISTANCE = 0;
    const double MAX_ANGLE = M_PI / 3.0;

    // Create the scannable points
    ScannablePoints scannable_points = createScannablePoints(
        rng,
        fruit_mesh, NUM_POINTS, MAX_DISTANCE, MIN_DISTANCE,
        MAX_ANGLE);

    // Create a robot model
    const auto& robot = experiments::createProceduralRobotModel();
    const robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
    const robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");

    math::Vec3d fruit_center = mesh_aabb(fruit_mesh).center();

    // Allocate a BVH convex_hull for the tree trunk.
    const auto& tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
    fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

    // First, create the convex hull.
    cgal::CgalMeshData mesh_data(tree_model.leaves_mesh);

    const double EE_SCAN_DISTANCE = 0.1;

    std::optional<RobotState> sample = generateUniformRandomArmVectorState(
        robot,
        tree_trunk_object,
        fruit_center,
        rng,
        1000,
        EE_SCAN_DISTANCE
    );

    if (!sample.has_value())
    {
        throw std::runtime_error("Failed to find a collision-free state");
    }

    // Get the arm vector from the sample:
    auto sample_fk = forwardKinematics(robot, sample->joint_values, flying_base, sample->base_tf);
    auto fruit_to_ee = sample_fk.forLink(end_effector).translation - fruit_center;

    // Get the lat/long angles of the arm vector:
    double lat_angle = spherical_geometry::latitude(fruit_to_ee);
    double long_angle = spherical_geometry::longitude(fruit_to_ee);

    // Generate a few additional samples by increasing the long angle:
    RobotPath scan_motion = generateSidewaysScanningMotion(
        robot,
        tree_trunk_object,
        fruit_center,
        long_angle,
        lat_angle,
        EE_SCAN_DISTANCE,
        Direction::LEFT
    );

    // Then, try the straight-out motion:
    auto approach_path = straightout(robot, *sample, mesh_data.tree, mesh_data.mesh_path);

    PathPoint collision_point{};
    if (check_path_collides(robot, tree_trunk_object, approach_path.path, collision_point))
    {
        throw std::runtime_error("Straight-out motion collides");
    }

    RobotPath path = approach_path.path;

    // Append the scan motion to the path.
    path.states.insert(path.states.end(), scan_motion.states.begin(), scan_motion.states.end());
    // And back:
    path.states.insert(path.states.end(), scan_motion.states.rbegin(), scan_motion.states.rend());

    // Same but to the right now:
    RobotPath scan_motion_right = generateSidewaysScanningMotion(
        robot,
        tree_trunk_object,
        fruit_center,
        long_angle,
        lat_angle,
        EE_SCAN_DISTANCE,
        Direction::RIGHT
    );

    path.states.insert(path.states.end(), scan_motion_right.states.begin(), scan_motion_right.states.end());
    path.states.insert(path.states.end(), scan_motion_right.states.rbegin(), scan_motion_right.states.rend());

    // Define the current position on the path
    PathPoint path_point = {0, 0.0};

    // Define the speed of interpolation
    double interpolation_speed = 0.02;

    RobotState initial_state = path.states[0];

    // Create the fruit points visualization
    VtkLineSegmentsVisualization fruit_points_visualization = createFruitLinesVisualization(scannable_points);

    // Add the fruit points visualization to the viewer
    viewer.addActor(fruit_points_visualization.getActor());

    // Add the fruit mesh to the viewer
    viewer.addMesh(fruit_mesh, {0.8, 0.8, 0.8}, 1.0);

    // Add the tree trunk mesh to the viewer
    viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

    // Set the camera transform for the viewer
    viewer.setCameraTransform({2.0, 1.0, 1.0}, {0.0, 0.0, 0.0});

    // Visualize the robot state
    auto robot_viz = vizualisation::vizualize_robot_state(viewer, robot,
                                                          forwardKinematics(
                                                              robot, initial_state.joint_values,
                                                              flying_base, initial_state.base_tf));

    // Initialize a vector to keep track of which points have been seen
    SeenPoints points_seen = SeenPoints::create_all_unseen(scannable_points);

    // Register the timer callback function to be called at regular intervals
    viewer.addTimerCallback([&]()
    {
        // Advance the path point along the path
        advancePathPointClamp(path, path_point, interpolation_speed, equal_weights_max_distance);

        // Interpolate the robot's state
        auto interpolated_state = interpolate(path_point, path);

        // Update the robot's state in the visualization
        const auto fk = forwardKinematics(robot, interpolated_state.joint_values,
                                          robot.findLinkByName("flying_base"), interpolated_state.base_tf);

        update_robot_state(robot, fk, robot_viz);

        // Get the position of the robot's end effector
        const auto& end_effector_position = fk.forLink(robot.findLinkByName("end_effector")).translation;

        update_visibility(scannable_points, end_effector_position, points_seen);

        // Update the colors of the fruit points visualization
        fruit_points_visualization.setColors(generateVisualizationColors(points_seen));
    });
    viewer.setCameraTransform(fruit_center + math::Vec3d{1.5, 0.0, 1.0}, fruit_center);

    // Start the viewer
    viewer.start();
}
