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
