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
    const double MAX_DISTANCE = INFINITY;
    const double MIN_DISTANCE = 0;
    const double MAX_ANGLE = M_PI / 3.0;
    std::vector<ScannablePoints> all_scannable_points;

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
        viewer.addMesh(fruit_mesh, {1.0, 0.0, 0.0}, 1.0);
    }

    // Set the camera transform for the viewer
    viewer.setCameraTransform({2.0, 1.0, 1.0}, {0.0, 0.0, 0.0});

    const auto& robot = experiments::createProceduralRobotModel();

    // Create a state outside the tree model.
    RobotState initial_state = fromEndEffectorAndVector(robot, {0, 5, 5}, {0, 1, 1});

    // Plan the final path as a whole:
    RobotPath final_path = plan_multigoal_path(robot, tree_model, initial_state);

    // Start the viewer
    viewer.start();
}
