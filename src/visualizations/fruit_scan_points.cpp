// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <map>
#include <random_numbers/random_numbers.h>
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/mesh_utils.h"
#include "../experiment_utils/scan_paths.h"
#include "../visualization/VtkPolyLineVisualization.h"
#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/scan_path_generators.h"

using namespace mgodpl;

/**
   * @brief Creates a VtkLineSegmentsVisualization object for fruit points.
   *
   * This function creates a VtkLineSegmentsVisualization object that represents
   * the fruit points in the 3D space. Each point is represented as a line segment
   * that starts at the point's position and extends in the direction of the point's normal.
   *
   * @param scannable_points The scannable points on the fruit surface.
   * @return A VtkLineSegmentsVisualization object that can be used to visualize the fruit points.
   */
VtkLineSegmentsVisualization createFruitLinesVisualization(const ScannablePoints& scannable_points)
{
    VtkLineSegmentsVisualization fruit_points_visualization(1, 1, 1);

    std::vector<std::pair<math::Vec3d, math::Vec3d>> fruit_lines;
    fruit_lines.reserve(scannable_points.surface_points.size());
    for (const auto& [position, normal] : scannable_points.surface_points)
    {
        fruit_lines.emplace_back(position, position + normal * 0.01);
    }
    fruit_points_visualization.updateLine(fruit_lines);

    return fruit_points_visualization;
}

/**
 * @brief Generates colors for the visualization based on visibility of points.
 *
 * This function generates a vector of colors (represented as math::Vec3d objects)
 * for the visualization. Each color corresponds to a point. If the point has been seen,
 * the color is green (0.0, 1.0, 0.0). If the point has not been seen, the color is red (1.0, 0.0, 0.0).
 *
 * @param ever_seen The visibility status of each point.
 * @return A vector of colors for the visualization.
 */
std::vector<math::Vec3d> generateVisualizationColors(const SeenPoints& ever_seen)
{
    std::vector<math::Vec3d> vis_colors;
    for (const auto& v : ever_seen.ever_seen)
    {
        if (v)
        {
            vis_colors.emplace_back(0.0, 1.0, 0.0);
        }
        else
        {
            vis_colors.emplace_back(1.0, 0.0, 0.0);
        }
    }
    return vis_colors;
}

std::vector<JsonMeta<ParametricPath>> getOrbits(const mgodpl::math::Vec3d& fruit_center, double EYE_ORBIT_RADIUS)
{
    const auto orbit_types = mgodpl::gen_orbits(fruit_center, EYE_ORBIT_RADIUS);

    std::cout << "Choose orbit type: " << std::endl;
    for (int i = 0; i < orbit_types.size(); i++)
    {
        std::cout << i << ": " << orbit_types[i].first << std::endl;
    }

    std::cout << (orbit_types.size()) << ": All" << std::endl;

    int orbitType;
    std::cin >> orbitType;

    if (orbitType < 0 || orbitType > orbit_types.size())
    {
        std::cout << "Invalid orbit type. Exiting..." << std::endl;
        return {};
    }

    std::vector<JsonMeta<ParametricPath>> orbits;

    if (orbitType == orbit_types.size())
    {
        std::cout << "All orbits" << std::endl;
        // All of them.
        for (const auto& orbit_type : orbit_types)
        {
            auto orbit = orbit_type.second;
            orbits.insert(orbits.end(), orbit.begin(), orbit.end());
        }
    }
    else
    {
        std::cout << "Orbit type: " << orbit_types[orbitType].first << std::endl;
        orbits = orbit_types[orbitType].second;
    }

    return orbits;
}

REGISTER_VISUALIZATION(visualize_several_orbits_simultaneously)
{
    // Load the tree meshes
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Grab the fruit mesh
    shape_msgs::msg::Mesh fruit_mesh = tree_model.fruit_meshes[0];

    // Calculate the center of the fruit mesh
    math::Vec3d fruit_center = mesh_aabb(fruit_mesh).center();

    // Initialize a random number generator
    random_numbers::RandomNumberGenerator rng;

    // Define constants for the scannable points
    const size_t NUM_POINTS = 200;
    const double MAX_DISTANCE = INFINITY;
    const double MIN_DISTANCE = 0;
    const double MAX_ANGLE = M_PI / 3.0;

    // Create the scannable points
    ScannablePoints scannable_points = createScannablePoints(rng, fruit_mesh, NUM_POINTS, MAX_DISTANCE, MIN_DISTANCE,
                                                             MAX_ANGLE);

    // Add the fruit mesh to the viewer
    viewer.addMesh(tree_model.fruit_meshes[0], {1.0, 0.0, 0.0}, 1.0);

    // Define the initial orbit radius
    const double EYE_ORBIT_RADIUS = 0.1;

    // Create several orbit functions
    std::vector<ParametricPath> orbits;
    orbits.reserve(20);
    for (int i = 0; i < 20; ++i)
    {
        // Create an orbit function for each radius and add it to the orbits vector
        orbits.push_back(fixed_radius_equatorial_orbit(fruit_center, EYE_ORBIT_RADIUS + i * 0.1));
    }

    // Declare a vector to store the eye positions for each orbit
    std::vector<std::vector<mgodpl::math::Vec3d>> eye_positions(orbits.size());

    // Create several instances of VtkPolyLineVisualization
    std::vector<VtkPolyLineVisualization> eye_positions_visualizations;
    for (int i = 0; i < orbits.size(); ++i)
    {
        // Create a VtkPolyLineVisualization for each orbit and add it to the viewer
        eye_positions_visualizations.emplace_back(1, 0, 0); // Red color
        viewer.addActor(eye_positions_visualizations.back().getActor());
    }

    // Add a timer callback to the viewer
    viewer.addTimerCallback([&]()
    {
        static double t = 0.0;
        t += 0.01;

        for (int i = 0; i < orbits.size(); ++i)
        {
            // Use the orbit function to set the eye_position
            math::Vec3d eye_position = orbits[i](t);

            // Update the vector with the new eye position
            eye_positions[i].push_back(eye_position);

            // Update the polyline with the new set of eye positions
            eye_positions_visualizations[i].updateLine(eye_positions[i]);
        }

        // Stop the viewer when the timer exceeds 1.0
        if (t > 1.0)
        {
            viewer.stop();
        }
    });

    // Set the camera transform for the viewer
    viewer.setCameraTransform(fruit_center + math::Vec3d{1.5, 0.0, 1.0}, fruit_center);

    // Start the viewer
    viewer.start();
}

REGISTER_VISUALIZATION(scan_progressive_orbit)
{
    // Load the tree meshes
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Grab the fruit mesh
    shape_msgs::msg::Mesh fruit_mesh = tree_model.fruit_meshes[0];

    // Calculate the center of the fruit mesh
    math::Vec3d fruit_center = mesh_aabb(fruit_mesh).center();

    // Initialize a random number generator
    random_numbers::RandomNumberGenerator rng;

    // Create the scannable points
    ScannablePoints scannable_points = createScannablePoints(rng, fruit_mesh, 1000, INFINITY, 0, M_PI / 3.0);

    // Initialize all points as unseen
    SeenPoints ever_seen = SeenPoints::create_all_unseen(scannable_points);

    // Create the fruit points visualization
    VtkLineSegmentsVisualization fruit_points_visualization = createFruitLinesVisualization(scannable_points);

    // Add the fruit points visualization to the viewer
    viewer.addActor(fruit_points_visualization.getActor());

    // Add the fruit mesh to the viewer
    viewer.addMesh(tree_model.fruit_meshes[0], {0.8,0.8,0.8}, 1.0);

    // Define the initial orbit radius
    double EYE_ORBIT_RADIUS = 0.1;

    // Declare a vector to store the eye positions
    std::vector<mgodpl::math::Vec3d> eye_positions;

    // Create an instance of VtkPolyLineVisualization
    VtkPolyLineVisualization eye_positions_visualization(1, 0, 0); // Red color

    // Add the polyline visualization to the viewer
    viewer.addActor(eye_positions_visualization.getActor());

    std::vector<JsonMeta<ParametricPath>> orbits = getOrbits(fruit_center, EYE_ORBIT_RADIUS);

    const bool LOOP_ANIMATION = false;

    // Add a timer callback to the viewer
    viewer.addTimerCallback([&]()
    {
        // Define a static variable for the timer
        static double t = 0.0;
        // Define a static variable for the current orbit index
        static size_t current_orbit_index = 0;

        t += 0.01;

        // If the timer exceeds 1.0, reset it and move to the next orbit
        if (t > 1.0)
        {
            t = 0.0;
            // Increment the current orbit index, modulo the number of orbits
            current_orbit_index = (current_orbit_index + 1) % orbits.size();

            if (current_orbit_index == 0 && !LOOP_ANIMATION)
            {
                viewer.stop();
            }

            // Clear the eye positions vector
            eye_positions.clear();

            // Reset the scanned points
            ever_seen = SeenPoints::create_all_unseen(scannable_points);
        }

        // Use the orbit function to set the eye_position
        math::Vec3d eye_position = orbits[current_orbit_index].data(t);

        // Update the vector with the new eye position
        eye_positions.push_back(eye_position);

        // Update the polyline with the new set of eye positions
        eye_positions_visualization.updateLine(eye_positions);

        // Update the visibility of the scannable points
        update_visibility(scannable_points, eye_position, ever_seen);

        // Set the colors of the fruit points visualization
        fruit_points_visualization.setColors(generateVisualizationColors(ever_seen));
    });

    // Set the camera transform for the viewer
    viewer.setCameraTransform(fruit_center + math::Vec3d{2.0, 1.0, 1.0}, fruit_center);

    // Start the viewer
    viewer.start();
}
