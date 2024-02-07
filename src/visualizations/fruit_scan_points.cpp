// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <random_numbers/random_numbers.h>
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../math/Triangle.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/mesh_utils.h"
#include "../experiment_utils/scan_paths.h"
#include "../visualization/VtkPolyLineVisualization.h"

using namespace mgodpl;

int main(int argc, char** argv)
{
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Grab the fruit mesh
    shape_msgs::msg::Mesh fruit_mesh = tree_model.fruit_meshes[0];

    math::Vec3d fruit_center = mesh_aabb(fruit_mesh).center();

    random_numbers::RandomNumberGenerator rng;

    const size_t NUM_POINTS = 200;
    const double MAX_DISTANCE = INFINITY;
    const double MIN_DISTANCE = 0;
    const double MAX_ANGLE = M_PI / 3.0;

    ScannablePoints scannable_points = createScannablePoints(rng, fruit_mesh, NUM_POINTS, MAX_DISTANCE, MIN_DISTANCE, MAX_ANGLE);
    VtkLineSegmentsVisualization fruit_points_visualization(1, 1, 1);

    std::vector<std::pair<math::Vec3d, math::Vec3d>> fruit_lines;

    fruit_lines.reserve(scannable_points.surface_points.size());
    for (const auto& [position, normal] : scannable_points.surface_points)
    {
        fruit_lines.emplace_back(position, position + normal * 0.05);
    }
    fruit_points_visualization.updateLine(fruit_lines);

    SimpleVtkViewer viewer;

    viewer.addMesh(tree_model.fruit_meshes[0], {1.0, 0.0, 0.0}, 1.0);

    const double EYE_ORBIT_RADIUS = 0.5;

    math::Vec3d eye_position = fruit_center + math::Vec3d{1.0, 0.0, 0.0} * EYE_ORBIT_RADIUS;

    auto eye_sphere = viewer.addSphere(0.02, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 1.0);

    viewer.addActor(fruit_points_visualization.getActor());

    // Initialize all points as unseen
    SeenPoints ever_seen = SeenPoints::create_all_unseen(scannable_points);

    // Create the orbit function
    ParametricPath orbit = fixed_radius_equatorial_orbit(fruit_center, EYE_ORBIT_RADIUS);

    // Declare a vector to store the eye positions
    std::vector<mgodpl::math::Vec3d> eye_positions;

    // Create an instance of VtkPolyLineVisualization
    VtkPolyLineVisualization eye_positions_visualization(1, 0, 0); // Red color

    viewer.addTimerCallback([&](){
        static double t = 0.0;
        t += 0.01;
        eye_position = orbit(t); // Use the orbit function to set the eye_position

        // Update the vector with the new eye position
        eye_positions.push_back(eye_position);

        // Update the polyline with the new set of eye positions
        eye_positions_visualization.updateLine(eye_positions);

        eye_sphere->SetPosition(eye_position.x(), eye_position.y(), eye_position.z());
        update_visibility(scannable_points, eye_position, ever_seen);

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

        fruit_points_visualization.setColors(vis_colors);

        if (t > 1.0)
        {
            viewer.stop();
        }
    });

    // Add the polyline visualization to the viewer
    viewer.addActor(eye_positions_visualization.getActor());

    // viewer.startRecording("fruit_scan_points.ogv");

    viewer.setCameraTransform(fruit_center + math::Vec3d{1.5, 0.0, 1.0}, fruit_center);

    viewer.start();
}