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

using namespace mgodpl;

/**
 * @brief Checks if a point is visible from a given position.
 *
 * This function takes a SurfacePoint object and a Vec3d object representing the eye position.
 * It computes the visibility of the point from the eye position based on the distance and angle.
 * A point is considered visible if it is within the maximum distance and the angle between the point's normal
 * and the vector from the point to the eye is less than the maximum angle.
 *
 * Note: This function does not account for occlusions. It only checks the visibility based on distance and angle.
 *
 * @param point         A SurfacePoint object representing a point in 3D space with a position and a normal.
 * @param eye_position  A Vec3d object representing the position of the eye in 3D space.
 * @param max_distance  The maximum distance from the eye position to consider a point visible.
 * @param max_angle     The maximum angle between the normal of a point and the vector from the point to the eye
 *                      to consider a point visible.
 * @return              A boolean value. If true, the point is visible from the eye position. If false, the point is not visible.
 */
bool is_visible(const SurfacePoint& point, const math::Vec3d& eye_position, const double max_distance,
                const double max_angle)
{
    // Calculate the vector from the point to the eye position
    auto delta = eye_position - point.position;

    // Calculate the distance from the point to the eye position
    double distance = delta.norm();

    // If the distance is greater than the maximum distance, the point is not visible
    if (distance > max_distance)
    {
        return false;
    }

    // Calculate the angle between the point's normal and the vector from the point to the eye
    double angle = std::acos(point.normal.dot(delta) / distance);

    // If the angle is greater than the maximum angle, the point is not visible
    if (angle > max_angle)
    {
        return false;
    }

    // If the point passed both the distance and angle checks, it is visible
    return true;
}

/**
 * @brief Updates the visibility status of a set of points from a given eye position.
 *
 * This function takes a vector of SurfacePoint objects, a Vec3d object representing the eye position,
 * and a vector of booleans representing whether each point has ever been seen.
 * It updates the visibility status of each point in the vector. A point is considered visible
 * if the dot product of the point's normal and the vector from the point to the eye is negative.
 *
 * @param fruit_points 	A vector of SurfacePoint objects. Each SurfacePoint object represents a point in 3D space
 * 						and has a position and a normal.
 * @param max_distance 	The maximum distance from the eye position to consider a point visible.
 * @param max_angle     The maximum angle between the normal of a point and the vector from the point to the eye
 * @param eye_position 	A Vec3d object representing the position of the eye in 3D space.
 * @param ever_seen     A vector of boolean values. Each value corresponds to a point in the input vector. If the value is true,
 * 						the point has ever been seen from the eye position. If the value is false, the point has never been seen.
 */
void update_visibility(const std::vector<SurfacePoint>& fruit_points,
                       const double max_distance,
                       const double max_angle,
                       const math::Vec3d& eye_position,
                       std::vector<bool>& ever_seen)
{
    for (size_t i = 0; i < fruit_points.size(); ++i)
    {
        if (is_visible(fruit_points[i], eye_position, max_distance, max_angle))
        {
            ever_seen[i] = true;
        }
    }
}

int main(int argc, char** argv)
{
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Grab the fruit mesh
    shape_msgs::msg::Mesh fruit_mesh = tree_model.fruit_meshes[0];

    math::Vec3d fruit_center = mesh_aabb(fruit_mesh).center();

    random_numbers::RandomNumberGenerator rng;

    std::vector<SurfacePoint> fruit_points = sample_points_on_mesh(rng, fruit_mesh, 200);

    VtkLineSegmentsVisualization fruit_points_visualization(1, 1, 1);

    std::vector<std::pair<math::Vec3d, math::Vec3d>> fruit_lines;

    fruit_lines.reserve(fruit_points.size());
    for (const auto& fruit_point : fruit_points)
    {
        fruit_lines.emplace_back(fruit_point.position, fruit_point.position + fruit_point.normal * 0.05);
    }
    fruit_points_visualization.updateLine(fruit_lines);

    SimpleVtkViewer viewer;

    viewer.addMesh(tree_model.fruit_meshes[0], {1.0, 0.0, 0.0}, 1.0);

    const double EYE_ORBIT_RADIUS = 0.5;

    math::Vec3d eye_position = fruit_center + math::Vec3d{1.0, 0.0, 0.0} * EYE_ORBIT_RADIUS;

    auto eye_sphere = viewer.addSphere(0.02, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 1.0);

    viewer.addActor(fruit_points_visualization.getActor());

    std::vector<bool> ever_seen(fruit_points.size(), false);

    viewer.addTimerCallback([&]()
    {
        static double t = 0.0;
        t += 0.02;
        eye_position = fruit_center + math::Vec3d{std::cos(t), std::sin(t), 0.0} * EYE_ORBIT_RADIUS;

        eye_sphere->SetPosition(eye_position.x(), eye_position.y(), eye_position.z());

        const double MAX_DISTANCE = INFINITY;
        const double MAX_ANGLE = M_PI / 3.0;

        update_visibility(fruit_points, MAX_DISTANCE, MAX_ANGLE, eye_position, ever_seen);

        // Print some stats:
        size_t num_visible = std::count(ever_seen.begin(), ever_seen.end(), true);
        double percent = round(100.0 * num_visible / ever_seen.size());
        std::cout << "Seen: " << num_visible << " / " << ever_seen.size() << " (" << percent << "%)" << std::endl;

        std::vector<math::Vec3d> vis_colors;
        for (const auto& v : ever_seen)
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

        if (t > 2.0 * M_PI)
        {
            viewer.stop();
        }
    });

    viewer.startRecording("fruit_scan_points.ogv");

    viewer.setCameraTransform(fruit_center + math::Vec3d{1.5, 0.0, 1.0}, fruit_center);

    viewer.start();
}
