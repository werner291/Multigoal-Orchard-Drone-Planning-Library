// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "../experiment_utils/TreeMeshes.h"
#include "../math/Triangle.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/mesh_utils.h"

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

    const double EYE_ORBIT_RADIUS = 0.5;

    math::Vec3d eye_position = fruit_center + math::Vec3d{1.0, 0.0, 0.0} * EYE_ORBIT_RADIUS;

    SeenPoints ever_seen = SeenPoints::create_all_unseen(scannable_points);

    static double t = 0.0;
    t += 0.02;
    eye_position = fruit_center + math::Vec3d{std::cos(t), std::sin(t), 0.0} * EYE_ORBIT_RADIUS;

    update_visibility(scannable_points, eye_position, ever_seen);

    // Print some stats:
    const size_t num_seen = ever_seen.count_seen();
    const double percent = round(100.0 * static_cast<double>(num_seen) / static_cast<double>(ever_seen.ever_seen.size()));
    std::cout << "Seen: " << num_seen << " / " << ever_seen.ever_seen.size() << " (" << percent << "%)" << std::endl;

    return 0;
}