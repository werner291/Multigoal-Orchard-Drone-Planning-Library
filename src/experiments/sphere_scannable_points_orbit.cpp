#include "../experiment_utils/TreeMeshes.h"
#include "../math/Triangle.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/mesh_utils.h"
#include "../experiment_utils/scan_paths.h"

// Namespace for the project
using namespace mgodpl;

int main(int argc, char** argv)
{
    // Load the tree meshes
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Grab the fruit mesh from the tree model
    shape_msgs::msg::Mesh fruit_mesh = tree_model.fruit_meshes[0];

    // Calculate the center of the fruit mesh
    math::Vec3d fruit_center = mesh_aabb(fruit_mesh).center();

    // Random number generator
    random_numbers::RandomNumberGenerator rng;

    // Constants for the scannable points
    const size_t NUM_POINTS = 200;
    const double MAX_DISTANCE = INFINITY;
    const double MIN_DISTANCE = 0;
    const double MAX_ANGLE = M_PI / 3.0;

    // Create the scannable points
    ScannablePoints scannable_points = createScannablePoints(rng, fruit_mesh, NUM_POINTS, MAX_DISTANCE, MIN_DISTANCE, MAX_ANGLE);

    // Radius of the eye orbit
    const double EYE_ORBIT_RADIUS = 0.5;

    // Create the orbit function
    ParametricPath orbit = fixed_radius_equatorial_orbit(fruit_center, EYE_ORBIT_RADIUS);

    // Initialize all points as unseen
    SeenPoints ever_seen = SeenPoints::create_all_unseen(scannable_points);

    // Number of segments in the orbit
    const int NUM_SEGMENTS = 100;

    // Evaluate the path
    PathEvaluationResult result = evaluatePath(orbit, scannable_points, ever_seen, NUM_SEGMENTS);

    // Print the results
    std::cout << "Number of points seen: " << result.num_points_seen << std::endl;
    std::cout << "Total distance traveled: " << result.total_distance_traveled << std::endl;

    return 0;
}