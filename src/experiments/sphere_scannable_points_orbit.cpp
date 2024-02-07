#include "../experiment_utils/TreeMeshes.h"
#include "../math/Triangle.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/mesh_utils.h"

// Namespace for the project
using namespace mgodpl;

// Type alias for a function that takes a double and returns a Vec3d.
// The interpretation is that of a parametric path in 3D space,
// where the input parameter is abstract time (between 0 and 1).
using ParametricPath = std::function<math::Vec3d(double)>;

/**
 * @brief Creates a parametric path representing a fixed-radius equatorial orbit around a given center point.
 *
 * This function generates a fixed-radius orbit around a point in the form of a parametric path.
 * The path is defined in the xy plane (z = 0) and is parameterized by a time parameter `t` between 0 and 1.
 * The position on the orbit at time `t` is calculated using the cosine and sine of `t * 2.0 * M_PI` to generate the x and y coordinates.
 *
 * @param center The center point of the orbit.
 * @param radius The radius of the orbit.
 * @return A function that takes a time parameter `t` between 0 and 1 and returns a `math::Vec3d` representing the position on the orbit at that time.
 */
ParametricPath fixed_radius_equatorial_orbit(const math::Vec3d& center, double radius) {
    return [center, radius](double t) {
        const double angle = t * 2.0 * M_PI;
        return center + math::Vec3d{std::cos(angle), std::sin(angle), 0.0} * radius;
    };
}

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

    // Variable to keep track of the total distance traversed
    double total_distance = 0.0;
    // Store the previous eye position
    math::Vec3d previous_eye_position = orbit(0.0);

    // Number of segments in the orbit
    const int NUM_SEGMENTS = 100;

    // Loop over each segment
    for (int i = 0; i <= NUM_SEGMENTS; ++i) {
        // Calculate t for the current segment
        double t = static_cast<double>(i) / NUM_SEGMENTS;
        // Calculate the new eye position using the orbit function
        math::Vec3d eye_position = orbit(t);

        // Calculate the distance traversed
        total_distance += (eye_position - previous_eye_position).norm();
        // Update the previous eye position
        previous_eye_position = eye_position;

        // Update the visibility of the points
        update_visibility(scannable_points, eye_position, ever_seen);

        // Print some stats:
        const size_t num_seen = ever_seen.count_seen();
        const double percent = round(100.0 * static_cast<double>(num_seen) / static_cast<double>(ever_seen.ever_seen.size()));
        std::cout << "Seen: " << num_seen << " / " << ever_seen.ever_seen.size() << " (" << percent << "%)" << std::endl;
    }

    // Print the total distance traversed
    std::cout << "Total distance traversed: " << total_distance << std::endl;

    return 0;
}