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

/**
 * @brief A struct to hold the result of a path evaluation.
 *
 * This struct encapsulates the number of points seen and the total distance traveled
 * during the evaluation of a parametric path.
 */
struct PathEvaluationResult {
    /**
     * @brief The number of points seen during the path evaluation.
     */
    size_t num_points_seen;

    /**
     * @brief The total distance traveled during the path evaluation.
     */
    double total_distance_traveled;
};

/**
 * @brief Evaluates a given path.
 *
 * This function takes a ParametricPath and evaluates it by calculating the number of points seen
 * and the total distance traveled. The function also updates the visibility status of each point
 * in the SeenPoints object.
 *
 * @param path The ParametricPath to evaluate.
 * @param scannable_points The ScannablePoints object containing the points to check for visibility.
 * @param ever_seen The SeenPoints object to update with the visibility status of each point.
 * @param num_segments The number of segments in the path.
 * @return A PathEvaluationResult struct containing the number of points seen and the total distance traveled.
 */
PathEvaluationResult evaluatePath(const ParametricPath& path, const ScannablePoints& scannable_points, SeenPoints& ever_seen, int num_segments) {
    // Variable to keep track of the total distance traversed
    double total_distance = 0.0;
    // Store the previous eye position
    math::Vec3d previous_eye_position = path(0.0);

    // Loop over each segment
    for (int i = 0; i <= num_segments; ++i) {
        // Calculate t for the current segment
        double t = static_cast<double>(i) / num_segments;
        // Calculate the new eye position using the path function
        math::Vec3d eye_position = path(t);

        // Calculate the distance traversed
        total_distance += (eye_position - previous_eye_position).norm();
        // Update the previous eye position
        previous_eye_position = eye_position;

        // Update the visibility of the points
        update_visibility(scannable_points, eye_position, ever_seen);
    }

    // Calculate the number of points seen
    size_t num_seen = ever_seen.count_seen();

    // Return the number of points seen and the total distance traversed
    return {num_seen, total_distance};
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

    // Number of segments in the orbit
    const int NUM_SEGMENTS = 100;

    // Evaluate the path
    PathEvaluationResult result = evaluatePath(orbit, scannable_points, ever_seen, NUM_SEGMENTS);

    // Print the results
    std::cout << "Number of points seen: " << result.num_points_seen << std::endl;
    std::cout << "Total distance traveled: " << result.total_distance_traveled << std::endl;

    return 0;
}