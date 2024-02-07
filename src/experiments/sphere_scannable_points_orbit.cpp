#include "../experiment_utils/TreeMeshes.h"
#include "../math/Triangle.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/mesh_utils.h"
#include "../experiment_utils/scan_paths.h"
#include <json/json.h>
#include <fstream>
#include <filesystem>

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
    const size_t NUM_POINTS = 1000;
    const double MAX_DISTANCE = INFINITY;
    const double MIN_DISTANCE = 0;
    const double MAX_ANGLE = M_PI / 3.0;

    // Create the scannable points
    ScannablePoints scannable_points = createScannablePoints(rng, fruit_mesh, NUM_POINTS, MAX_DISTANCE, MIN_DISTANCE, MAX_ANGLE);

    // Number of segments in the orbit
    const int NUM_SEGMENTS = 100;

    // Create a Json::Value to store the results
    Json::Value results;

    // Iterate over the orbit radii from 0.1 to 2.0 in steps of 0.1
    for (int i = 0; i <= 20; ++i) {
        double radius = i * 0.1;

        // Create the orbit function for the current radius
        ParametricPath orbit = fixed_radius_equatorial_orbit(fruit_center, radius);

        // Initialize all points as unseen
        SeenPoints ever_seen = SeenPoints::create_all_unseen(scannable_points);

        // Evaluate the path
        PathEvaluationResult result = evaluatePath(orbit, scannable_points, ever_seen, NUM_SEGMENTS);

        // Add the results to the Json::Value
        Json::Value resultJson;
        resultJson["radius"] = radius;
        resultJson["num_points_seen"] = result.num_points_seen;
        resultJson["total_distance_traveled"] = result.total_distance_traveled;
        results.append(resultJson);
    }

    // Filename:
    const std::string filename = "analysis/data/orbits.json";

    // Open a file output stream
    std::ofstream file(filename);

    // Create a new Json::Value object
    Json::Value output;

    // Add the results to the new object
    output["results"] = results;

    // Add the metadata
    output["metadata"]["num_points"] = NUM_POINTS;
    output["metadata"]["max_distance"] = MAX_DISTANCE;
    output["metadata"]["min_distance"] = MIN_DISTANCE;
    output["metadata"]["max_angle"] = MAX_ANGLE;

    // Now, instead of writing `results` to the file, write `output`
    file << output;

    // Close the file output stream
    file.close();

    // Print the absolute path of the file
    std::cout << "Results file: " << std::filesystem::absolute(filename) << std::endl;

    return 0;
}