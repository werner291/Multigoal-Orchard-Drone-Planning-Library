#include "../experiment_utils/TreeMeshes.h"
#include "../math/Triangle.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/mesh_utils.h"
#include "../experiment_utils/scan_paths.h"
#include "../experiment_utils/scan_path_generators.h"
#include "../experiment_utils/declarative/SensorModelParameters.h"
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
    Mesh fruit_mesh = tree_model.fruit_meshes[0];

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

    // Get all the orbits
    auto all_orbits = gen_orbits(fruit_center, 1.0); // Assuming 1.0 as target radius

    // Iterate over each orbit
    for (const auto& orbit_pair : all_orbits) {
        for (const auto& orbit : orbit_pair.second) {
            // Initialize all points as unseen
            SeenPoints ever_seen = SeenPoints::create_all_unseen(scannable_points);

            // Evaluate the path
            PathEvaluationResult result = evaluatePath(orbit.data, scannable_points, ever_seen, NUM_SEGMENTS);

            // Add the results to the Json::Value
            Json::Value resultJson;
            resultJson["path_meta"] = orbit.meta;
            resultJson["num_points_seen"] = result.num_points_seen;
            resultJson["total_distance_traveled"] = result.total_distance_traveled;
            results[orbit_pair.first].append(resultJson);
        }
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
