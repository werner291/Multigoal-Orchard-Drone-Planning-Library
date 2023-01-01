#include <random>
#include <Eigen/Dense>
#include <vector>
#include <range/v3/all.hpp>
#include <iomanip>
#include <json/json.h>
#include <fstream>
#include <iostream>

#include "../utilities/math_utils.h"
#include "../occupancy_mapping/OccupancyMap.h"
#include "../occupancy_mapping/HierarchicalCategoricalOccupancyOctree.h"
#include "../occupancy_mapping/HierarchicalBoundaryCellAnnotatedRegionOctree.h"

static const double TEST_POINT_DISTANCE_FROM_BOUNDARY = 0.5;
// Struct to represent a sphere
struct Sphere {
	Eigen::Vector3d center;
	double radius;
};

/**
 * Convert a Sphere struct to a JSON object
 *
 * @param sphere  Sphere struct
 * @return JSON object
 */
Json::Value toJSON(const Sphere &sphere) {
	Json::Value json;
	json["center"] = toJSON(sphere.center);
	json["radius"] = sphere.radius;
	return json;
}

/**
 * Convert a JSON object to a Sphere struct
 *
 * @param json  JSON object
 * @return Sphere struct
 */
Sphere sphereFromJSON(const Json::Value &json) {
	return Sphere{fromJsonVector3d(json["center"]), json["radius"].asDouble()};
}

/**
 * Generates a vector of randomly generated spheres
 *
 * @param num_spheres  Number of spheres to generate
 * @return Vector of Sphere objects
 */
std::vector<Sphere> generateRandomSpheres(int num_spheres) {
	// Set the random seed
	std::random_device rd;
	std::mt19937 gen(rd());

	// Create a uniform distribution for generating random coordinates
	std::uniform_real_distribution<> coord_dist(-10.0, 10.0);

	// Create a uniform distribution for generating random radii
	std::uniform_real_distribution<> radius_dist(0.1, 1.0);

	// Vector to store the generated spheres
	std::vector<Sphere> spheres;

	// Generate the requested number of random spheres
	for (int i = 0; i < num_spheres; i++) {
		// Generate a random center point for the sphere
		Eigen::Vector3d center(coord_dist(gen), coord_dist(gen), coord_dist(gen));

		// Generate a random radius for the sphere
		double radius = radius_dist(gen);

		// Create a sphere object and add it to the vector
		spheres.push_back({center, radius});
	}

	return spheres;
}

/**
 * Check if a point lies inside a sphere
 *
 * @param point  Point to check
 * @param sphere  Sphere to check
 * @return True if the point lies inside the sphere, false otherwise
 */
bool pointInSphere(const Eigen::Vector3d &point, const Sphere &sphere) {
	return (point - sphere.center).norm() < sphere.radius;
}

/**
 * Check if a point lies inside the union of a set of spheres
 *
 * @param point  Point to check
 * @param spheres  Vector of spheres to check
 * @return True if the point lies inside the union of the spheres, false otherwise
 */
bool pointInSpheres(const Eigen::Vector3d &point, const std::vector<Sphere> &spheres) {
	// Check if any of the spheres contain the point
	return std::any_of(spheres.begin(), spheres.end(), [&](const Sphere &sphere) {
		return pointInSphere(point, sphere);
	});
}

/**
 * Generates a set of random points within a margin around the boundary of the given sphere.
 *
 * @param sphere       The sphere for which the margin should be calculated.
 * @param n            The number of points to generate.
 * @param margin       The margin around the boundary of the sphere within which points should be generated.
 * @return             A vector of 3D points.
 */
std::vector<Eigen::Vector3d> generateTestPoints(const Sphere &sphere, int n, double margin) {
	// Vector to store the generated points
	std::vector<Eigen::Vector3d> points;
	points.reserve(n);

	// Setup random number generation
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution dis;
	std::uniform_real_distribution margin_dis(-margin, margin);

	// Generate random points until we have the desired number of points
	while (points.size() < n) {
		// Generate a random point and normalize it
		Eigen::Vector3d point(dis(gen), dis(gen), dis(gen));
		point.normalize();

		// Scale the point to be within the margin around the sphere
		point *= sphere.radius + margin_dis(gen);

		// Offset the point by the center of the sphere
		point += sphere.center;

		// Add the point to the vector
		points.push_back(point);
	}

	return points;
}


/**
 * Generate a vector of point/boolean pairs indicating whether each point lies within the union of all the spheres in the given vector
 *
 * @param spheres  Vector of spheres
 * @param num_test_points  Number of test points to generate for each sphere
 * @param max_distance  Maximum distance from the boundary surface of each sphere to generate test points
 *
 * @return Vector of point/boolean pairs indicating whether each point lies within the union of all the spheres in the given vector
 */
std::vector<std::pair<Eigen::Vector3d, bool>>
generateTestPointsOnSpheres(const std::vector<Sphere> &spheres, int num_test_points, double max_distance) {

	// Generate test points near the boundary surface of each sphere
	return
		// Generate test points for each sphere
			spheres | ranges::views::transform([&](const Sphere &sphere) {
				return generateTestPoints(sphere, num_test_points, max_distance);
			})
			// Flatten the vector of vectors into a single vector
			| ranges::views::join
			// Pair each up with a boolean
			| ranges::views::transform([&](const Eigen::Vector3d &point) -> std::pair<Eigen::Vector3d, bool> {
				return {point, pointInSpheres(point, spheres)};
			})
			// Convert the view to a std::vector
			| ranges::to_vector;
}


void writeResults(std::vector<Sphere> &spheres,
				  std::vector<std::pair<Eigen::Vector3d, bool>> &test_points,
				  std::vector<std::pair<std::string, std::vector<bool>>> &results) {// Generate a timestamp for the current time
	auto now = time(nullptr);
	tm tm = *localtime(&now);
	std::stringstream timestamp;
	timestamp << std::put_time(&tm, "%Y%m%d-%H%M%S");

	// Create a JSON object to store the data
	Json::Value root;

	// Add the spheres to the JSON object
	Json::Value spheres_json(Json::arrayValue);
	for (const auto &sphere: spheres) {
		spheres_json.append(toJSON(sphere));
	}
	root["spheres"] = spheres_json;

	// Add the points to the JSON object
	Json::Value points_json(Json::arrayValue);
	for (const auto &point: test_points) {
		Json::Value point_json;
		point_json["point"] = toJSON(point.first);
		point_json["expected_result"] = point.second;
		points_json.append(point_json);
	}
	root["points"] = points_json;

	// Add the test results to the JSON object
	Json::Value results_json(Json::objectValue);
	for (const auto &[name, resultss]: results) {
		Json::Value results_array(Json::arrayValue);
		for (bool result: resultss) {
			results_array.append(result);
		}
		results_json[name] = results_array;
	}
	root["results"] = results_json;

	// Write the JSON object to a file
	std::ofstream out_file("analysis/data/test_results_" + timestamp.str() + ".json");
	out_file << root.toStyledString();
	out_file.close();
}

/**
 * Compute a bounding box that encloses a vector of spheres.
 *
 * @param spheres  Vector of spheres
 * @return Bounding box that encloses the given spheres
 */
Eigen::AlignedBox3d computeBoundingBox(const std::vector<Sphere> &spheres) {
	Eigen::AlignedBox3d bounding_box;
	for (const auto &sphere: spheres) {
		bounding_box.extend(sphere.center - Eigen::Vector3d(sphere.radius, sphere.radius, sphere.radius));
		bounding_box.extend(sphere.center + Eigen::Vector3d(sphere.radius, sphere.radius, sphere.radius));
	}
	return bounding_box;
}

NodeCount countNodes(const OccupancyMap* map) {

	// Try downcasting it to the various types of OccupancyMap and count the nodes.

	if (auto categorical = dynamic_cast<const HierarchicalCategoricalOccupancyOctree*>(map)) {
		return categorical->getOctree().count_nodes();
	}

	if (auto sample_annotated = dynamic_cast<const HierarchicalBoundaryCellAnnotatedRegionOctree*>(map)) {
		return sample_annotated->getTree().count_nodes();
	}

	throw std::runtime_error("Unknown occupancy map type");

}

// Enumeration to keep track of different error types
enum ErrorType {
	EXPECTED_FREE_BUT_OCCUPIED,
	EXPECTED_OCCUPIED_BUT_FREE,
	CORRECT_FREE,
	CORRECT_OCCUPIED
};

struct ConfusionMatrix {

	size_t expected_free_but_occupied = 0;
	size_t expected_occupied_but_free = 0;
	size_t correct_free = 0;
	size_t correct_occupied = 0;

};

Json::Value toJSON(const ConfusionMatrix &confusion_matrix) {
	Json::Value json;
	json["expected_free_but_occupied"] = (int) confusion_matrix.expected_free_but_occupied;
	json["expected_occupied_but_free"] = (int) confusion_matrix.expected_occupied_but_free;
	json["correct_free"] = (int) confusion_matrix.correct_free;
	json["correct_occupied"] = (int) confusion_matrix.correct_occupied;
	return json;
}

/**
* @brief Determines the type of error based on the expected and actual results.
*
* @param expected_result The expected result of the function.
* @param expected The expected output of the function.
* @param actual The actual output of the function.
* @return The type of error that occurred.
*/
ErrorType getErrorType(const bool expected_result,
					   const OccupancyMap::RegionType &expected,
					   const OccupancyMap::RegionType &actual) {
	ErrorType error_type;
	if (expected_result) {
		if (actual == expected) {
			error_type = CORRECT_FREE;
		} else {
			error_type = EXPECTED_FREE_BUT_OCCUPIED;
		}
	} else {
		if (actual == expected) {
			error_type = CORRECT_OCCUPIED;
		} else {
			error_type = EXPECTED_OCCUPIED_BUT_FREE;
		}
	}
	return error_type;
}

// Define a struct to store map names, pointers to OccupancyMap objects, and the maximum depth of each map
struct MapInfo {
	std::string name;
	std::unique_ptr<OccupancyMap> map;
	size_t max_depth;
};

/**
 * @brief Tests the given occupancy map against the given test points, returning a confusion matrix.
 *
 * @param occupancyMap The occupancy map to test.
 * @param test_points The test points to test the map against (each point is paired with a boolean indicating whether the point is expected to be free (true) or unseen (false)).
 *
 * @return A confusion matrix
 */
ConfusionMatrix testMap(const OccupancyMap& occupancyMap,
						const std::vector<std::pair<Eigen::Vector3d, bool>> &test_points) {
	ConfusionMatrix confusion_matrix;

	size_t processed = 0;

	// Iterate over the test points
	for (const auto&[point, expected_result]: test_points) {
		// Get the expected region type
		OccupancyMap::RegionType expected_region_type;
		if (expected_result) {
			expected_region_type = OccupancyMap::RegionType::FREE;
		} else {
			expected_region_type = OccupancyMap::RegionType::UNSEEN;
		}

		// Get the actual region type
		OccupancyMap::RegionType actual_region_type = occupancyMap.query_at(point);

		// Determine the type of error that occurred
		ErrorType error_type = getErrorType(expected_result, expected_region_type, actual_region_type);

		// Update the confusion matrix
		switch (error_type) {
			case EXPECTED_FREE_BUT_OCCUPIED:
				confusion_matrix.expected_free_but_occupied++;
				break;
			case EXPECTED_OCCUPIED_BUT_FREE:
				confusion_matrix.expected_occupied_but_free++;
				break;
			case CORRECT_FREE:
				confusion_matrix.correct_free++;
				break;
			case CORRECT_OCCUPIED:
				confusion_matrix.correct_occupied++;
				break;
		}

		if (processed ++ % 10000 == 0) {
			std::cout << "Processed " << processed << "/" << test_points.size() << " test points" << std::endl;
		}
	}

	return confusion_matrix;

}


/**
 * @brief Main function that generates random spheres, constructs occupancy maps based on those spheres, and tests the accuracy of the maps.
 * @return The exit code of the program.
 */
int main() {

	// Generate 100 random spheres
	std::vector<Sphere> spheres = generateRandomSpheres(10);

	// Compute the bounding box that contains all of the spheres
	Eigen::AlignedBox3d bounding_box = computeBoundingBox(spheres);

	// Set the center of the bounding box as the center of the occupancy maps
	Eigen::Vector3d center = bounding_box.center();

	// Set the size of the occupancy maps as the maximum dimension of the bounding box, with a small buffer added
	double cube_size = bounding_box.sizes().maxCoeff() + TEST_POINT_DISTANCE_FROM_BOUNDARY * 2.0;

	// Do something with the spheres (e.g., add them to a list or scene)
	// Create a vector to store the OccupancyMap objects and their names
	std::vector<MapInfo> maps;

	// Create HierarchicalCategoricalOccupancyOctree and HierarchicalBoundaryCellAnnotatedRegionOctree objects with depths ranging from 2 to 5
	for (size_t i = 2; i <= 10; i++) {
		maps.push_back({
							   "HierarchicalCategoricalOccupancyOctree",
							   std::make_unique<HierarchicalCategoricalOccupancyOctree>(center, cube_size, i),
							   i
					   });
		maps.push_back({
							   "HierarchicalBoundaryCellAnnotatedRegionOctree",
							   std::make_unique<HierarchicalBoundaryCellAnnotatedRegionOctree>(center, cube_size, i),
							   i
					   });
	}

	std::vector<NodeCount> node_counts;

	// Incorporate the spheres into all OccupancyMap objects
	for (auto &mapInfo: maps) {
		OccupancyMap &map = *mapInfo.map;
		for (const Sphere &sphere: spheres) {
			// Incorporate each sphere into the map by adding a boundary sample at the point on the sphere's surface closest to the map's center
			map.incorporate(sphere.center, [&](const Eigen::Vector3d &point) -> BoundarySample {

				// Compute the point on the sphere's surface that is closest to the map's center
				Eigen::Vector3d point_on_sphere = sphere.center + (point - sphere.center).normalized() * sphere.radius;

				return {point_on_sphere, BoundaryType::OCCLUDING};

			});
		}

		// Print the number of nodes in the map
		NodeCount node_count = countNodes(&map);
		std::cout << mapInfo.name << " has " << (node_count.split_count + node_count.leaf_count) << " nodes (" << node_count.split_count << " splits, " << node_count.leaf_count << " leaves)" << std::endl;
		node_counts.push_back(node_count);
	}

	// Generate test points on the surface of the spheres
	std::vector<std::pair<Eigen::Vector3d, bool>> test_points = generateTestPointsOnSpheres(spheres, 10000, TEST_POINT_DISTANCE_FROM_BOUNDARY);
	std::vector<ConfusionMatrix> confusion_matrices = maps | ranges::views::transform([&](const MapInfo &mapInfo) {
		return testMap(*mapInfo.map, test_points);
	}) | ranges::to_vector;

	Json::Value root;

	for (const auto& [map_info, confusion_matrix, node_count]: ranges::views::zip(maps, confusion_matrices, node_counts)) {

		std::cout << map_info.name << " (depth " << map_info.max_depth << ") has " << node_count.split_count << " splits, " << node_count.leaf_count << " leaves, and " << node_count.leaf_count * 8 << " cells" << std::endl;
		std::cout << "Confusion matrix:" << std::endl;
		std::cout << "  Expected free but occupied: " << confusion_matrix.expected_free_but_occupied << std::endl;
		std::cout << "  Expected occupied but free: " << confusion_matrix.expected_occupied_but_free << std::endl;
		std::cout << "  Correct free: " << confusion_matrix.correct_free << std::endl;
		std::cout << "  Correct occupied: " << confusion_matrix.correct_occupied << std::endl;

		Json::Value map_json;
		map_json["name"] = map_info.name;
		map_json["max_depth"] = (int) map_info.max_depth;
		map_json["split_count"] = (int) node_count.split_count;
		map_json["leaf_count"] = (int) node_count.leaf_count;
		map_json["total_count"] = (int) node_count.split_count + (int) node_count.leaf_count;
		map_json["confusion"] = toJSON(confusion_matrix);

		root.append(map_json);
	}

	// Generate a file path for the output file with stringstream; include a timestamp
	std::stringstream ss;
	ss << "analysis/data/occmap_";
	ss << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	ss << ".json";

	// Write the JSON to the output file
	std::ofstream file(ss.str());
	file << root;
	file.close();

	return 0;
}


