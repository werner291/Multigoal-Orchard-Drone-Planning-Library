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

std::vector<Eigen::Vector3d> generateTestPoints(const Sphere &sphere, int n, double max_distance) {
	std::vector<Eigen::Vector3d> points;
	points.reserve(n);

	// Generate random points within a bounding box that encloses the sphere
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis_x(sphere.center.x() - sphere.radius - max_distance,
										   sphere.center.x() + sphere.radius + max_distance);
	std::uniform_real_distribution<> dis_y(sphere.center.y() - sphere.radius - max_distance,
										   sphere.center.y() + sphere.radius + max_distance);
	std::uniform_real_distribution<> dis_z(sphere.center.z() - sphere.radius - max_distance,
										   sphere.center.z() + sphere.radius + max_distance);

	while (points.size() < n) {
		Eigen::Vector3d point(dis_x(gen), dis_y(gen), dis_z(gen));
		if ((point - sphere.center).norm() < sphere.radius + max_distance) {
			points.push_back(point);
		}
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

int main() {
	// Generate 100 random spheres
	std::vector<Sphere> spheres = generateRandomSpheres(10);

	Eigen::AlignedBox3d bounding_box = computeBoundingBox(spheres);

	Eigen::Vector3d center = bounding_box.center();
	double cube_size = bounding_box.sizes().maxCoeff() + TEST_POINT_DISTANCE_FROM_BOUNDARY * 2.0;

	// Do something with the spheres (e.g., add them to a list or scene)
	// Create a vector to store the OccupancyMap objects and their names
	std::vector<std::pair<std::string, std::unique_ptr<OccupancyMap>>> maps;
	maps.emplace_back("HierarchicalCategoricalOccupancyOctree", std::make_unique<HierarchicalCategoricalOccupancyOctree>(center, cube_size, 5));
//	maps.emplace_back("HierarchicalBoundaryCellAnnotatedRegionOctree", std::make_unique<HierarchicalBoundaryCellAnnotatedRegionOctree>(center, cube_size, 5));

	// Incorporate the spheres into all OccupancyMap objects
	for (auto &map_pair: maps) {
		OccupancyMap &map = *map_pair.second;
		for (const Sphere &sphere: spheres) {
			map.incorporate(sphere.center, [&](const Eigen::Vector3d &point) -> BoundarySample {

				Eigen::Vector3d point_on_sphere = sphere.center + (point - sphere.center).normalized() * sphere.radius;

				return {point_on_sphere, BoundaryType::OCCLUDING};

			});
		}
	}

	std::cout << "At center: " << OccupancyMap::region_type_to_string(maps[0].second->query_at(center)) << std::endl;

	std::vector<std::pair<Eigen::Vector3d, bool>> test_points = generateTestPointsOnSpheres(spheres, 10000,
																							TEST_POINT_DISTANCE_FROM_BOUNDARY);

	enum ErrorType {
		EXPECTED_FREE_BUT_OCCUPIED,
		EXPECTED_OCCUPIED_BUT_FREE,
		CORRECT_FREE,
		CORRECT_OCCUPIED
	};

	// Create a map to store the results of the tests, with the map names as the keys
	std::vector<std::pair<std::string, std::vector<ErrorType>>> results;

	// Iterate over the occupancy maps
	for (const auto &[name, map]: maps) {

		results.emplace_back(name, std::vector<ErrorType>());

		// Iterate over the test points
		for (const auto &[point, expected_result]: test_points) {

			auto expected = expected_result ? OccupancyMap::RegionType::FREE : OccupancyMap::RegionType::UNSEEN;
			auto actual = map->query_at(point);

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

			results.back().second.push_back(error_type);
		}
	}

	for (const auto &[name, result]: results) {
		std::cout << name << std::endl;
		std::cout << "Expected free but occupied: " << std::count(result.begin(), result.end(), EXPECTED_FREE_BUT_OCCUPIED) << std::endl;
		std::cout << "Expected occupied but free: " << std::count(result.begin(), result.end(), EXPECTED_OCCUPIED_BUT_FREE) << std::endl;
		std::cout << "Correct free: " << std::count(result.begin(), result.end(), CORRECT_FREE) << std::endl;
		std::cout << "Correct occupied: " << std::count(result.begin(), result.end(), CORRECT_OCCUPIED) << std::endl;
	}

//	writeResults(spheres, test_points, results);

	return 0;
}


