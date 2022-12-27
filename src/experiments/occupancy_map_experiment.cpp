#include <random>
#include <Eigen/Dense>
#include <vector>

// Struct to represent a sphere
struct Sphere {
  Eigen::Vector3d center;
  double radius;
};

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
 * Generates random points on the surface of a sphere
 *
 * @param sphere   Sphere object to generate points on
 * @param num_points  Number of points to generate
 * @return Vector of 3D points on the surface of the sphere
 */
std::vector<Eigen::Vector3d> generateSurfacePoints(const Sphere& sphere, int num_points) {
  // Set the random seed
  std::random_device rd;
  std::mt19937 gen(rd());

  // Create a normal distribution for generating random points on the surface of the sphere
  std::normal_distribution<> coord_dist(0.0, 1.0);

  // Use the generator function constructor of std::vector to generate the points
  std::vector<Eigen::Vector3d> points(num_points, [&](int) {
    // Generate random Cartesian coordinates
    Eigen::Vector3d coords(coord_dist(gen), coord_dist(gen), coord_dist(gen));

    // Normalize the coordinates
    coords.normalize();

    // Scale the coordinates by the radius of the sphere
    coords *= sphere.radius;

    // Shift the point by the center of the sphere
    Eigen::Vector3d point = sphere.center + coords;

    return point;
  });

  return points;
}

/**
 * Check if a point lies inside a sphere
 *
 * @param point  Point to check
 * @param sphere  Sphere to check
 * @return True if the point lies inside the sphere, false otherwise
 */
bool pointInSphere(const Eigen::Vector3d& point, const Sphere& sphere) {
  return (point - sphere.center).norm() < sphere.radius;
}

/**
 * Check if a point lies inside the union of a set of spheres
 *
 * @param point  Point to check
 * @param spheres  Vector of spheres to check
 * @return True if the point lies inside the union of the spheres, false otherwise
 */
bool pointInSpheres(const Eigen::Vector3d& point, const std::vector<Sphere>& spheres) {
  // Check if any of the spheres contain the point
  return std::any_of(spheres.begin(), spheres.end(), [&](const Sphere& sphere) {
    return pointInSphere(point, sphere);
  });
}

std::vector<Eigen::Vector3d> generateTestPoints(const Sphere& sphere, int n, double max_distance) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(n);

  // Generate random points within a bounding box that encloses the sphere
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(sphere.center.x() - sphere.radius - max_distance, sphere.center.x() + sphere.radius + max_distance);
  std::uniform_real_distribution<> dis_y(sphere.center.y() - sphere.radius - max_distance, sphere.center.y() + sphere.radius + max_distance);
  std::uniform_real_distribution<> dis_z(sphere.center.z() - sphere.radius - max_distance, sphere.center.z() + sphere.radius + max_distance);

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
 * @return Vector of point/boolean pairs indicating whether each point lies within the union of all the spheres in the given vector
 */
std::vector<Eigen::Vector3d> generateTestPoints(
    const std::vector<Sphere>& spheres, int num_test_points, double max_distance) {
  // Generate test points near the boundary surface of each sphere
  return
      // Generate test points for each sphere
      spheres
      | ranges::view::transform([&](const Sphere& sphere) {
          return generateTestPoints(sphere, num_test_points, max_distance);
        })
      // Concatenate all the test points into a single range
      | ranges::view::concat
      // Convert the range to a vector
      | ranges::to_vector;
}




int main() {
  // Generate 100 random spheres
  std::vector<Sphere> spheres = generateRandomSpheres(100);

  // Do something with the spheres (e.g., add them to a list or scene)
  // Create a vector to store the OccupancyMap objects and their names
  std::vector<std::pair<std::string, std::unique_ptr<OccupancyMap>>> maps {
    {"HierarchicalCategoricalOccupancyOctree", std::make_unique<HierarchicalCategoricalOccupancyOctree>()},
    {"HierarchicalBoundaryCellAnnotatedRegionOctree", std::make_unique<HierarchicalBoundaryCellAnnotatedRegionOctree>()}
  };

  // Incorporate the spheres into all OccupancyMap objects
  for (auto& map_pair : maps) {
    OccupancyMap& map = *map_pair.second;
    for (const Sphere& sphere : spheres) {
      map.incorporate(sphere.center, [&](const Eigen::Vector3d& point) {
        return (point - sphere.center).norm() < sphere.radius;
      });
    }
  }
  
  std::vector<std::pair<Eigen::Vector3d, bool>> test_points = generateTestPoints(spheres, 1000, 0.1);
  
  // Create a map to store the results of the tests, with the map names as the keys
  std::map<std::string, std::vector<bool>> test_results;

  // Iterate over the test points
  for (const auto& [point, expected_result] : test_points) {
    // Iterate over the occupancy maps
    for (const auto& [name, map] : maps) {
      // Test the point against the map
      bool actual_result = (map->query_at(point) == OccupancyMap::RegionType::OCCUPIED);

      // Record the result of the test
      test_results[name].emplace_back(actual_result == expected_result);
    }
  }

  // Generate a timestamp for the current time
  auto now = std::time(nullptr);
  std::tm tm = *std::localtime(&now);
  std::stringstream timestamp;
  timestamp << std::put_time(&tm, "%Y%m%d-%H%M%S");

  // Create a JSON object to store the data
  Json::Value root;

  // Add the spheres to the JSON object
  Json::Value spheres_json(Json::arrayValue);
  for (const auto& sphere : spheres) {
    Json::Value sphere_json;
    sphere_json["center"] = toJson(sphere.center);
    sphere_json["radius"] = sphere.radius;
    spheres_json.append(sphere_json);
  }
  root["spheres"] = spheres_json;

  // Add the points to the JSON object
  Json::Value points_json(Json::arrayValue);
  for (const auto& point : test_points) {
    Json::Value point_json;
    point_json["point"] = toJson(point.first);
    point_json["expected_result"] = point.second;
    points_json.append(point_json);
  }
  root["points"] = points_json;

  // Add the test results to the JSON object
  Json::Value results_json(Json::objectValue);
  for (const auto& [name, results] : test_results) {
    Json::Value results_array(Json::arrayValue);
    for (const auto& result : results) {
      results_array.append(result);
    }
    results_json[name] = results_array;
  }
  root["results"] = results_json;

  // Write the JSON object to a file
  std::ofstream out_file("analysis/data/test_results_" + timestamp.str() + ".json");
  out_file << root.toStyledString();
  out_file.close();

  return 0;
}


