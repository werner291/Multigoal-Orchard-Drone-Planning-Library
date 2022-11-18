#include <ompl/util/RandomNumbers.h>
#include "scan_points.h"
#include "../utilities/msgs_utilities.h"

std::vector<ScanTargetPoint> buildScanTargetPoints(const std::vector<shape_msgs::msg::Mesh> &mesh, size_t n) {

	// Allocate space for the points
	std::vector<ScanTargetPoint> points;
	points.reserve(mesh.size() * n);

	// Sample points on each mesh
	for (size_t i = 0; i < mesh.size(); i++) {
		auto mesh_points = samplePointsOnMeshUniformly(mesh[i], n);
		for (auto &point : mesh_points) {
			points.push_back({point.point, point.normal, i});
		}
	}

	return points;

}

std::vector<PointWithNormal> samplePointsOnMeshUniformly(const shape_msgs::msg::Mesh &mesh, size_t n) {

	std::vector<double> cumulative_areas;
	cumulative_areas.reserve(mesh.triangles.size());

	double total_area = 0.0;
	for (const auto &triangle : mesh.triangles) {

		// Convert the triangle points to Eigen vectors.
		const auto &p1 = toEigen(mesh.vertices[triangle.vertex_indices[0]]);
		const auto &p2 = toEigen(mesh.vertices[triangle.vertex_indices[1]]);
		const auto &p3 = toEigen(mesh.vertices[triangle.vertex_indices[2]]);

		// Compute the area of the triangle.
		const double area = 0.5 * (p2 - p1).cross(p3 - p1).norm();

		// Add the area to the total.
		total_area += area;

		// Add the area to the cumulative areas.
		cumulative_areas.push_back(total_area);
	}

	// Allocate the memory for the points.
	std::vector<PointWithNormal> points;
	points.reserve(n);

	ompl::RNG rng;
	for (size_t i = 0; i < n; i++) {

		// Pick a random triangle.
		const double r = rng.uniform01() * total_area;
		const auto it = std::lower_bound(cumulative_areas.begin(), cumulative_areas.end(), r);
		const auto &triangle = mesh.triangles[it - cumulative_areas.begin()];

		// Convert the triangle points to Eigen vectors.
		const auto &p1 = toEigen(mesh.vertices[triangle.vertex_indices[0]]);
		const auto &p2 = toEigen(mesh.vertices[triangle.vertex_indices[1]]);
		const auto &p3 = toEigen(mesh.vertices[triangle.vertex_indices[2]]);

		// Pick a random point in the triangle; see https://math.stackexchange.com/a/18686 for an argument why taking
		// the square root of the random numbers yields a uniform distribution.
		const double r1 = sqrt(rng.uniform01());
		const double r2 = sqrt(rng.uniform01());

		// Calculate the point and add it to the list.
		points.push_back(PointWithNormal {
			// Point
			(1 - r1) * p1 + r1 * (1 - r2) * p2 + r1 * r2 * p3,
			// Normal
			(p2 - p1).cross(p3 - p1).normalized()
		});
	}

	// Return the points.
	return points;

}
