#include <ompl/util/RandomNumbers.h>
#include "scan_points.h"
#include "../utilities/msgs_utilities.h"

std::vector<ScanTargetPoint> buildScanTargetPoints(const shape_msgs::msg::Mesh &mesh, size_t n) {

	// TODO: Argue uniformity of some kind.

	// Allocate the memory for the points.
	std::vector<ScanTargetPoint> points;
	points.reserve(n);

	ompl::RNG rng;
	for (size_t i = 0; i < n; i++) {

		// Pick a random triangle.
		const auto &triangle = mesh.triangles[rng.uniformInt(0, (int) mesh.triangles.size() - 1)];

		// Convert the triangle points to Eigen vectors.
		const auto &p1 = toEigen(mesh.vertices[triangle.vertex_indices[0]]);
		const auto &p2 = toEigen(mesh.vertices[triangle.vertex_indices[1]]);
		const auto &p3 = toEigen(mesh.vertices[triangle.vertex_indices[2]]);

		// Pick a random point in the triangle.
		const double r1 = rng.uniform01();
		const double r2 = rng.uniform01();

		// Store the point.
		ScanTargetPoint point;
		point.point = p1 + r1 * (p2 - p1) + r2 * (p3 - p1);
		point.normal = (p2 - p1).cross(p3 - p1).normalized();
		points.push_back(point);
	}

	// Return the points.
	return points;

}