
#include "StreamingConvexHull.h"
#include "utilities/msgs_utilities.h"
#include "utilities/convex_hull.h"
#include <range/v3/all.hpp>
#include <ompl/util/RandomNumbers.h>

double infzero(double x) {
	if (x > 0) return std::numeric_limits<double>::infinity();
	if (x < 0) return -std::numeric_limits<double>::infinity();
	return 0;
}

StreamingConvexHull::StreamingConvexHull(const std::vector<Eigen::Vector3d> &directions) {
	for (const auto &direction: directions) {
		supporting_set.emplace_back(direction, std::nullopt);
	}
}

StreamingConvexHull StreamingConvexHull::fromSpherifiedCube(size_t segments) {

	// Generate the vertices of a subdivided cube and normalize them.
	std::vector<Eigen::Vector3d> directions;

	for (size_t i = 0; i <= segments; i++) {
		double x = ((double) i / (double) segments) - 0.5;

		for (size_t j = 0; j <= segments; j++) {
			double y = ((double) j / (double) segments) - 0.5;

			for (size_t k = 0; k <= segments; k++) {

				double z = ((double) k / (double) segments) - 0.5;

				// To avoid the interior points, check if we're either minimizing or maximizing one of the coordinates.
				if (i == 0 || i == segments || j == 0 || j == segments || k == 0 || k == segments) {
					directions.emplace_back(x, y, z);
					directions.back().normalize();
				}
			}

		}
	}

	return StreamingConvexHull(directions);

}

bool StreamingConvexHull::addPoint(const Eigen::Vector3d &point) {

	bool changed = false;

	for (auto &supporting_point: supporting_set) {

		bool has_point = supporting_point.second.has_value();
		double candidate_coeff = supporting_point.first.dot(point);
		double original_coeff = has_point ? supporting_point.first.dot(*supporting_point.second) : -std::numeric_limits<double>::infinity();

		if (candidate_coeff > original_coeff) {
			supporting_point.second = point;
			changed = true;
		}
	}

	return changed;
}


shape_msgs::msg::Mesh StreamingConvexHull::toMesh() const {

	auto points =
			supporting_set
			| ranges::views::filter([](const auto &pair) { return pair.second.has_value(); })
			| ranges::views::transform([](const auto &pair) { return msgFromEigen(*pair.second); })
			| ranges::to<std::vector>();

	// deduplicate
	points |= ranges::actions::sort([](const auto &a, const auto &b) {
		if (a.x < b.x) return true;
		if (a.x > b.x) return false;
		if (a.y < b.y) return true;
		if (a.y > a.y) return false;
		return a.z < b.z;
	})
			| ranges::actions::unique([](const auto &a, const auto &b) {
				return a.x == b.x && a.y == b.y && a.z == b.z;
			});

	if (points.size() < 4) {
		shape_msgs::msg::Mesh mesh;

		// Empty.
		return mesh;
	}

	// Check if coplanar
	Eigen::Vector3d v1 = toEigen(points[1]) - toEigen(points[0]);
	Eigen::Vector3d v2 = toEigen(points[2]) - toEigen(points[0]);

	// If we find any non-coplanar points, return the convex hull.

	for (size_t i = 4; i < points.size(); i++) {
		Eigen::Vector3d v3 = toEigen(points[i]) - toEigen(points[0]);
		if (abs(v1.cross(v2).dot(v3)) > 1e-6) {
			return convexHull(points);
		}
	}

	shape_msgs::msg::Mesh mesh;

	// Empty.
	return mesh;

}

bool StreamingConvexHull::addPointsRandomized(const std::vector<Eigen::Vector3d> &points) {

	bool changed = false;

	if (points.size() > 0) {
		ompl::RNG rng;

		// We add 100 points at random.
		for (size_t i = 0; i < 100; i++) {
			changed |= addPoint(points[rng.uniformInt(0, points.size() - 1)]);
		}
	}

	return changed;
}

const std::vector<std::pair<Eigen::Vector3d, std::optional<Eigen::Vector3d>>> &
StreamingConvexHull::getSupportingSet() const {
	return supporting_set;
}
