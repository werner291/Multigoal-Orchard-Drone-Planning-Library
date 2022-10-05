
#ifndef NEW_PLANNERS_SCAN_POINTS_H
#define NEW_PLANNERS_SCAN_POINTS_H

#include <Eigen/Core>
#include <shape_msgs/msg/mesh.hpp>

/**
 * A point somewhere in the world that needs to be scanned.
 */
struct ScanTargetPoint {
	/// The position of the point
	Eigen::Vector3d point { 0.0, 0.0, 0.0 };

	Eigen::Vector3d normal { 0.0, 0.0, 0.0 };
};

/**
 *
 * Sample a set of n points (ScanTargetPoint) on the surface of the given mesh, with normals.
 *
 * @param mesh 		The mesh to sample points on.
 * @param n 		The number of points to sample.
 * @return 			The sampled points.
 */
std::vector<ScanTargetPoint> buildScanTargetPoints(const shape_msgs::msg::Mesh &mesh, size_t n);

#endif //NEW_PLANNERS_SCAN_POINTS_H
