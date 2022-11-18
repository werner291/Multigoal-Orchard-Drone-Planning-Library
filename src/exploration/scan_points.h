
#ifndef NEW_PLANNERS_SCAN_POINTS_H
#define NEW_PLANNERS_SCAN_POINTS_H

#include <Eigen/Core>
#include <shape_msgs/msg/mesh.hpp>

struct PointWithNormal {
	Eigen::Vector3d point;
	Eigen::Vector3d normal;
};

/**
 * Given a mesh, return a point picked uniformly at random from the surface of the mesh.
 *
 * Probability of picking a point from a given triangle is proportional to that triangle's area.
 *
 * @param mesh 			The mesh to pick a point from.
 * @return 				A point picked uniformly at random from the surface of the mesh.
 */
std::vector<PointWithNormal> samplePointsOnMeshUniformly(const shape_msgs::msg::Mesh &mesh, size_t n);

/**
 * A point somewhere in the world that needs to be scanned.
 */
struct ScanTargetPoint {
	/// The position of the point
	Eigen::Vector3d point { 0.0, 0.0, 0.0 };

	Eigen::Vector3d normal { 0.0, 0.0, 0.0 };

	size_t apple_id { 0 };
};

/**
 *
 * Given a vector of meshes, return a vector of points that need to be scanned,
 * including a surface normal and the id of the mesh in the vector.
 *
 * @param mesh 		The mesh to sample points on.
 * @param n 		The number of points to sample per mesh.
 * @return 			The sampled points.
 */
std::vector<ScanTargetPoint> buildScanTargetPoints(const std::vector<shape_msgs::msg::Mesh> &meshes, size_t n = 10);

#endif //NEW_PLANNERS_SCAN_POINTS_H
