#ifndef NEW_PLANNERS_STREAMINGCONVEXHULL_H
#define NEW_PLANNERS_STREAMINGCONVEXHULL_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <shape_msgs/msg/mesh.hpp>

/**
 * @class StreamingConvexHull
 * @brief A class implementing the streaming convex hull algorithm
 *
 * This algorithm is based on the thesis "Streaming Algorithms for Approximate Convex Hulls" by Ananya Kumar.
 * It works by maintaining a set of direction unit vectors and a set of supporting points.
 * The supporting points are the points that are furthest in the direction of the direction unit vectors.
 */
class StreamingConvexHull {
private:
	/// The supporting set of direction unit vectors and their corresponding supporting points
	std::vector<std::pair<Eigen::Vector3d, std::optional<Eigen::Vector3d>>> supporting_set;

public:
	/**
	 * @brief Get the supporting set of the convex hull
	 *
	 * @return A constant reference to the supporting set
	 */
	[[nodiscard]] const std::vector<std::pair<Eigen::Vector3d, std::optional<Eigen::Vector3d>>> &
	getSupportingSet() const;

	/**
	 * @brief Construct a StreamingConvexHull object with a given set of direction vectors
	 *
	 * @param directions A vector of direction unit vectors
	 */
	explicit StreamingConvexHull(const std::vector<Eigen::Vector3d> &directions);

	/**
	 * @brief Create a StreamingConvexHull object from a spherified cube
	 *
	 * @param segments The number of segments in the spherified cube
	 *
	 * @return A StreamingConvexHull object
	 */
	static StreamingConvexHull fromSpherifiedCube(size_t segments);

	/**
	 * @brief Add a point to the convex hull
	 *
	 * @param point The point to add
	 *
	 * @return true if the supporting set was changed, false otherwise
	 */
	bool addPoint(const Eigen::Vector3d &point);

	/**
	 * @brief Convert the convex hull to a mesh
	 *
	 * @return The mesh representation of the convex hull
	 */
	[[nodiscard]] shape_msgs::msg::Mesh toMesh() const;
};

#endif //NEW_PLANNERS_STREAMINGCONVEXHULL_H
