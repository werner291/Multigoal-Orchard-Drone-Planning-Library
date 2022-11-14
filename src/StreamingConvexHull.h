
#ifndef NEW_PLANNERS_STREAMINGCONVEXHULL_H
#define NEW_PLANNERS_STREAMINGCONVEXHULL_H

#include <boost/geometry.hpp>
#include <boost/geometry/index/parameters.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <shape_msgs/msg/mesh.hpp>
#include "StreamingMeshHullAlgorithm.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

/**
 * Algorithm based on the thesis "Streaming Algorithms for Approximate Convex Hulls" by Ananya Kumar.
 *
 * It works by maintaining a set of direction unit vectors and a set of supporting points.
 * The supporting points are the points that are furthest in the direction of the direction unit vectors.
 */
class StreamingConvexHull : public StreamingMeshHullAlgorithm {

	std::vector<std::pair<Eigen::Vector3d, std::optional<Eigen::Vector3d>>> supporting_set;
public:
	const std::vector<std::pair<Eigen::Vector3d, std::optional<Eigen::Vector3d>>> &getSupportingSet() const;

public:
	explicit StreamingConvexHull(const std::vector<Eigen::Vector3d>& directions);

	static StreamingConvexHull fromSpherifiedCube(size_t segments);

	virtual bool addPoint(const Eigen::Vector3d &point) override;

	[[nodiscard]] virtual shape_msgs::msg::Mesh toMesh() const override;

};




#endif //NEW_PLANNERS_STREAMINGCONVEXHULL_H
