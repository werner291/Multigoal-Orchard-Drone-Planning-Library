
#ifndef NEW_PLANNERS_SEGMENTEDPOINTCLOUD_H
#define NEW_PLANNERS_SEGMENTEDPOINTCLOUD_H

#include <array>
#include <Eigen/Core>
#include <optional>

/**
 * Classification categories for points in SegmentedPointCloud.
 */
enum PointType {
	/// Point lies on the surface of a hard obstacle.
	PT_OBSTACLE,
	/// Point lies on the surface of a soft obstacle: it may be passed, but you cannot see through it.
	PT_SOFT_OBSTACLE,
	/// Point lies on the surface of a target object.
	PT_TARGET
};

/**
 * A point in a SegmentedPointCloud, labeled by position and point type.
 */
struct Point {
	/// Type of the point.
	PointType type;
	/// Position of the point
	Eigen::Vector3d position;
};

/**
 * A point cloud, segmented into different categories.
 */
struct SegmentedPointCloud {
	std::vector<Point> points;
};

#endif //NEW_PLANNERS_SEGMENTEDPOINTCLOUD_H
