
#ifndef NEW_PLANNERS_SEGMENTEDPOINTCLOUD_H
#define NEW_PLANNERS_SEGMENTEDPOINTCLOUD_H

#include <array>
#include <Eigen/Core>
#include <optional>

/**
 * A point cloud, segmented into different categories.
 */
struct SegmentedPointCloud {

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

		bool operator==(const Point &other) const {
			return type == other.type && position == other.position;
		}

		bool operator!=(const Point &other) const {
			return !(*this == other);
		}
	};

	std::vector<Point> points;
};

/**
 * Get the leaf points from the given point cloud. It additionally filters out points that are too close to the ground.
 *
 * @param segmentedPointCloud 		The point cloud to get the leaf points from.
 * @return 							The leaf points.
 */
std::vector<Eigen::Vector3d> isolateLeafPoints(const SegmentedPointCloud &segmentedPointCloud);

#endif //NEW_PLANNERS_SEGMENTEDPOINTCLOUD_H
