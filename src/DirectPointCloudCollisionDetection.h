//
// Created by werner on 21-11-22.
//

#ifndef NEW_PLANNERS_DIRECTPOINTCLOUDCOLLISIONDETECTION_H
#define NEW_PLANNERS_DIRECTPOINTCLOUDCOLLISIONDETECTION_H

#include <vector>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Orthogonal_incremental_neighbor_search.h>
#include <geometric_shapes/shapes.h>

#include "RobotPath.h"

static const double COLLISION_DETECTION_MAX_STEP = 0.2;

/**
 * A collision detector that checks for collisions between a point cloud and robot states.
 *
 * TODO: A lot of redundancy in this class due to slightly different queries. Can this be refactored?
 */
class DirectPointCloudCollisionDetection {

	using K = CGAL::Simple_cartesian<double>;
	using Point = K::Point_3;
	using TreeTraits = CGAL::Search_traits_3<K>;
	using NN_incremental_search = CGAL::Orthogonal_incremental_neighbor_search<TreeTraits>;
	using NN_incremental_iterator = NN_incremental_search::iterator;
	using Tree = NN_incremental_search::Tree;

	// TODO: Is this the right structure? KdTree cannot be dynamically updated,
	// so it needs to be rebuilt after every point cloud update.
	Tree tree;

	/// A version number that is incremented every time that points are added.
	size_t version = 0;
public:

	/**
	 * Get the version number, which will be different every time that points are added.
	 *
	 * Useful for checking whether the point cloud has changed since the last time that
	 * a collision check was performed.
	 *
	 * @return 		The version number.
	 */
	size_t getVersion() const;

	/**
	 * Add points to collide with.
	 *
	 * Note: under current implementation, this will lazily rebuild the tree upon the next query.
	 *
	 * @param points  The points to add.
	 */
	void addPoints(std::vector<Eigen::Vector3d> points);

	[[nodiscard]] bool checkCollision(const moveit::core::RobotState &state) const;

	[[nodiscard]] bool checkCollisionInterpolated(const moveit::core::RobotState &state1, const moveit::core::RobotState &state2, double maxStep) const;

	[[nodiscard]] bool checkCollisionInterpolated(const RobotPath& path, double maxStep) const;

	[[nodiscard]] bool checkCollision(const shapes::ShapeConstPtr &shape, const Eigen::Isometry3d &pose) const;

	bool checkCollision(const shapes::Box &shape, const Eigen::Isometry3d &pose) const;

	double distanceToCollision(const moveit::core::RobotState &state, double maxDistance) const;

	double distanceToCollision(const shapes::ShapeConstPtr &shape, const Eigen::Isometry3d &pose, double maxDistance) const;

	double distanceToCollision(const shapes::Box &shape, const Eigen::Isometry3d &pose, double maxDistance) const;

	struct ClosestPointOnRobot {
		Eigen::Vector3d point;
		Eigen::Vector3d on_robot;
		const moveit::core::LinkModel* link;
		double distance;
	};

	struct ClosestPoint {
		Eigen::Vector3d point_on_obstacle;
		Eigen::Vector3d point_on_query;
		double distance;
	};

	[[nodiscard]] std::optional<ClosestPointOnRobot> closestPoint(const moveit::core::RobotState &state, double maxDistance) const;

	[[nodiscard]] std::optional<ClosestPoint> closestPoint(const shapes::ShapeConstPtr &shape, const Eigen::Isometry3d &pose, double maxDistance) const;

	[[nodiscard]] std::optional<ClosestPoint> closestPoint(const shapes::Box &shape, const Eigen::Isometry3d &pose, double maxDistance) const;

};


#endif //NEW_PLANNERS_DIRECTPOINTCLOUDCOLLISIONDETECTION_H
