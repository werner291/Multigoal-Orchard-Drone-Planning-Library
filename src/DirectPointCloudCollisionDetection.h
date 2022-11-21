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

/**
 * A collision detector that checks for collisions between a point cloud and robot states.
 */
class DirectPointCloudCollisionDetection {

	using K = CGAL::Simple_cartesian<double>;
	using Point = K::Point_3;
	using TreeTraits = CGAL::Search_traits_3<K>;
	using NN_incremental_search = CGAL::Orthogonal_incremental_neighbor_search<TreeTraits>;
	using NN_iterator = NN_incremental_search::iterator;
	using Tree = NN_incremental_search::Tree;

	// TODO: Is this the right structure? KdTree cannot be dynamically updated,
	// so it needs to be rebuilt after every point cloud update.
	Tree tree;

public:

	/**
	 * Add points to collide with.
	 *
	 * Note: under current implementation, this will lazily rebuild the tree upon the next query.
	 *
	 * @param points  The points to add.
	 */
	void addPoints(std::vector<Eigen::Vector3d> points);

	[[nodiscard]] bool checkCollision(const moveit::core::RobotState &state) const;

	[[nodiscard]] bool checkCollision(const shapes::ShapeConstPtr &shape, const Eigen::Isometry3d &pose) const;

	bool checkCollision(const shapes::Box &shape, const Eigen::Isometry3d &pose) const;

};


#endif //NEW_PLANNERS_DIRECTPOINTCLOUDCOLLISIONDETECTION_H
