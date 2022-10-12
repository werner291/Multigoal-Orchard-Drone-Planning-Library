
#ifndef NEW_PLANNERS_DYNAMICCONVEXHULLALGORITHM_H
#define NEW_PLANNERS_DYNAMICCONVEXHULLALGORITHM_H


#include <CGAL/Simple_cartesian.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_3.h>
#include <CGAL/Random.h>
#include <geometric_shapes/bodies.h>

#include "OnlinePointCloudMotionControlAlgorithm.h"
#include "../StreamingConvexHull.h"
#include "../HashedSpatialIndex.h"

#include "../AnytimeOptimalInsertion.h"


class DynamicConvexHullAlgorithm : public OnlinePointCloudMotionControlAlgorithm {

	Eigen::Vector3d last_end_effector_position;

	HashedSpatialIndex<std::monostate> targetPoints;
	size_t next_point_id = 0;

	StreamingConvexHull convexHull;

	AnytimeOptimalInsertion<size_t> visit_ordering;
public:
	const AnytimeOptimalInsertion<size_t> &getVisitOrdering() const;

private:

	std::vector<std::pair<const Eigen::Vector3d, Eigen::Vector3d>> targetPointsOnChullSurface;
public:
	const std::vector<std::pair<const Eigen::Vector3d, Eigen::Vector3d>> &getTargetPointsOnChullSurface() const;

	const StreamingConvexHull &getConvexHull() const;

	DynamicConvexHullAlgorithm(const moveit::core::RobotState &initial_state, const std::function<void(robot_trajectory::RobotTrajectory)> &trajectoryCallback);

	void updatePointCloud(const moveit::core::RobotState &current_state,
						  const SegmentedPointCloud &segmentedPointCloud) override;

};

#endif //NEW_PLANNERS_DYNAMICCONVEXHULLALGORITHM_H
