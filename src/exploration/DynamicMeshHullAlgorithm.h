
#ifndef NEW_PLANNERS_DYNAMICMESHHULLALGORITHM_H
#define NEW_PLANNERS_DYNAMICMESHHULLALGORITHM_H

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

class DynamicMeshHullAlgorithm : public OnlinePointCloudMotionControlAlgorithm {

	Eigen::Vector3d last_end_effector_position;

	HashedSpatialIndex<std::monostate> targetPoints;
	size_t next_point_id = 0;

	std::shared_ptr<StreamingMeshHullAlgorithm> pointstream_to_hull;

	AnytimeOptimalInsertion<size_t> visit_ordering;

	struct TargetPoint {
		const Eigen::Vector3d observed_location;
		Eigen::Vector3d hull_location;
		bool visited;
	};

	std::vector<TargetPoint> targetPointsOnChullSurface;

public:
	[[nodiscard]] const AnytimeOptimalInsertion<size_t> &getVisitOrdering() const;

	[[nodiscard]] const std::vector<TargetPoint> &getTargetPointsOnChullSurface() const;

	[[nodiscard]] const std::shared_ptr<StreamingMeshHullAlgorithm> &getConvexHull() const;

	DynamicMeshHullAlgorithm(const moveit::core::RobotState &initial_state,
							 const std::function<void(robot_trajectory::RobotTrajectory)> &trajectoryCallback,
							 std::shared_ptr<StreamingMeshHullAlgorithm> pointstreamToHull);

	void updatePointCloud(const moveit::core::RobotState &current_state,
						  const SegmentedPointCloud &segmentedPointCloud) override;

	void updateTrajectory(const moveit::core::RobotState &current_state, const shape_msgs::msg::Mesh &chull);
};

#endif //NEW_PLANNERS_DYNAMICMESHHULLALGORITHM_H
