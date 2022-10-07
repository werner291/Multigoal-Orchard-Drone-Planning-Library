
#ifndef NEW_PLANNERS_DYNAMICBOUNDINGSPHEREALGORITHM_H
#define NEW_PLANNERS_DYNAMICBOUNDINGSPHEREALGORITHM_H


#include <CGAL/Simple_cartesian.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_3.h>
#include <CGAL/Random.h>
#include <geometric_shapes/bodies.h>

#include "OnlinePointCloudMotionControlAlgorithm.h"

class DynamicBoundingSphereAlgorithm : public OnlinePointCloudMotionControlAlgorithm {
public:
	DynamicBoundingSphereAlgorithm(const std::function<void(robot_trajectory::RobotTrajectory)> &trajectoryCallback);

public:

	void updatePointCloud(const moveit::core::RobotState &current_state,
						  const SegmentedPointCloud &segmentedPointCloud) override;

	[[nodiscard]] bodies::BoundingSphere getBoundingSphere();
};

#endif //NEW_PLANNERS_DYNAMICBOUNDINGSPHEREALGORITHM_H
