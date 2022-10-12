
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

template<typename V>
class DynamicOrderOptimization {

	std::vector<V> visit_ordering;
	std::function<double(const V &)> first_distance_function;
	std::function<double(const V&, const V&)> distance_function;
public:

	explicit DynamicOrderOptimization(const std::function<double(const V &)>& firstDistanceFunction,
									  const std::function<double(const V &, const V &)> &distanceFunction)
			: distance_function(distanceFunction), first_distance_function(firstDistanceFunction) {
	}

	const std::vector<V> &getVisitOrdering() const {
		return visit_ordering;
	}

	void iterate() {

		// Pick a random index.
		size_t index = rand() % visit_ordering.size();

		// Remove it.
		V removed = visit_ordering[index];
		visit_ordering.erase(visit_ordering.begin() + index);

		// Re-insert it.
		insert(removed);

	}

	void insert(V item) {

		if (visit_ordering.empty()) {
			visit_ordering.push_back(item);
			return;
		} else {

			size_t best_insertion_spot = 0;
			double least_costly_distance = first_distance_function(item) + distance_function(item, visit_ordering[0]);

			for (size_t i = 1; i < visit_ordering.size(); ++i) {
				double distance = distance_function(item, visit_ordering[i]) + distance_function(item, visit_ordering[i-1]);
				if (distance < least_costly_distance) {
					least_costly_distance = distance;
					best_insertion_spot = i;
				}
			}

			visit_ordering.insert(visit_ordering.begin() + best_insertion_spot, item);

		}

	}

};

class DynamicConvexHullAlgorithm : public OnlinePointCloudMotionControlAlgorithm {

	Eigen::Vector3d last_end_effector_position;

	HashedSpatialIndex<std::monostate> targetPoints;
	size_t next_point_id = 0;

	StreamingConvexHull convexHull;

	DynamicOrderOptimization<size_t> visit_ordering;
public:
	const DynamicOrderOptimization<size_t> &getVisitOrdering() const;

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
