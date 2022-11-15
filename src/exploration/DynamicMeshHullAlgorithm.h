
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
#include "../RobotPath.h"
#include "../AnytimeOptimalInsertion.h"

static const double TARGET_POINT_DEDUP_THRESHOLD = 0.05;

static const int ITERATION_COMPUTE_TIME_LIMIT = 30;

static const double DISTANCE_CONSIDERED_SCANNED = 0.1;

/**
 * A motion control algorithm that uses a dynamic mesh to represent the outer "shell" of the obstacles,
 * updating this hull as new data comes in from the robot's sensors.
 */
class DynamicMeshHullAlgorithm : public OnlinePointCloudMotionControlAlgorithm {

	/// The last-known position of the robot; this is assumed to be very recent.
	moveit::core::RobotState last_robot_state;
	/// The end-effector position of the last-known robot state; this is a cached value derived from last_robot_state.
	Eigen::Vector3d last_end_effector_position;

	/// A spatial index used to de-duplicate target points, as the robot may see the same target
	/// multiple times with slight, non-significant differences in position.
	HashedSpatialIndex<std::monostate> targetPointsDedupIndex;

	/// An algorithm that computes a mesh that approximately envelops a stream of points. (TODO: guarantees?)
	std::shared_ptr<StreamingMeshHullAlgorithm> pointstream_to_hull;

	/// An algorithm that computes the optimial visitation order of a set of points, given a starting point.
	AnytimeOptimalInsertion<size_t> visit_ordering;

	/// A point somewhere in space that must be inspected, paired with the point on the mesh hull that is closest to it.
	struct TargetPoint {
		///	The original point that must be inspected.
		const Eigen::Vector3d observed_location;
		/// The point on the mesh hull that is closest to the observed_location;
		/// this is a semi-heavy operation to compute, so we cache it here.
		Eigen::Vector3d hull_location;
	};

	/// A list of target points to inspect. The indices of this list are used as keys in the visit_ordering,
	/// so we must not change the order of this list.
	std::vector<TargetPoint> targetPointsOnChullSurface;

	RobotPath last_path;

public:
	/// Return the visitation ordering algorithm state, for debugging and visualization
	[[nodiscard]] const AnytimeOptimalInsertion<size_t> &getVisitOrdering() const;

	/// Return the list of known target points, for debugging and visualization
	[[nodiscard]] const std::vector<TargetPoint> &getTargetPointsOnChullSurface() const;

	/// Return the algorithm that computes the hull around obstacle points, for debugging and visualization
	[[nodiscard]] const std::shared_ptr<StreamingMeshHullAlgorithm> &getConvexHull() const;

	/**
	 * Constructor. Initializes the algorithm with the given robot model in a given initial state
	 * and a few parameters; otherwise, we assume no prior knowledge of the environment.
	 *
	 * @param initial_state 			The initial state of the robot.
	 * @param trajectoryCallback 		A callback that will be called when a new trajectory is computed.
	 * @param pointstreamToHull 		The algorithm that computes the hull around obstacle points.
	 */
	DynamicMeshHullAlgorithm(const moveit::core::RobotState &initial_state,
							 const std::function<void(robot_trajectory::RobotTrajectory)> &trajectoryCallback,
							 std::shared_ptr<StreamingMeshHullAlgorithm> pointstreamToHull);

	/**
	 * Update the algorithm's knowledge of the environment with a new point cloud
	 * (read from sensors) and current robot state.
	 *
	 * @param current_state 			The current state of the robot.
	 * @param segmentedPointCloud 		The point cloud read from the robot's sensors (assumed pre-segmented in a perfect manner)
	 */
	void updatePointCloud(const moveit::core::RobotState &current_state,
						  const SegmentedPointCloud &segmentedPointCloud) override;

	/**
	 * Evaluate the current trajectory based on current knowledge of the environment, and update it if needed.
	 */
	void updateTrajectory();
};

#endif //NEW_PLANNERS_DYNAMICMESHHULLALGORITHM_H
