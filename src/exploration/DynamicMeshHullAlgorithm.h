
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
#include "../shell_space/CGALMeshShell.h"
#include "../DirectPointCloudCollisionDetection.h"
#include "../utilities/moveit.h"

static const double TARGET_POINT_DEDUP_THRESHOLD = 0.05;

static const int ITERATION_COMPUTE_TIME_LIMIT = 30;

static const double DISTANCE_CONSIDERED_SCANNED = 0.1;

static const double PADDING = 0.5;

static const double COLLISION_DETECTION_MAX_STEP = 0.2;


/**
 * A motion control algorithm that uses a dynamic mesh to represent the outer "shell" of the obstacles,
 * updating this hull as new data comes in from the robot's sensors.
 *
 * TODO: This is turning into a God class. Refactor if possible.
 */
class DynamicMeshHullAlgorithm : public OnlinePointCloudMotionControlAlgorithm {

	using target_id = size_t;

	RobotPastTrace robot_past;

	/// The end-effector position of the last-known robot state; this is a cached value derived from last_robot_state.
	Eigen::Vector3d last_end_effector_position;

	/// A spatial index used to de-duplicate target points, as the robot may see the same target
	/// multiple times with slight, non-significant differences in position.
	HashedSpatialIndex<std::monostate> targetPointsDedupIndex;

	/// An algorithm that computes a mesh that approximately envelops a stream of points. (TODO: guarantees?)
	std::shared_ptr<StreamingMeshHullAlgorithm> pointstream_to_hull;

	/// An algorithm that computes the optimal visitation order of a set of points, given a starting point.
	AnytimeOptimalInsertion<size_t> visit_ordering;

	DirectPointCloudCollisionDetection collision_detector;

	/// A point somewhere in space that must be inspected, paired with the point on the mesh hull that is on_which_mesh to it.
	struct TargetPoint {

		///	The original point that must be inspected.
		const SegmentedPointCloud::TargetPoint observed_location;

		/// The point on the mesh hull that is on_which_mesh to the observed_location;
		/// this is a semi-heavy operation to compute, so we cache it here.
		Eigen::Vector3d hull_location;

		/// If optional has a value, contains a RobotPath that can be used to reach the target point
		/// from the shell; it was collision-free when it was computed.
		std::optional<RobotPath> approach_path_cache;

		size_t collision_version = 0;

	};

	/// A list of target points to inspect. The indices of this list are used as keys in the visit_ordering,
	/// so we must not change the order of this list.
	std::vector<TargetPoint> targetPointsOnChullSurface;


	/// The mesh hull that represents the outer shell of the obstacle points.
	/// Will be null if not enough non-coplanar points have been seen to compute a hull
	std::shared_ptr<CGALMeshShell> cgal_hull {nullptr};

	/**
	 * Incorporate a new point cloud into the algorithm's knowledge about the world.
	 *
	 * @param segmentedPointCloud 		The point cloud to incorporate.
	 */
	void updatePointCloud(const SegmentedPointCloud::ByType &segmentedPointCloud );

	/**
	 * Evaluate the current trajectory based on current knowledge of the environment, and update it if needed.
	 */
	void updateTrajectory();

	/**
	 * The current most up-to-date trajectory/path, last emitted from the trajectoryCallback.
	 *
	 * Every segment represents a path from somewhere in space to a target point, matching the order from visit_ordering at the time of emission.
	 *
	 * Note: the first waypoint in this path is NOT the current robot state; that gets added in emitUpdatedTrajectory().
	 */
	std::vector<RobotPath> lastPath;

	/**
	 * Concatenate the portions of lastPath and prepend the current robot state to it, then emit it.
	 */
	void emitUpdatedPath();

	/**
	 * Check if the current robot state is on any of the segments of lastPath, and delete all leading up to that point.
	 *
	 * Postcondition: the current robot state is NOT on the path
	 */
	void advance_path_to_current();

	/**
	 * Search along the current path and compare it to the planned visitation order and convex hull,
	 * then cut off the path as soon as it starts to deviate or might cause a collision.
	 */
	void cut_invalid_future();

	/**
	 * Deform the currently-planned path to increase desirable properties,
	 * such as smoothness and sensor coverage.
	 */
	void optimizePlan();

	/**
	 * Extend the current trajectory as much as possible within the given time limit
	 *
	 * TODO: This doesn't quite make sense performance-wise, as most of this will be invalidated by the next update.
	 * It does make for some nice visualizations, though, so we'll maybe want to make this behavior configurable.
	 *
	 * Alternatively, maybe something based on improving path quality until convergence and/or deadline?
	 *
	 * @param deadline 			The time at which to stop extending the trajectory. At least one segment will be added.
	 * @param shell				The mesh shell that represents the outer shell of the obstacle points (TODO: can we replace this with cgal_hull?)
	 * @param shell_space		The CGAL hull, formulated in terms of MoveIt terms.
	 *
	 * TODO: Can we consolidate these parameters?
	 *
	 */
	void extend_plan(const std::chrono::high_resolution_clock::time_point &deadline,
					 const std::shared_ptr<ArmHorizontalDecorator<CGALMeshShellPoint>> &shell,
					 const MoveItShellSpace<CGALMeshShellPoint> &shell_space);

	/**
	 * Compute an approach path to a target point. That is: a path from a shell state near the target point to the target point.
	 *
	 * This operation is fairly expensive, but we cache the result if the shell hasn't changed since the last time we computed it,
	 * and if no collisions have been introduced since then.
	 *
	 * @param target_index		The index of the target point to compute an approach path to.
	 * @return					A path from a shell state near the target point to the target point.
	 */
	std::optional<RobotPath> approachPathForTarget(target_id target_index);

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
	void update(const moveit::core::RobotState &current_state,
				const SegmentedPointCloud::ByType &segmentedPointCloud) override;

	void processObstaclePoints(const std::vector<Eigen::Vector3d> &soft_obstacle);

	void processTargetPoints(const std::vector<SegmentedPointCloud::TargetPoint> &target);

	void updateShell();

	void removeVisitedTargets();

	static void removeFrontReversal(const moveit::core::RobotState &from_state, RobotPath &path) ;

	RobotPath computeInitialApproachPath(const MoveItShellSpace<CGALMeshShellPoint> &shell_space);
};

#endif //NEW_PLANNERS_DYNAMICMESHHULLALGORITHM_H
