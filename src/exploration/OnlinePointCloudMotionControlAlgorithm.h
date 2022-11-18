
#ifndef NEW_PLANNERS_ONLINEPOINTCLOUDMOTIONCONTROLALGORITHM_H
#define NEW_PLANNERS_ONLINEPOINTCLOUDMOTIONCONTROLALGORITHM_H

#include <functional>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "SegmentedPointCloud.h"
#include "../TreeMeshes.h"

/**
 * An axis-aligned bounding box that is guaranteed to contain all points to be explored.
 *
 * he robot need not search outside for points of interest, although it may need to seek
 * out obstacles outside the box.
 */
struct TargetBoundingBox {
	Eigen::AlignedBox3d boundingBox;
};

/**
 * Abstract class for online motion control algorithms.
 */
class OnlinePointCloudMotionControlAlgorithm {

protected:
	/**
	 * Callback function to be called when a new trajectory is computed.
	 *
	 * @param trajectory 	Computed trajectory, parameterized by time relative to time of calling (starts at 0).
	 */
	std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback;

public:
	/**
	 * Constructor.
	 *
	 * @param trajectoryCallback 	Callback function to be called when a new trajectory is computed.
	 */
	explicit OnlinePointCloudMotionControlAlgorithm(std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback);

	/**
	 * Inform the algorithm about the current state of the robot and what it sees.
	 *
	 * @param current_state 			Current state of the robot.
	 * @param segmentedPointCloud 		Current point cloud from the sensor, segmented into different types.
	 */
	virtual void update(const moveit::core::RobotState& current_state, const SegmentedPointCloud::ByType &segmentedPointCloud) = 0;

};

#endif //NEW_PLANNERS_ONLINEPOINTCLOUDMOTIONCONTROLALGORITHM_H
