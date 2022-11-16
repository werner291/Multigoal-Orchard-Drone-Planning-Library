//
// Created by werner on 4-10-22.
//

#ifndef NEW_PLANNERS_STUPIDALGORITHM_H
#define NEW_PLANNERS_STUPIDALGORITHM_H

#include "OnlinePointCloudMotionControlAlgorithm.h"

/**
 * A stupid algorithm that simply spins the robot around in place, then beelines to the first target point it sees.
 */
class StupidGoToFirstPointAlgorithm : public OnlinePointCloudMotionControlAlgorithm {

	/// Whether we've seen a target point yet
	bool firstPointFound = false;

public:
	/**
	 * Constructor.
	 *
	 * $param	trajectoryCallback		Callback to call when a new trajectory is available.
	 */
	explicit StupidGoToFirstPointAlgorithm(std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback);

	/**
	 * Called when a new point cloud is available.
	 *
	 * @param current_state 			The current state of the robot.
	 * @param segmentedPointCloud 		The segmented point cloud.
	 */
	void update(const moveit::core::RobotState& current_state, const SegmentedPointCloud &segmentedPointCloud) override;

};

#endif //NEW_PLANNERS_STUPIDALGORITHM_H
