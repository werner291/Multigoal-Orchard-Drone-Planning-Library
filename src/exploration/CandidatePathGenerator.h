//
// Created by werner on 21-11-22.
//

#ifndef NEW_PLANNERS_CANDIDATEPATHGENERATOR_H
#define NEW_PLANNERS_CANDIDATEPATHGENERATOR_H

#include <moveit/robot_state/robot_state.h>
#include "SegmentedPointCloud.h"
#include "../RobotPath.h"

class CandidatePathGenerator {

	moveit::core::RobotState start_state;

	SegmentedPointCloud::TargetPoint target_point;

public:

	CandidatePathGenerator(const moveit::core::RobotState &start_state,
	                       SegmentedPointCloud::TargetPoint target_point);

	/**
	 * Generate a candidate path.
	 *
	 * @return A candidate path.
	 */
	[[nodiscard]] RobotPath generateCandidatePath();

};

#endif //NEW_PLANNERS_CANDIDATEPATHGENERATOR_H
