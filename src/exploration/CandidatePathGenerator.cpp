//
// Created by werner on 21-11-22.
//

#include "CandidatePathGenerator.h"
#include "../utilities/moveit.h"

#include <utility>

CandidatePathGenerator::CandidatePathGenerator(const moveit::core::RobotState &start_state,
											   Eigen::Vector3d target_point)
		: start_state(start_state), target_point(std::move(target_point)) {
}

RobotPath CandidatePathGenerator::generateCandidatePath() {

	return RobotPath {{start_state, setEndEffectorToPosition(start_state, target_point)}
	};

}

