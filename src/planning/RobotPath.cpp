// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 14-2-24.
//

#include "RobotPath.h"

mgodpl::RobotState mgodpl::interpolate(const mgodpl::PathPoint& path_point, const mgodpl::RobotPath& robot_path)
{
    // Check if the segment index is valid
    assert(path_point.segment_i < robot_path.states.size() - 1 && "Segment index is out of bounds");

    // Get the two states that correspond to the segment
    const RobotState& state1 = robot_path.states[path_point.segment_i];
    const RobotState& state2 = robot_path.states[path_point.segment_i + 1];

    // Interpolate between the two states
    return interpolate(state1, state2, path_point.segment_t);
}
