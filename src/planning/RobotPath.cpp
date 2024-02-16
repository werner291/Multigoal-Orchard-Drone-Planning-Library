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

double mgodpl::calculateSegmentLength(const mgodpl::RobotPath& robot_path, const mgodpl::PathPoint& path_point,
                                      DistanceFn distanceFunc)
{
    const auto& start_state = robot_path.states[path_point.segment_i];
    const auto& end_state = robot_path.states[path_point.segment_i + 1];
    return distanceFunc(start_state, end_state);
}

bool mgodpl::clampPathPoint(const mgodpl::RobotPath& robot_path, mgodpl::PathPoint& path_point)
{
    if (path_point.segment_i >= robot_path.states.size())
    {
        path_point.segment_i = robot_path.states.size() - 1;
        path_point.segment_t = 1.0;
        return true;
    }
    return false;
}

bool mgodpl::wrapPathPoint(const mgodpl::RobotPath& robot_path, mgodpl::PathPoint& path_point)
{
    if (path_point.segment_i >= robot_path.states.size())
    {
        path_point.segment_i = 0;
        return true;
    }
    return false;
}

void advance_naive(mgodpl::PathPoint& path_point, double advancement, double segment_length)
{
    path_point.segment_t += advancement / segment_length;

    if (path_point.segment_t > 1.0)
    {
        path_point.segment_i++;
        path_point.segment_t = 0.0;
    }
}

bool mgodpl::advancePathPointClamp(const mgodpl::RobotPath& robot_path, mgodpl::PathPoint& path_point,
                                   double advancement, DistanceFn distanceFunc)
{
    double segment_length = calculateSegmentLength(robot_path, path_point, distanceFunc);
    advance_naive(path_point, advancement, segment_length);
    return clampPathPoint(robot_path, path_point);
}

bool mgodpl::advancePathPointWrap(const mgodpl::RobotPath& robot_path, mgodpl::PathPoint& path_point,
                                  double advancement, DistanceFn distanceFunc)
{
    double segment_length = calculateSegmentLength(robot_path, path_point, distanceFunc);
    advance_naive(path_point, advancement, segment_length);
    return wrapPathPoint(robot_path, path_point);
}
