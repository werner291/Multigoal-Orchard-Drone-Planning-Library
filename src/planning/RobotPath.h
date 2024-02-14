// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/22/24.
//

#ifndef MGODPL_ROBOTPATH_H
#define MGODPL_ROBOTPATH_H


#include <vector>
#include "RobotState.h"

namespace mgodpl

{
    /**
     * @struct RobotPath
     * @brief A structure representing a path for the robot.
     *
     * This structure contains a vector of RobotState objects, each representing a state of the robot at a certain point along the path.
     */
    struct RobotPath
    {
        /// @brief A vector of RobotState objects representing the states of the robot along the path.
        std::vector<RobotState> states;
    };

    /**
     * @struct PathPoint
     * @brief A structure representing a point on a path.
     *
     * This structure is used to define a point on a path based on states and the interpolated motions between them.
     */
    struct PathPoint
    {
        /// @brief The index of the segment on the path. This is the segment between states segment_i and segment_i+1 (0-indexed).
        size_t segment_i;

        /// @brief The time at which the robot is at this point on the segment.
        double segment_t;
    };


    /**
     * @brief Interpolates between two states of a robot path at a given point.
     *
     * @param path_point The point on the path where the interpolation is to be done.
     * @param robot_path The path of the robot.
     * @return The interpolated RobotState at the given point on the path.
     */
    RobotState interpolate(const mgodpl::PathPoint& path_point, const mgodpl::RobotPath& robot_path);
}


#endif //MGODPL_ROBOTPATH_H
