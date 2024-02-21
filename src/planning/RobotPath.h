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

        void append(const RobotState& state)
        {
            states.push_back(state);
        }

        void append(const RobotPath& path)
        {
            states.insert(states.end(), path.states.begin(), path.states.end());
        }
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

    /// Define a function pointer type for distance calculation functions
    using DistanceFn = double (*)(const RobotState&, const RobotState&);

    /**
     * @brief Calculate the length of a segment in a robot path.
     *
     * This function calculates the length of the segment at the given point on the path.
     * The length is calculated using the provided distance function.
     *
     * @param robot_path The robot path.
     * @param path_point The point on the path.
     * @param distanceFunc The function to use for calculating the distance between two states.
     * @return The length of the segment.
     */
    double calculateSegmentLength(const mgodpl::RobotPath& robot_path, const mgodpl::PathPoint& path_point,
                                  DistanceFn distanceFunc);

    /**
     * @brief Clamp a path point to the end of a robot path.
     *
     * This function modifies a path point so that it does not exceed the end of the robot path.
     * If the path point is at or beyond the end of the path, it is set to the last state of the path.
     *
     * @param robot_path The robot path.
     * @param path_point The path point to clamp (this is an output parameter).
     *
     * @return  True if the path point was clamped, and false otherwise.
     */
    bool clampPathPoint(const mgodpl::RobotPath& robot_path, mgodpl::PathPoint& path_point);

    /**
     * @brief Wrap a path point to the start of a robot path.
     *
     * This function modifies a path point so that it wraps around to the start of the robot path
     * when it reaches the end of the path.
     *
     * @param robot_path The robot path.
     * @param path_point The path point to wrap (this is an output parameter).
     *
     * @return True if the path point was wrapped, and false otherwise.
     */
    bool wrapPathPoint(const mgodpl::RobotPath& robot_path, mgodpl::PathPoint& path_point);

    /**
     * @brief Advance a path point along a robot path, clamping at the end.
     *
     * This function advances a path point along a robot path by a given amount.
     * The path point is clamped to the end of the path if it would otherwise exceed it.
     * The function returns true if the path point was clamped, and false otherwise.
     *
     * @param robot_path The robot path.
     * @param path_point The path point to advance.
     * @param advancement The amount to advance the path point.
     * @param distanceFunc The function to use for calculating the distance between two states.
     * @return True if the path point was clamped, false otherwise.
     */
    bool advancePathPointClamp(const mgodpl::RobotPath& robot_path, mgodpl::PathPoint& path_point, double advancement,
                               DistanceFn distanceFunc);

    /**
     * @brief Advance a path point along a robot path, wrapping at the end.
     *
     * This function advances a path point along a robot path by a given amount.
     * The path point wraps around to the start of the path if it reaches the end of the path.
     * The function returns true if the path point was wrapped, and false otherwise.
     *
     * @param robot_path The robot path.
     * @param path_point The path point to advance.
     * @param advancement The amount to advance the path point.
     * @param distanceFunc The function to use for calculating the distance between two states.
     * @return True if the path point was wrapped, false otherwise.
     */
    bool advancePathPointWrap(const mgodpl::RobotPath& robot_path, mgodpl::PathPoint& path_point, double advancement,
                              DistanceFn distanceFunc);

    /**
     * @brief Concatenates two RobotPaths.
     *
     * This function takes two RobotPath objects as input and returns a new RobotPath that is the concatenation of the two input paths.
     * The states of the second path are appended to the states of the first path in the resulting RobotPath.
     *
     * @param path1 The first RobotPath.
     * @param path2 The second RobotPath.
     * @return A new RobotPath that is the concatenation of path1 and path2.
     */
    RobotPath concatenate(const RobotPath& path1, const RobotPath& path2);

    /**
     * @brief Reverses a RobotPath.
     *
     * This function takes a RobotPath object as input and returns a new RobotPath that is the reverse of the input path.
     * The states in the resulting RobotPath are in the reverse order of the states in the input path.
     *
     * @param path The RobotPath to reverse.
     * @return A new RobotPath that is the reverse of the input path.
     */
    RobotPath reverse(const RobotPath& path);
}


#endif //MGODPL_ROBOTPATH_H
