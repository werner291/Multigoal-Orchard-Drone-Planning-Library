// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/6/23.
//

#ifndef MGODPL_JOINTSPACEPATH_H
#define MGODPL_JOINTSPACEPATH_H

#include "JointSpacePoint.h"
#include "moveit_forward_declarations.h"
#include <optional>

namespace mgodpl::moveit_facade {
	struct JointSpacePath {
		std::vector<moveit_facade::JointSpacePoint> path;
	};

	/**
	 * Check whether the given point p is on the motion between the two given points
	 * a,b by checking the distances of the edges_padded of the triangle (a,b,p).
	 *
	 * To qualify, |ap| + |pb| = |ab| must hold.
	 *
	 * @param robot 		The robot model (to compute joint distances).
	 * @param motion 		The two points (a,b) between which the point is checked.
	 * @param point 		The point p to check.
	 * @param threshold 	The threshold for the distance check.
	 * @return 				True if the point is on the motion, false otherwise.
	 */
	bool stateIsOnMotion(const moveit::core::RobotModel& robot, const std::array<JointSpacePoint, 2>& motion, const JointSpacePoint& point, const double threshold = 1.e-6);

	/**
	 * @brief 			Given a JointSpacePath, find the first motion that a given point is on.
	 *
	 * @param robot 	The robot model (to compute joint distances).
	 * @param path		The path to search.
	 * @param point		The point to search for.
	 *
	 * @return 			The index of the first motion that the point is on, or nullopt if no such motion exists.
	 */
	std::optional<size_t> findMotionOnPath(const moveit::core::RobotModel& robot, const JointSpacePath& path, const JointSpacePoint& point);


}

#endif //MGODPL_JOINTSPACEPATH_H
