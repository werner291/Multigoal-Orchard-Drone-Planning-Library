// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/9/24.
//

#ifndef MGODPL_LOCAL_OPTIMIZATION_H
#define MGODPL_LOCAL_OPTIMIZATION_H

#include <fcl/geometry/bvh/BVH_model.h>

#include "RobotModel.h"
#include "RobotPath.h"
#include "collision_detection.h"
#include "RandomNumberGenerator.h"

namespace mgodpl {

	/**
	 * Given a RobotPath and a waypoint index, try to shortcut the path by deleting the waypoint at the given index.
	 *
	 * This succeeds if this does not introduce a new collision; if it fails the path is unmodified.
	 *
	 * @param robot 		The robot model.
	 * @param path 			The path to shorten; modified in place if successful. Unmodified otherwise.
	 * @param waypointIndex The index of the waypoint to delete.
	 * @param obstacle 		The BVH model of the tree trunk
	 *
	 * @return True if the path was successfully shortened, false otherwise.
	 */
	bool tryShortcutByDeletingWaypoint(const mgodpl::robot_model::RobotModel& robot,
									   RobotPath& path,
									   size_t waypointIndex,
									   const fcl::CollisionObjectd& obstacle);

	/**
	 * Given a RobotPath, try to delete every waypoint in the path that can be deleted without introducing a collision.
	 *
	 * @param robot 		The robot model.
	 * @param path 			The path to shorten; modified in place if successful. Unmodified otherwise.
	 * @param obstacle 		The BVH model of the tree trunk.
	 * @return 				True if the path was successfully shortened at any point, false otherwise.
	 */
	bool tryDeletingEveryWaypoint(const mgodpl::robot_model::RobotModel& robot,
								  RobotPath& path,
								  const fcl::CollisionObjectd& obstacle);

	/**
	 * Given a RobotPath and two waypoints, try to shortcut the path by connecting two index/point pairs with a straight line.
	 *
	 * @param robot 		The robot model.
	 * @param path 			The path to shorten; modified in place if successful. Unmodified otherwise.
	 * @param start 		The index and point of the start of the shortcut.
	 * @param end 			The index and point of the end of the shortcut.
	 * @param obstacle 		The BVH model of the tree trunk.
	 * @return 				True if the path was successfully shortened, false otherwise.
	 */
	bool tryShortcutBetweenPathPoints(const mgodpl::robot_model::RobotModel& robot,
									  RobotPath& path,
									  const PathPoint& start,
									  const PathPoint& end,
									  const fcl::CollisionObjectd &obstacle);

	PathPoint generateRandomPathPoint(const RobotPath& path, random_numbers::RandomNumberGenerator& rng) {
		size_t segment_i = rng.uniformInteger(0, path.states.size() - 2);
		double segment_t = rng.uniformReal(0.0, 1.0);
		return PathPoint{segment_i, segment_t};
	}

	/**
	 * Give a RobotPath, repeatedly try to generate two nearby path points and shortcut between them.
	 *
	 * @param robot 		 The robot model.
	 * @param path 			 The path to shorten; modified in place if successful. Unmodified otherwise.
	 * @param obstacle 		 The BVH model of the tree trunk.
	 * @param rng 			 The random number generator to use.
	 * @param max_iterations The maximum number of iterations to try.
	 * @return 				 True if the path was successfully shortened at any point, false otherwise.
	 */
	bool tryShortcuttingRandomly(const mgodpl::robot_model::RobotModel& robot,
								 RobotPath& path,
								 const fcl::CollisionObjectd& obstacle,
								 random_numbers::RandomNumberGenerator& rng);

	/**
	 * Attempt a "midpoint pull" optimization of a given waypoint.
	 *
	 * That is: given a waypoint and its two neighbors, try to pull the waypoint towards the midpoint of the neighbors.
	 *
	 * @param robot 		The robot model.
	 * @param path 			The path to optimize.
	 * @param waypointIndex The index of the waypoint to optimize.
	 * @param pull_factor 	The factor by which to pull the waypoint towards the midpoint (0.0 = no pull, 1.0 = pull all the way).
	 * @param obstacle 		The BVH model of the tree trunk.
	 *
	 * @return True if the optimization was successful, false otherwise.
	 */
	bool tryMidpointPull(const mgodpl::robot_model::RobotModel& robot,
						 RobotPath& path,
						 size_t waypointIndex,
						 double pull_factor,
						 const fcl::CollisionObjectd& obstacle);

}

#endif //MGODPL_LOCAL_OPTIMIZATION_H
