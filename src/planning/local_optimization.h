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
	 * Try to shorten a path by deleting the waypoint at the given index. This will result in moving directly from
	 * the waypoint before the index to the waypoint after the index.
	 *
	 * The waypoint to be deleted should not be the first or the last waypoint. (Checked by assertion in debug builds.)
	 *
	 * @param path						The path to shorten.
	 * @param waypointIndex				The index of the waypoint to delete.
	 * @param check_motion_collides		A function that checks if a motion between two states collides.
	 * @return							True if the path was successfully shortened, false otherwise.
	 */
	bool tryShortcutByDeletingWaypoint(
		RobotPath &path,
		size_t waypointIndex,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides
	);

	/**
	 * Given a RobotPath, try to delete every waypoint in the path that can be deleted without introducing a collision.
	 *
	 * @param robot 		The robot model.
	 * @param path 			The path to shorten; modified in place if successful. Unmodified otherwise.
	 * @param obstacle 		The BVH model of the tree trunk.
	 * @return 				True if the path was successfully shortened at any point, false otherwise.
	 */
	bool tryDeletingEveryWaypoint(const mgodpl::robot_model::RobotModel &robot,
	                              RobotPath &path,
	                              const fcl::CollisionObjectd &obstacle);

	/**
	 * Given a RobotPath and two waypoints, try to shortcut the path by connecting two index/point pairs with a straight line.
	 *
	 * @param path 			The path to shorten; modified in place if successful. Unmodified otherwise.
	 * @param start 		The index and point of the start of the shortcut.
	 * @param end 			The index and point of the end of the shortcut. (Should be after the start.)
	 * @param check_motion_collides A function that checks if a motion between two states collides.
	 *
	 * @return 				True if the path was successfully shortened, false otherwise.
	 */
	bool tryShortcutBetweenPathPoints(
		RobotPath &path,
		const PathPoint &start,
		const PathPoint &end,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides
	);

	/**
	 * Generate a random path point on the given path.
	 *
	 * @param path	The path to generate a random path point on.
	 * @param rng	The random number generator to use.
	 * @return	A random path point on the path.
	 */
	PathPoint generateRandomPathPoint(const RobotPath &path, random_numbers::RandomNumberGenerator &rng);

	/**
	 * Give a RobotPath, repeatedly try to generate two nearby path points and shortcut between them.
	 *
	 * @param path 			 The path to shorten; modified in place if successful. Unmodified otherwise.
	 * @param rng 			 The random number generator to use.
	 * @param check_motion_collides A function that checks if a motion between two states collides.
	 *
	 * @return 				 True if the path was successfully shortened at any point, false otherwise.
	 */
	bool tryShortcuttingRandomlyLocally(
		RobotPath &path,
		random_numbers::RandomNumberGenerator &rng,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides
	);

	/**
	 * Given a RobotPath, try to optimize the path by shortcutting between two random path points that are arbitrarily far apart.
	 *
	 * @param path 			The path to optimize; modified in place if successful. Unmodified otherwise.
	 * @param check_motion 	The motion to check for collisions; true if the motion is in collision, false otherwise.
	 * @param rng 			The random number generator to use.
	 *
	 * @return True if the path was successfully shortened, false otherwise.
	 */
	bool tryShortcuttingRandomlyGlobally(
		RobotPath &path,
		const std::function<bool(const RobotState &, const RobotState &)> &
		check_motion,
		random_numbers::RandomNumberGenerator &rng
	);

	/**
	 * Attempt a "midpoint pull" optimization of a given waypoint.
	 *
	 * That is: given a waypoint and its two neighbors, try to pull the waypoint towards the midpoint of the neighbors.
	 *
	 * @param path 			The path to optimize.
	 * @param waypointIndex The index of the waypoint to optimize.
	 * @param pull_factor 	The factor by which to pull the waypoint towards the midpoint (0.0 = no pull, 1.0 = pull all the way).
	 * @param check_motion_collides A function that checks if a motion between two states collides.
	 *
	 * @return True if the optimization was successful, false otherwise.
	 */
	bool tryMidpointPull(
		RobotPath &path,
		size_t waypointIndex,
		double pull_factor,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides
	);
}

#endif //MGODPL_LOCAL_OPTIMIZATION_H
