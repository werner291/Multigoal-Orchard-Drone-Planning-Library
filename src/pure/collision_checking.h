// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file collision_checking.h
 *
 * @brief This file contains traits and functions for collision checking.
 */

#ifndef MGODPL_COLLISION_CHECKING_H
#define MGODPL_COLLISION_CHECKING_H

namespace mgodpl::collision_checking {

	/**
	 * @brief Check whether a configuration is in collision.
	 */
	template<typename Configuration, typename CollisionEnvironment>
	bool collides(const Configuration& configuration, const CollisionEnvironment& collision_environment);

	/**
	 * @brief Check whether a movement from one configuration to another is collision-free.
	 */
	template<typename Configuration, typename CollisionEnvironment>
	bool collides(const Configuration& configuration1,
				  const Configuration& configuration2,
				  const CollisionEnvironment& collision_environment);

}

#endif //MGODPL_COLLISION_CHECKING_H
