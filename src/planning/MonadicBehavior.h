// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/6/23.
//

#ifndef MGODPL_MONADICBEHAVIOR_H
#define MGODPL_MONADICBEHAVIOR_H

#include <variant>
#include "JointSpacePoint.h"

namespace mgodpl::planning {

	/**
	 * A class that represents a behavior that can be executed by the robot; it can be monadically composed.
	 *
	 * @tparam S 		The state type, or the information the robot may use to decide what to do next.
	 * @tparam R 		The result type, letting the parent behavior know about the "conclusion" of the behavior.
	 */
	template<typename S, typename R>
	class MonadicBehavior {


		std::variant<moveit_facade::JointSpacePoint, R> nextMovementOrResult(const S&);

	};

}


#endif //MGODPL_MONADICBEHAVIOR_H
