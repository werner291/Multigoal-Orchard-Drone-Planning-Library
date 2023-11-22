// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/22/23.
//

#ifndef MGODPL_FCL_FORWARD_DECLARATIONS_H
#define MGODPL_FCL_FORWARD_DECLARATIONS_H

namespace fcl {
	template<typename S>
	class BVHModel;

	template<typename S>
	struct OBB;

	using OBBd = OBB<double>;

	template<typename S>
	class CollisionObject;

	using CollisionObjectd = CollisionObject<double>;
}

#endif //MGODPL_FCL_FORWARD_DECLARATIONS_H
