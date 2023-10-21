// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/20/23.
//

#include "aabb_of.h"

namespace mgodpl::math {

	AABBd aabb_of(const Triangle &triangle) {
		AABBd box = AABBd::inverted_infinity();

		box.expand(triangle.a);
		box.expand(triangle.b);
		box.expand(triangle.c);

		return box;
	}

}