// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/20/23.
//

#ifndef MGODPL_AABB_OF_H
#define MGODPL_AABB_OF_H

#include "AABB.h"
#include "Triangle.h"

namespace mgodpl::math {

	AABBd aabb_of(const Triangle &triangle);

}

#endif //MGODPL_AABB_OF_H
