// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_TRIANGLE_H
#define MGODPL_TRIANGLE_H

#include "Vec3.h"
#include "AABB.h" // TODO: I prefer moving this to some kinda "Compute AABB" header.

namespace mgodpl::math {
	struct Triangle {
		Vec3d a, b, c;

		Triangle(const Vec3d &a, const Vec3d &b, const Vec3d &c);

		[[nodiscard]] Vec3d normal() const;

		[[nodiscard]] double area() const;
	};
}

#endif //MGODPL_TRIANGLE_H
