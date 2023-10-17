// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_TRIANGLE_H
#define MGODPL_TRIANGLE_H

#include "Vec3.h"

namespace mgodpl::math {
	struct Triangle {
		Vec3d a, b, c;

		AABBd aabb() const {
			AABBd box = AABBd::inverted_infinity();

			box.expand(a);
			box.expand(b);
			box.expand(c);

			return box;
		}

		[[nodiscard]] double area() const {
			return 0.5 * (b - a).cross(c - a).norm();
		}
	};
}

#endif //MGODPL_TRIANGLE_H
