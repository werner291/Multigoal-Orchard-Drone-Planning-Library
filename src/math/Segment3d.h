// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_SEGMENT3D_H
#define MGODPL_SEGMENT3D_H

#include "Vec3.h"
#include "ParametricLine.h"

namespace mgodpl::math {
	struct Segment3d {
		Vec3d a;
		Vec3d b;

		Segment3d(const Vec3d &a, const Vec3d &b);

		/**
		 * Compute the point on the line closest to the given point.
		 *
		 * @param p 	The point to compute the closest point to.
		 * @return 		The point on the line closest to p.
		 */
		[[nodiscard]] Vec3d closest_point(const Vec3d &p) const;

		/**
		 * Extend it into a line.
		 */
		[[nodiscard]] ParametricLine extend_to_line() const;
	};
}

#endif //MGODPL_SEGMENT3D_H
