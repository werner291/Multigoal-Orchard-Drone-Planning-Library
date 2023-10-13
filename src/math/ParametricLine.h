// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_PARAMETRICLINE_H
#define MGODPL_PARAMETRICLINE_H

#include "Vec3.h"

namespace mgodpl::math {
	struct ParametricLine {
		Vec3d _origin; // A point on the line.
		Vec3d _direction; // The direction of the line (not necessarily normalized).

		ParametricLine(const Vec3d &origin, const Vec3d &direction);

		/**
		 * Compute the point on the line closest to the given point.
		 *
		 * @param p 	The point to compute the closest point to.
		 * @return 		The point on the line closest to p.
		 */
		[[nodiscard]] Vec3d closest_point(const Vec3d &p) const;

		/**
		 * Compute the point on the line closest to the given point.
		 *
		 * @param p 	The point to compute the closest point to.
		 * @return 		The point on the line closest to p.
		 */
		[[nodiscard]] const Vec3d& direction() const;

		/**
		 * Compute the point on the line closest to the given point.
		 *
		 * @param p 	The point to compute the closest point to.
		 * @return 		The point on the line closest to p.
		 */
		[[nodiscard]] const Vec3d& origin() const;
	};
}

#endif //MGODPL_PARAMETRICLINE_H
