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
		[[nodiscard]] inline const Vec3d& direction() const {
			return _direction;
		}

		/**
		 * Compute the point on the line closest to the given point.
		 *
		 * @param p 	The point to compute the closest point to.
		 * @return 		The point on the line closest to p.
		 */
		[[nodiscard]] inline const Vec3d& origin() const {
			return _origin;
		}

		Vec3d pointAt(double d) const;

		static ParametricLine through_points(const Vec3d &vec3, Vec3d vec31);
	};
}

#endif //MGODPL_PARAMETRICLINE_H
