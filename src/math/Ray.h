// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/18/23.
//

#ifndef MGODPL_RAY_H
#define MGODPL_RAY_H

#include "ParametricLine.h"

namespace mgodpl::math {
	/**
	 * A ray.
	 *
	 * It's just a ParametricLine restricted to the [0,inf) range.
	 */
	struct Ray {

		ParametricLine line; /// The ray is implemented as a parametric line restricted to the [0,inf) range.

		inline Ray(const Vec3d &origin, const Vec3d &direction) : line(origin, direction) {
		}

		/**
		 * Advance the origin point of the ray along the ray by a given parameter.
		 */
		[[nodiscard]] Ray advanced(double d) const;

		/**
		 * Get the parametric line of the ray.
		 */
		[[nodiscard]] const ParametricLine& parametric_line() const {
			return line;
		}

		[[nodiscard]] Vec3<double> pointAt(double aDouble) const;
	};
}

#endif //MGODPL_RAY_H
