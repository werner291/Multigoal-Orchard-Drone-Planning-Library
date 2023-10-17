// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/16/23.
//

#ifndef MGODPL_OPENPYRAMID_H
#define MGODPL_OPENPYRAMID_H

#include "Vec3.h"
#include "HalfSpace.h"

namespace mgodpl::math {




	/**
	 * A pyramid with an open base, defined by a point and the vertices of the base polygon.
	 *
	 * @tparam nBase 		The number of points in the base.
	 */
	template<unsigned int nBase>
	class OpenPyramid {

		Vec3d _apex;
		std::array<Vec3d, nBase> _base;

public:
		/**
		 * Check if a given point lies in the interior of the pyramid.
		 *
		 * @param point	The point.
		 * @return 		True if the point lies in the interior of the pyramid.
		 */
		[[nodiscard]] bool inside(const Vec3d& point) const {

			std::array<Plane, nBase> planes {
					Plane::from_point_and_normal(_base[0], (_base[1] - _base[0]).cross(_apex - _base[0])),
					Plane::from_point_and_normal(_base[1], (_base[2] - _base[1]).cross(_apex - _base[1])),
					Plane::from_point_and_normal(_base[2], (_base[3] - _base[2]).cross(_apex - _base[2]))
			};

			for (const auto& plane : planes) {
				if (plane.signed_distance(point) > 0) {
					return false;
				}
			}

		}

	};

}


#endif //MGODPL_OPENPYRAMID_H
