// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/16/23.
//

#ifndef MGODPL_PLANE_H
#define MGODPL_PLANE_H

#include "Vec3.h"

namespace mgodpl::math {

	/**
	 * A plane in 3D space.
	 */
	class Plane {
		Vec3d _normal;
		double _d;

	public:
		/**
		 * Construct a plane from a point and a normal vector.
		 *
		 * @param point 		A point on the plane.
		 * @param normal 		The normal vector of the plane.
		 * @return 				The plane.
		 */
		static Plane from_point_and_normal(const Vec3d& point, const Vec3d& normal);

		/**
		 * Create a new plane from the equation ax + by + cz + d = 0.
		 * @param coefficients The coefficients a, b, c, d.
		 */
		explicit Plane(const std::array<double, 4>& coefficients);

		Plane (const Vec3d& normal, double d) : _normal(normal), _d(d) {}

		[[nodiscard]] double signed_distance(const Vec3d& point) const;

		[[nodiscard]] const Vec3d& normal() const {
			return _normal;
		}

		[[nodiscard]] double d() const {
			return _d;
		}
	};

}

#endif //MGODPL_PLANE_H
