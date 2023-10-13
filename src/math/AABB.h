// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_AABB_H
#define MGODPL_AABB_H

#include <algorithm>
#include "Vec3.h"

namespace mgodpl::math {
	/**
	 * A 3D axis-aligned bounding box.
	 * @tparam Scalar The type of the coordinates.
	 */
	template<typename Scalar>
	struct AABB {

		Vec3<Scalar> _min, _max = Vec3<Scalar>(0, 0, 0);

		/**
		 * Constructor based on min and max corners.
		 * @param min The minimum corner.
		 * @param max The maximum corner.
		 */
		AABB(const Vec3<Scalar> &min, const Vec3<Scalar> &max) : _min(min), _max(max) {

		}

		template<size_t N>
		static AABB from_points(const std::array<Vec3<Scalar>, N> &points) {
			Vec3<Scalar> min { INFINITY, INFINITY, INFINITY };
			Vec3<Scalar> max { -INFINITY, -INFINITY, -INFINITY };
			for (const Vec3<Scalar> &pt : points) {
				min = min.min(pt);
				max = max.max(pt);
			}
			return AABB(min, max);
		}

		bool contains(const Vec3<Scalar> &pt) const {
			return pt.dominates(_min) && _max.dominates(pt);
		}

		/**
		 * Get the center of the AABB.
		 * @return The center of the AABB.
		 */
		Vec3<Scalar> center() const {
			return (_min + _max) / (Scalar) 2;
		}

		/**
		 * Get the size of the AABB.
		 * @return The size of the AABB.
		 */
		Vec3<Scalar> size() const {
			return _max - _min;
		}

		/**
		 * Get the minimum coordinate of the AABB.
		 * @return The minimum coordinate of the AABB.
		 */
		Vec3<Scalar> min() const {
			return _min;
		}

		/**
		 * Get the maximum coordinate of the AABB.
		 * @return The maximum coordinate of the AABB.
		 */
		Vec3<Scalar> max() const {
			return _max;
		}

		/**
		 * Check whether this AABB intersects with another AABB. This includes the case where the AABBs touch.
		 *
		 * @param other 	The other AABB.
		 */
		bool intersects(const AABB<Scalar> &other) const {
			return _max.dominates(other._min) && other._max.dominates(_min);
		}

		void expand(const Vec3<Scalar> &pt) {
			_min = _min.min(pt);
			_max = _max.max(pt);
		}

		/**
		 * Allocate an "inverted infinite" AABB, i.e.e and AABB that has the max set to (-INFINITY, -INFINITY, -INFINITY) and the min set to (INFINITY, INFINITY, INFINITY).
		 *
		 * This way, after the first point is added with expand(), the AABB will be set to tightly fit the point.
		 *
		 * @return 		An "inverted infinite" AABB.
		 */
		static AABB inverted_infinity() {
			return AABB(Vec3<Scalar>(INFINITY, INFINITY, INFINITY), Vec3<Scalar>(-INFINITY, -INFINITY, -INFINITY));
		}

	};

	using AABBd = AABB<double>;
	using AABBi = AABB<int>;
}

#endif //MGODPL_AABB_H
