// This source code is copyrighted by University College Roosevelt, 2022. All rights reserved.
// Author: Werner
// Date: 10/13/23

#ifndef MGODPL_AABB_H
#define MGODPL_AABB_H

#include <algorithm>
#include <optional>

#include "Vec3.h"

namespace mgodpl::math {

	/**
	 * @brief A 3D axis-aligned bounding box template.
	 *
	 * @tparam Scalar The type of the coordinates.
	 */
	template<typename Scalar>
	struct AABB {

		Vec3<Scalar> _min, _max = Vec3<Scalar>(0, 0, 0);

		/**
		 * @brief Constructor for the AABB based on minimum and maximum corners.
		 *
		 * @param min The minimum corner.
		 * @param max The maximum corner.
		 */
		AABB(const Vec3<Scalar> &min, const Vec3<Scalar> &max) : _min(min), _max(max) {

		}

		/**
		 * @brief Create an AABB from an array of points.
		 *
		 * @param points An array of Vec3 points.
		 *
		 * @return An AABB containing all the points.
		 */
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

		/**
		 * @brief Check if a point is contained within this AABB.
		 *
		 * @param pt The point to check.
		 *
		 * @return True if the point is inside the AABB, false otherwise.
		 */
		bool contains(const Vec3<Scalar> &pt) const {
			return pt.dominates(_min) && _max.dominates(pt);
		}

		bool contains(const AABB<Scalar> &aabb) const {
			return contains(aabb._min) && contains(aabb._max);
		}

		/**
		 * @brief Get the center of the AABB.
		 *
		 * @return The center of the AABB.
		 */
		Vec3<Scalar> center() const {
			return (_min + _max) / (Scalar) 2;
		}

		/**
		 * @brief Get the size of the AABB.
		 *
		 * @return The size of the AABB.
		 */
		Vec3<Scalar> size() const {
			return _max - _min;
		}

		/**
		 * @brief Get the minimum coordinate of the AABB.
		 *
		 * @return The minimum coordinate of the AABB.
		 */
		const Vec3<Scalar>& min() const {
			return _min;
		}

		/**
		 * @brief Get the maximum coordinate of the AABB.
		 *
		 * @return The maximum coordinate of the AABB.
		 */
		const Vec3<Scalar>& max() const {
			return _max;
		}

		/**
 * @brief Get the minimum coordinate of the AABB.
 *
 * @return The minimum coordinate of the AABB.
 */
		 Vec3<Scalar>& min()  {
			return _min;
		}

		/**
		 * @brief Get the maximum coordinate of the AABB.
		 *
		 * @return The maximum coordinate of the AABB.
		 */
		 Vec3<Scalar>& max()  {
			return _max;
		}



		/**
		 * @brief Check whether this AABB intersects with another AABB. This includes the case where the AABBs touch.
		 *
		 * @param other The other AABB to check for intersection.
		 *
		 * @return True if there is an intersection, false otherwise.
		 */
		bool intersects(const AABB<Scalar> &other) const {
			return _max.dominates(other._min) && other._max.dominates(_min);
		}

		/**
		 * @brief Expand the AABB to include a point.
		 *
		 * @param pt The point to include in the AABB.
		 */
		void expand(const Vec3<Scalar> &pt) {
			_min = _min.min(pt);
			_max = _max.max(pt);
		}

		/**
		 * @brief Create an "inverted infinite" AABB. The min is set to (INFINITY, INFINITY, INFINITY) and the max to (-INFINITY, -INFINITY, -INFINITY).
		 *
		 * This way, after the first point is added with expand(), the AABB will be set to tightly fit the point.
		 *
		 * @return An "inverted infinite" AABB.
		 */
		static AABB inverted_infinity() {
			return AABB(Vec3<Scalar>(INFINITY, INFINITY, INFINITY), Vec3<Scalar>(-INFINITY, -INFINITY, -INFINITY));
		}

		/**
		 * @brief Inflate the AABB by a given amount.
		 * @param d 		The amount to inflate the AABB by.
		 * @return 			The inflated AABB.
		 */
		AABB inflated(Scalar d) const {
			return AABB(_min - Vec3<Scalar>(d, d, d), _max + Vec3<Scalar>(d, d, d));
		}

		AABB translated(Vec3<Scalar> vec3) const {
			return {_min + vec3, _max + vec3};
		}

		AABB combined(AABB aabb) const {
			return AABB(_min.min(aabb._min), _max.max(aabb._max));
		}

		std::optional<AABB> intersection(const AABB &aabb) const {
			if (intersects(aabb)) {
				return AABB(_min.max(aabb._min), _max.min(aabb._max));
			} else {
				return std::nullopt;
			}
		}

		Scalar volume() const {
			return size().x() * size().y() * size().z();
		}
	};

	using AABBd = AABB<double>; // Double precision AABB
	using AABBi = AABB<int>;    // Integer precision AABB
}

#endif // MGODPL_AABB_H
