// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#ifndef MGODPL_VECMATH_UTILS_H
#define MGODPL_VECMATH_UTILS_H

#include <Eigen/Core>

namespace mgodpl::math {

	/**
	 * Convert a Vector3 to the nearest unit vector.
	 *
	 * @tparam Scalar 		The scalar type.
	 * @param direction 	The direction to convert. (Nonzero)
	 * @return 				The nearest unit vector.
	 */
	template<typename Scalar>
	Eigen::Vector3<Scalar> toNearestUnit(const Eigen::Vector3<Scalar> &direction) {

		assert(direction.squaredNorm() > 0.0);

		if (std::abs(direction.x()) >= std::abs(direction.y()) && std::abs(direction.x()) >= std::abs(direction.z())) {
			return {direction.x() > 0 ? 1 : -1, 0, 0};
		} else if (std::abs(direction.y()) >= std::abs(direction.x()) &&
				   std::abs(direction.y()) >= std::abs(direction.z())) {
			return {0, direction.y() > 0 ? 1 : -1, 0};
		} else {
			return {0, 0, direction.z() > 0 ? 1 : -1};
		}
	}

}

#endif //MGODPL_VECMATH_UTILS_H
