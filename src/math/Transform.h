// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#ifndef MGODPL_TRANSFORM_H
#define MGODPL_TRANSFORM_H


#include "Vec3.h"
#include "Quaternion.h"

namespace mgodpl::math {

	/**
	 * @brief A 3D transformation, consisting of a translation and a rotation.
	 */
	struct Transformd {
		Vec3d translation;
		Quaterniond orientation;

		static inline Transformd identity() {
			return {
					.translation = {0, 0, 0},
					.orientation = {0, 0, 0, 1}
			};
		}

		[[nodiscard]] Transformd then(const Transformd& other) const {
			return {
					.translation = translation + orientation.rotate(other.translation),
					.orientation = orientation * other.orientation
			};
		}

		static inline Transformd fromTranslation(const Vec3d& translation) {
			return {
					.translation = translation,
					.orientation = {0, 0, 0, 1}
			};
		}

		static inline Transformd fromRotation(const Quaterniond& rotation) {
			return {
					.translation = {0, 0, 0},
					.orientation = rotation
			};
		}

		inline Transformd inverse() const {
			Quaterniond inv_orientation = orientation.inverse();
			return {
					.translation = inv_orientation.rotate(-translation),
					.orientation = inv_orientation
			};
		}
	};

}


#endif //MGODPL_TRANSFORM_H
