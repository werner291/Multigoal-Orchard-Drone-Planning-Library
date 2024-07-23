// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#ifndef MGODPL_QUATERNION_H
#define MGODPL_QUATERNION_H

#include "Vec3.h"

#include <algorithm>

namespace mgodpl::math {
	struct AxisAngled {
		Vec3d axis;
		double angle;
	};

	/**
	 * @brief A quaternion in the form of (x, y, z, w), interpreted as a rotation.
	 */
	struct Quaterniond {
		double x, y, z, w;

		inline Quaterniond conjugate() const {
			return {-x, -y, -z, w};
		}

		inline Quaterniond inverse() const {
			return conjugate();
		}

		inline Quaterniond operator*(const Quaterniond &other) const {
			return {
				.x = w * other.x + x * other.w + y * other.z - z * other.y,
				// TODO: Copilot wrote this, check if correct.
				.y = w * other.y - x * other.z + y * other.w + z * other.x,
				.z = w * other.z + x * other.y - y * other.x + z * other.w,
				.w = w * other.w - x * other.x - y * other.y - z * other.z
			};
		}

		[[nodiscard]] inline Vec3d rotate(const Vec3d &v) const {
			Quaterniond qv{v.x(), v.y(), v.z(), 0};
			Quaterniond qv_rotated = *this * qv * conjugate();

			return {qv_rotated.x, qv_rotated.y, qv_rotated.z};
		}

		static inline Quaterniond fromAxisAngle(const Vec3d &axis, double angle) {
			double half_angle = angle / 2.0;
			double sin_half_angle = std::sin(half_angle);
			return {
				.x = axis.x() * sin_half_angle,
				.y = axis.y() * sin_half_angle,
				.z = axis.z() * sin_half_angle,
				.w = std::cos(half_angle)
			};
		}

		[[nodiscard]] AxisAngled toAxisAngle() const {
			double angle = 2.0 * std::acos(w);
			double sin_half_angle = std::sin(angle / 2.0);
			return {
				.axis = {x / sin_half_angle, y / sin_half_angle, z / sin_half_angle},
				.angle = angle
			};
		}

		[[nodiscard]] inline bool operator==(const Quaterniond &other) const {
			return x == other.x && y == other.y && z == other.z && w == other.w;
		}
	};

	inline double angular_distance(const Quaterniond &a, const Quaterniond &b) {
		double cos = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

		// Small numerical errors can cause cos to be slightly outside [-1, 1]; clamp to prevent NaN.
		cos = std::clamp(cos, -1.0, 1.0);

		return acos(2.0 * cos * cos - 1.0);
	}

	Quaterniond slerp(const Quaterniond &a, const Quaterniond &b, double t);
}

#endif //MGODPL_QUATERNION_H
