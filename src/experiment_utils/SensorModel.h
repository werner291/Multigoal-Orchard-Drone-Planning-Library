// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12/31/24.
//

#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include <cmath>
#include <json/value.h>
#include "../planning/scannable_points.h"

namespace mgodpl::experiments {
	/**
	 * A sensor model `SensorModelParameters` is defined by a tuple of real-valued parameters `(min_distance, max_distance, fov_angle, surface_max_angle)`, where:
	 *
	 * Given sensor model parameters `SensorModelParameters` and inputs:
	 *
	 * - `(eye_pos, eye_forward)`: Sensor position and direction, derived from a configuration by forward kinematics.
	 * - `(point.position, point.normal)`: Surface point and normal vector.
	 *
	 * TODO: This is redundant with ScalarModelParameters, consider merging them.
	 */
	struct SensorModelParameters {
		double min_distance = 0.0;
		double max_distance = INFINITY;
		double surface_max_angle = M_PI / 2.0;
		double fov_angle = M_PI / 2.0;

		/**
		 * Checks if a given surface point is visible from a specified eye position and direction.
		 *
		 * @param eye_pos The position of the eye (sensor).
		 * @param eye_forward The forward direction vector of the eye (sensor).
		 * @param point The surface point to check visibility for.
		 * @return True if the point is visible, false otherwise.
		 */
		[[nodiscard]] inline bool is_visible(
			const math::Vec3d &eye_pos,
			const math::Vec3d &eye_forward,
			const SurfacePoint &point) const {
			math::Vec3d delta = eye_pos - point.position;
			double distance = delta.norm();

			return distance <= max_distance &&
			       distance >= min_distance &&
			       std::acos(point.normal.dot(delta) / distance) <= surface_max_angle &&
			       std::acos(eye_forward.dot(-delta) / distance) <= fov_angle;
		}

		[[nodiscard]] Json::Value toJson() const {
			Json::Value json;
			json["min_distance"] = min_distance;
			json["max_distance"] = max_distance;
			json["surface_max_angle"] = surface_max_angle;
			json["fov_angle"] = fov_angle;
			return json;
		}
	};
}

#endif //SENSORMODEL_H
