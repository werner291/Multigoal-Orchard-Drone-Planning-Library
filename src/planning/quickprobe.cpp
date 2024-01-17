// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/3/24.
//

#include "quickprobe.h"

namespace mgodpl {

	QuickProbe::QuickProbe(const std::vector<Triangle> &triangles,
						   const mgodpl::math::Vec3d &center,
						   double arm_radius,
						   double probe_margin) {

	}

	std::array<VerticalSpherePatch, 2> mgodpl::occupied_by_triangle(const mgodpl::Triangle &triangle,
																	const mgodpl::math::Vec3d &center,
																	double arm_radius) {

		const auto& [a,b,c] = sorted_relative_vertices(triangle, center);

		double triangle_distance_from_center =
				std::min((triangle.vertices[0] - center).norm(),
						 std::min((triangle.vertices[1] - center).norm(),
						  (triangle.vertices[2] - center).norm()));

		double padding = angular_padding(arm_radius, triangle_distance_from_center);

		// The two patches that we will return.
		return {
				VerticalSpherePatch {
					.longitude_range = {a.longitude - padding, b.longitude + padding},
					.left_edge = {a.latitude - padding, a.latitude + padding},
					.right_edge = {b.latitude - padding, b.latitude + padding}
				},
				VerticalSpherePatch {
					.longitude_range = {b.longitude - padding, c.longitude + padding},
					.left_edge = {b.latitude - padding, b.latitude + padding},
					.right_edge = {c.latitude - padding, c.latitude + padding}
				},
		}
	}

}
