// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <iostream>
#include "signed_distance_fields.h"
#include "math_utils.h"

namespace sdf {

	ViewPyramidSdf open_view_pyramid_local(const Eigen::Vector2d &plane_slopes) {

		Eigen::Vector3d top_left = Eigen::Vector3d(-plane_slopes.x(), 1.0, plane_slopes.y());
		Eigen::Vector3d top_right = Eigen::Vector3d(plane_slopes.x(), 1.0, plane_slopes.y());
		Eigen::Vector3d bottom_left = Eigen::Vector3d(-plane_slopes.x(), 1.0, -plane_slopes.y());
		Eigen::Vector3d bottom_right = Eigen::Vector3d(plane_slopes.x(), 1.0, -plane_slopes.y());

		PlaneSdf top_plane{Plane3d(top_left.cross(top_right).normalized(), 0.0)};
		PlaneSdf bottom_plane{Plane3d(bottom_right.cross(bottom_left).normalized(), 0.0)};
		PlaneSdf left_plane{Plane3d(bottom_left.cross(top_left).normalized(), 0.0)};
		PlaneSdf right_plane{Plane3d(top_right.cross(bottom_right).normalized(), 0.0)};

		PlaneSdf forwardSdf{Plane3d(Eigen::Vector3d::UnitY(), 0.0)};

		return ((top_plane + bottom_plane) + (left_plane + right_plane)) + forwardSdf;

	}

	SdfValue RaySdf::operator()(const Eigen::Vector3d &p) const {
		Eigen::Vector3d closest_point = ray.closest_point(p);

		return {(p - closest_point).norm(), UVector3d((p - closest_point).normalized())};
	}

	SdfValue PlaneSdf::operator()(const Eigen::Vector3d &p) const {
		return {plane.signedDistance(p), UVector3d(plane.normal())};
	}
}