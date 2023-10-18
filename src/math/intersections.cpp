// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#include "intersections.h"

bool mgodpl::math::intersects(const mgodpl::math::AABBd &box, const mgodpl::math::Segment3d &segment) {

	auto intersections = line_aabb_intersection_params(box, segment.extend_to_line());

	if (!intersections) {
		return false;
	} else {
		return (*intersections)[0] <= 1.0 // The segment starts before exiting the box
			   && (*intersections)[1] >= 0.0; // The segment ends after entering the box
	}
}

std::optional<std::array<double, 2>>
mgodpl::math::line_aabb_intersection_params(const mgodpl::math::AABBd &box, const mgodpl::math::ParametricLine &segment) {

	double x_entry = param_at_plane(segment, 0, box.min().x());
	double x_exit = param_at_plane(segment, 0, box.max().x());
	if (x_entry > x_exit) {
		std::swap(x_entry, x_exit);
	}

	double y_entry = param_at_plane(segment, 1, box.min().y());
	double y_exit = param_at_plane(segment, 1, box.max().y());
	if (y_entry > y_exit) {
		std::swap(y_entry, y_exit);
	}

	double z_entry = param_at_plane(segment, 2, box.min().z());
	double z_exit = param_at_plane(segment, 2, box.max().z());
	if (z_entry > z_exit) {
		std::swap(z_entry, z_exit);
	}

	double entry = std::max({x_entry, y_entry, z_entry});
	double exit = std::min({x_exit, y_exit, z_exit});

	if (entry > exit) {
		return std::nullopt;
	} else {
		return std::array<double, 2>{entry, exit};
	}
}

double mgodpl::math::param_at_plane(const mgodpl::math::ParametricLine &p, int d, double value) {

	// Extract the direction component of the line in the given dimension.
	double dir_dim = p.direction()[d];

	// If the direction component is zero, the line is parallel to the plane.
	if (dir_dim == 0.0) {
		return NAN;
	} else {

		// Otherwise, compute the delta between the value and the origin in the given dimension.
		double delta = value - p.origin()[d];

		// Return the parameter at the plane.
		return delta / dir_dim;
	}
}

std::optional<double> mgodpl::math::param_at_plane(const mgodpl::math::Ray &r, int d, double value) {

	double t = param_at_plane(r.parametric_line(), d, value);

	if (t >= 0.0) {
		return t;
	} else {
		return std::nullopt;
	}
}
