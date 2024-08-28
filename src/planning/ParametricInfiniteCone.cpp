// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8/28/24.
//

#include "ParametricInfiniteCone.h"
mgodpl::math::Vec3d mgodpl::ParametricInfiniteCone::position(double t, double theta) const {
	return (axis + b * cos(theta) + c * sin(theta)) * t + apex;
}
mgodpl::ParametricInfiniteCone::ParametricInfiniteCone(const math::Vec3d &axis,
													   double apex_angle,
													   const math::Vec3d &apex) : axis(axis), apex(apex) {
	// Ensure the apex angle is within the valid range (0, π/2)
	assert(apex_angle > 0.0 && apex_angle < M_PI / 2.0);

	// Ensure the axis vector is a unit vector
	assert(abs(axis.squaredNorm() - 1.0) < 1e-6);

	// Calculate the vector b as the cross product of the unit Z vector and the axis
	b = math::Vec3d::UnitZ().cross(axis);
	c = axis.cross(b); // Calculate the vector c as the cross product of the axis and vector b
	// Now, axis, b, and c form an orthonormal basis.

	// Ensure the vector b is a unit vector
	assert(abs(b.squaredNorm() - 1.0) < 1e-6);
	assert(abs(c.squaredNorm() - 1.0) < 1e-6);

	// Scale vectors b and c by the tangent of the apex angle, to induce the correct cone shape
	b *= tan(apex_angle);
	c *= tan(apex_angle);
}
