// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8/28/24.
//

#ifndef PARAMETRICINFINITECONE_H
#define PARAMETRICINFINITECONE_H

#include "../math/Vec3.h"

namespace mgodpl {

	class ParametricInfiniteCone {

		// Three basis vectors:
		math::Vec3d axis, b, c;

		// The position of the apex:
		math::Vec3d apex;

	public:
		[[nodiscard]] math::Vec3d position(double t, double theta) const;

		/**
		 * Constructs a parametric infinite cone with the given axis and apex angle.
		 * Axis assumed to be normalized (checked with an assertion).
		 *
		 * Also, the axis is assumed to not be parallel to the z-axis.
		 */
		ParametricInfiniteCone(const math::Vec3d &axis,
							   double apex_angle,
							   const math::Vec3d &apex = math::Vec3d::Zero());
	};

} // namespace mgodpl


#endif // PARAMETRICINFINITECONE_H
