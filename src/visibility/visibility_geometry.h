// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/23/23.
//

#ifndef MGODPL_VISIBILITY_GEOMETRY_H
#define MGODPL_VISIBILITY_GEOMETRY_H

#include <optional>
#include <vector>
#include "../math/Vec3.h"
#include "../math/AABB.h"
#include "../math/Triangle.h"
#include "../math/Ray.h"
#include "../math/RangeInclusive.h"

namespace mgodpl::visibility {

	/**
	 * Given an eye position and a point, compute the ray occluded by that point, plus an optional offset.
	 *
	 * @param eye 		The eye position.
	 * @param occluding_point 	The point.
	 * @param offset	How far behind the point to put the 0-point of the line.
	 *
	 * @return 			The ray.
	 */
	math::Ray occluded_ray(const mgodpl::math::Vec3d &eye, const mgodpl::math::Vec3d &occluding_point, double offset = 0.0);

	/**
	 * An axis-aligned slab, representing the space between two parallel axis-aligned planes.
	 */
	template<typename Scalar>
	struct AASlab {
		int dimension; /// Whether it's the X, Y or Z axis that this is perpendicular to.
		math::RangeInclusive<Scalar> range; /// The coordinate range on that axis covered by the slab.
	};

	/**
	 * Given an AABB and a set of rays, compute the AABB of the intersection of
	 * the convex hull of the rays and the volume delimited by the AABB.
	 *
	 * @param aabb 		The AABB.
	 * @param rays 		The rays.
	 *
	 * @return 			The AABB of the intersection of the convex hull of the rays and the volume delimited by the AABB, or nullopt if the convex hull does not intersect the AABB.
	 */
	std::optional<math::AABBd>
	aabbInAABB(const math::AABBd &aabb, const std::array<mgodpl::math::Ray, 3> &rays);

	std::vector<std::pair<double, const math::Triangle *>> sorted_by_distance(
			const std::vector<mgodpl::math::Triangle> &triangles,
			const mgodpl::math::Vec3d &eye);
}

#endif //MGODPL_VISIBILITY_GEOMETRY_H
