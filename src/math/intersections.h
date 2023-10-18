// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_INTERSECTIONS_H
#define MGODPL_INTERSECTIONS_H

#include <optional>
#include "Segment3d.h"
#include "AABB.h"
#include "Ray.h"

namespace mgodpl::math {

	/**
	 * Given a parametric line and a dimension, compute the parameter at which the line intersects
	 * the plane perpendicular to the given dimension at the given value.
	 *
	 * For example, for d = 1, this will be when the line crosses the y = value plane.
	 *
	 * @param p		The parametric line.
	 * @param d 	The dimension of the plane to intersect with.
	 * @param value The value of the plane to intersect with.
	 * @return		The parameter at which the line intersects the plane.
	 */
	double param_at_plane(const ParametricLine &p, int d, double value);

	/**
	 * Givena ray and a dimension, compute the parameter at which the ray intersects.
	 *
	 * For example, for d = 1, this will be when the ray crosses the y = value plane.
	 *
	 * @param r		The ray.
	 * @param d 	The dimension of the plane to intersect with.
	 * @param value The value of the plane to intersect with.
	 * @return		The parameter at which the ray intersects the plane.
	 */
	std::optional<double> param_at_plane(const Ray &r, int d, double value);

	/**
	  *	@brief Determines whether the given line segment intersects the given AABB.
	  *
	  *	Precise definition: function returns True if, for a segment S and an AABB B,
	  *	there exists a point P on the segment on the boundary or inside the AABB.
	  *
	  *	@param box The AABB to test for find_intersection with the line segment.
	  *	@param segment The line segment to test for find_intersection with the AABB.
	  *	@return True if the line segment intersects the AABB, false otherwise.
	  */
	bool intersects(const AABBd &box, const Segment3d &segment);

	/**
 	 * @brief Computes the find_intersection parameters of the given line segment with the given AABB.
     * If the line segment intersects the AABB, returns the parameters at which the line intersects
     * the minimum and maximum corner of the AABB. If the line segment does not intersect the AABB,
     * returns an empty optional.
     * @param box The AABB to intersect with the line segment.
     * @param segment The line segment to intersect with the AABB.
     * @return The find_intersection parameters of the line segment with the AABB, or an empty optional if no find_intersection.
     */
	std::optional<std::array<double, 2>> line_aabb_intersection_params(const AABBd &box, const ParametricLine &segment);
}

#endif //MGODPL_INTERSECTIONS_H
