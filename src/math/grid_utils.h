// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#ifndef MGODPL_GRID_UTILS_H
#define MGODPL_GRID_UTILS_H

#include <vector>
#include <iostream>
#include <array>
#include "Vec3.h"

namespace mgodpl::grid_utils {

	/**
	* Given a view center and a grid size, return a vector of all coordinates in order of distance from the view center.
	*
	* The order of equidistant coordinates is undefined.
	*
	* Question: Is there a better way to do this? Can we do it without allocating a vector? (Some kinda sequence generator?)
	*
	* @param view_center	The view center.
	* @param sizes			The grid size.
	* @return			A vector of all coordinates in order of distance from the view center.
	*/
	std::vector<math::Vec3i> coordinates_in_order(const math::Vec3i &view_center, const math::Vec3i &sizes);

	/**
	* Given a coordinate, return an array of its six neighbors.
	*
	* @param matrix 	The coordinate.
	* @return 			An array of its six neighbors.
	*/
	std::array<math::Vec3i, 6> neighbors(const mgodpl::math::Vec3i& matrix);

}


#endif //MGODPL_GRID_UTILS_H
