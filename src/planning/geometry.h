// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/7/24.
//

#ifndef MGODPL_GEOMETRY_H
#define MGODPL_GEOMETRY_H

#include <array>
#include "../math/Vec3.h"

namespace mgodpl {
	/**
	 * \brief A triangle in 3D space, with vertices in Cartesian coordinates.
	 */
	struct Triangle {
		std::array<mgodpl::math::Vec3d, 3> vertices;

		[[nodiscard]] mgodpl::math::Vec3d normal() const {
			return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
		}
	};

	struct Edge {
		std::array<mgodpl::math::Vec3d, 2> vertices;
	};
}

#endif //MGODPL_GEOMETRY_H
