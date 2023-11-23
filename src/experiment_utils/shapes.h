// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#ifndef MGODPL_SHAPES_H
#define MGODPL_SHAPES_H


#include <vector>
#include <variant>
#include "../math/Vec3.h"

namespace mgodpl {

	struct Box {
		math::Vec3d size;
	};

	struct Mesh {
		std::vector<math::Vec3d> vertices;
		std::vector<std::array<size_t, 3>> triangles;
	};

	using Shape = std::variant<Box, Mesh>;

	Mesh loadMesh(const std::string& path);

}

#endif //MGODPL_SHAPES_H
