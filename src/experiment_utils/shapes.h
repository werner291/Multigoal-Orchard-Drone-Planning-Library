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
#include "../planning/Mesh.h"

namespace mgodpl {

	struct Box {
		math::Vec3d size;
	};

	enum class ShapeType {
		BOX = 0,
		MESH = 1,
	};

	using Shape = std::variant<Box, Mesh>;

	Mesh loadMesh(const std::string& path);

}

#endif //MGODPL_SHAPES_H
