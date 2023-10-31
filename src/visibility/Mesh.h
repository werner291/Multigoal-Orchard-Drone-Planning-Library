// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/31/23.
//

#ifndef MGODPL_MESH_H
#define MGODPL_MESH_H

#include <vector>
#include "../math/Vec3.h"
#include "../math/Triangle.h"

namespace mgodpl {
	/**
		 * An extremely simple indexed triangle mesh.
		 */
	struct Mesh {
		std::vector<math::Vec3d> vertices{};
		std::vector<math::Vec3i> faces{};

		/**
		 * Get a Triangle defined in terms of three Vec3d vertices based on the face index.
		 */
		inline math::Triangle triangle(int face_index) const {
			return math::Triangle(vertices[faces[face_index].x()],
								  vertices[faces[face_index].y()],
								  vertices[faces[face_index].z()]);
		}
	};

}


#endif //MGODPL_MESH_H
