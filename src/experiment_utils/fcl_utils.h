// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/21/23.
//

#ifndef MGODPL_FCL_UTILS_H
#define MGODPL_FCL_UTILS_H

#include <memory>
#include "../planning/Mesh.h"

namespace fcl {
	template <typename S>
	class BVHModel;
	template <typename S>
	struct OBB;
	using OBBd = OBB<double>;
}

namespace mgodpl::fcl_utils {

	/**
	 * @brief Converts a mesh from the ROS2 message format to FCL's BVHModel.
	 *
	 * This function takes a Mesh message from ROS2, extracts its vertices and triangles,
	 * and constructs a BVHModel<fcl::OBBd> using the FCL library.
	 *
	 * @param shape The input mesh in ROS2 message format (Mesh).
	 * @return A shared pointer to the constructed BVHModel<fcl::OBBd>.
	 */
	std::shared_ptr<fcl::BVHModel<fcl::OBBd> > meshToFclBVH(const Mesh &shape);

}

#endif //MGODPL_FCL_UTILS_H
