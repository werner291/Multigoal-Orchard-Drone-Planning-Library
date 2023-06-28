// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 28-6-23.
//

#ifndef MGODPL_TOPOLOGICAL_SPACE_H
#define MGODPL_TOPOLOGICAL_SPACE_H

namespace mgodpl {

	/**
	 * For a given space, defines the type of a point in that space.
	 * @tparam Space 		The type of the space.
	 */
	template<typename Space>
	struct space_point_t {
		using type = typename Space::SpacePoint;
	};

	/**
	 * For a given space, defines the type of a path in that space.
	 */
	template<typename Space>
	struct space_path_t {
		using type = typename Space::SpacePath;
	};

}

#endif //MGODPL_TOPOLOGICAL_SPACE_H
