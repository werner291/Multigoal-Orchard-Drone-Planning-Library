// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-6-23.
//

#ifndef MGODPL_PATH_H
#define MGODPL_PATH_H

#include <vector>

namespace mgodpl {

	template<typename Path>
	Path path_reverse(Path path);

	/**
	 * A variadic template function that concatenates a number of paths.
	 * @tparam Path 		The path type.
	 */
	template<typename Path>
	Path path_concatenate(Path paths...);

	template<typename Path>
	struct path_configuration_t {
		using type = typename Path::Configuration;
	};

}

#endif //MGODPL_PATH_H
